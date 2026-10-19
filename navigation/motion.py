"""Persistent goal tracking in world XY; independent of ROS and simulator APIs."""

import math
import numpy as np
from .settings import NavigationSettings
from .curves import speed_profile


def approach_speed(distance, cruise=0.5, stop=0.65, slow=1.0, minimum=0.08):
    if distance is None or not math.isfinite(distance) or distance <= stop:
        return 0.0
    if distance >= slow:
        return cruise
    return minimum + (cruise - minimum) * (distance - stop) / (slow - stop)


def braking_speed(distance, stop, deceleration, reaction_s):
    """Reference speed whose reaction travel + braking distance fits before stop.

    A kinematic planning bound, not a measurement of PX4's physical stopping power.
    """
    if distance is None or not math.isfinite(distance) or distance <= stop:
        return 0.0
    if (
        not all(math.isfinite(v) for v in (stop, deceleration, reaction_s))
        or deceleration <= 0
        or reaction_s < 0
    ):
        raise ValueError("Invalid braking model")
    margin = distance - stop
    reaction_term = deceleration * reaction_s
    return (
        2
        * deceleration
        * margin
        / (math.sqrt(reaction_term**2 + 2 * deceleration * margin) + reaction_term)
    )


def clearance_ahead(pose, direction, hits, half_width=0.45):
    direction = np.array(direction, dtype=float)
    norm = np.linalg.norm(direction)
    if norm < 1e-8:
        return None
    direction /= norm
    delta = np.asarray(hits) - np.asarray(pose)
    along = delta @ direction
    sideways = np.abs(delta[:, 0] * direction[1] - delta[:, 1] * direction[0])
    forward = (along > 0.05) & (sideways <= half_width)
    return float(along[forward].min()) if forward.any() else None


def smooth_yaw(yaw, rate, desired, dt, maximum_rate, acceleration):
    """Follow the shortest angular error with bounded rate and acceleration.

    No target (Hold/blocked/settled), stopped simulation, or a pacing gap freezes
    the heading reference. The caller supplies delivered simulation time.
    """
    if desired is None or not math.isfinite(dt) or not 0 < dt <= 0.5:
        return yaw, 0.0
    if not all(math.isfinite(v) for v in (yaw, rate, desired, maximum_rate, acceleration)):
        raise ValueError("Yaw inputs must be finite")
    if maximum_rate <= 0 or acceleration <= 0:
        raise ValueError("Yaw rate and acceleration must be positive")
    error = math.remainder(desired - yaw, 2 * math.pi)
    wanted = float(np.clip(2.0 * error, -maximum_rate, maximum_rate))
    rate = float(np.clip(wanted, rate - acceleration * dt, rate + acceleration * dt))
    return math.remainder(yaw + rate * dt, 2 * math.pi), rate


class FrameAlignment:
    def __init__(self, world_xy, world_yaw, local_ned_xy, local_yaw, position_yaw=0.0):
        # The installed bridge generates GPS from ENU_TO_NED exactly. Its
        # position axes share world North/East; an estimator heading bias must
        # not rotate metre-scale position goals. Keep that bias only for the
        # independent attitude/frame consistency check.
        self.heading_bias = math.remainder(world_yaw - (math.pi / 2 - local_yaw), 2 * math.pi)
        self.angle = float(position_yaw)
        c, s = math.cos(self.angle), math.sin(self.angle)
        self.rotation = np.array([[c, -s], [s, c]])
        self.offset = np.array(world_xy) - self.rotation @ np.array(local_ned_xy)[::-1]

    def world(self, ned_xy):
        return self.rotation @ np.array(ned_xy)[::-1] + self.offset

    def ned(self, world_xy):
        return (self.rotation.T @ (np.array(world_xy) - self.offset))[::-1]

    def ned_velocity(self, world_velocity):
        # Velocities rotate/swap axes; position offsets must never be applied.
        return (self.rotation.T @ np.asarray(world_velocity, dtype=float))[::-1]

    def yaw_ned(self, world_direction):
        direction = np.asarray(world_direction, dtype=float)
        if (
            direction.shape != (2,)
            or not np.isfinite(direction).all()
            or np.linalg.norm(direction) < 1e-6
        ):
            raise ValueError("Heading needs a finite nonzero world direction")
        bearing = math.atan2(direction[1], direction[0])
        # True world yaw = pi/2 - estimator heading + captured heading bias.
        # This attitude correction must not rotate position coordinates.
        return math.remainder(math.pi / 2 - bearing + self.heading_bias, 2 * math.pi)


class RouteFollower:
    def __init__(self, settings=None):
        self.settings = (settings or NavigationSettings()).validate()
        self.path = []
        self.index = 1
        self.target = None
        self.goal = None
        self.speed = 0.0
        self.velocity = np.zeros(2)
        self.phase = "HOLD"
        self.reason = "Choose a destination"
        self.tracking = None
        self.turn_stage = None

    def begin(self, path):
        self.path = [np.array(p, dtype=float) for p in path]
        if (
            len(self.path) < 2
            or any(p.shape != (2,) for p in self.path)
            or not np.isfinite(self.path).all()
        ):
            raise ValueError("A finite XY route is required")
        self.index = 1
        self.target = self.path[0].copy()
        self.goal = self.path[-1].copy()
        self.speed = 0.0
        self.velocity = np.zeros(2)
        self.phase = "NAVIGATING"
        self.reason = "Following planned route"
        self.limits = speed_profile(
            self.path, self.settings.acceleration_m_s2, self.settings.cruise_m_s
        )
        self.tracking_scale = 1.0
        self.turn_stage = None

    def hold(self, xy, reason="Operator hold", phase="HOLD"):
        self.target = np.array(xy, dtype=float)
        self.speed = 0.0
        self.velocity = np.zeros(2)
        self.phase = phase
        self.reason = reason
        self.turn_stage = None

    def retarget(self, path):
        """Replace a validated route without jumping the streamed reference."""
        target = self.target.copy()
        speed = self.speed
        velocity = self.velocity.copy()
        turn_stage = self.turn_stage
        tracking_scale = self.tracking_scale
        self.begin(path)
        self.target = target
        self.path[0] = target.copy()
        self.limits = speed_profile(
            self.path, self.settings.acceleration_m_s2, self.settings.cruise_m_s
        )
        self.speed = speed
        self.velocity = velocity
        self.turn_stage = turn_stage
        self.tracking_scale = tracking_scale
        self.reason = "Following updated frontier route"

    def tick(
        self,
        dt,
        pose,
        hits,
        cruise=None,
        *,
        observed_corridor=False,
        heading_error=None,
        motion_safe=None,
        tracking_dt=None,
        measured_speed=None,
    ):
        if self.phase != "NAVIGATING":
            return
        if not math.isfinite(dt) or not 0 <= dt <= 0.5:
            self.hold(pose, "Simulation pacing gap", "PAUSED")
            return
        error = float(np.linalg.norm(np.asarray(pose) - self.target))
        # /clock and telemetry can arrive in batches. Compare the measured
        # present position with this reference's present-time prediction, not
        # solely with the target from the previous simulation instant.
        prediction_dt = dt if tracking_dt is None else tracking_dt
        if not math.isfinite(prediction_dt) or not 0 <= prediction_dt <= 0.5:
            self.hold(pose, "Invalid tracking time interval", "PAUSED")
            return
        predicted = self.target + self.velocity * prediction_dt
        tracking_error = float(np.linalg.norm(np.asarray(pose) - predicted))
        self.tracking = dict(
            error_m=tracking_error,
            previous_target_error_m=error,
            prediction_dt_s=prediction_dt,
            predicted_reference_xy=predicted.tolist(),
            reference_xy=self.target.tolist(),
            measured_xy=list(map(float, pose)),
            reference_velocity_xy=self.velocity.tolist(),
        )
        if tracking_error > 0.35:
            self.hold(pose, "Trajectory tracking error exceeded 0.35 m", "PAUSED")
            return
        if self.index == len(self.path) - 1 and np.linalg.norm(self.target - self.goal) < 0.015:
            self.target = self.goal.copy()
            self.phase = "SETTLING"
            self.reason = "Settling at destination"
            return
        vector = self.path[self.index] - self.target
        if np.linalg.norm(vector) < 0.05 and self.index < len(self.path) - 1:
            self.index += 1
            vector = self.path[self.index] - self.target
        clearance = clearance_ahead(pose, vector, hits)
        cfg = self.settings
        # An endpoint-free strip may lead through a mapped doorway. Absence
        # alone is insufficient; the caller must certify the remaining route
        # against a fresh map with the complete vehicle footprint observed.
        cruise_cap = cfg.cruise_m_s if cruise is None else min(cruise, cfg.cruise_m_s)
        if clearance is None and observed_corridor:
            # The whole remaining polyline was certified against a fresh map.
            # Goal and bend braking below still apply; invent no range return.
            cap = cruise_cap
        else:
            cap = min(
                approach_speed(
                    clearance,
                    cruise_cap,
                    cfg.stop_distance_m,
                    cfg.slow_distance_m,
                    cfg.minimum_approach_m_s,
                ),
                braking_speed(
                    clearance, cfg.stop_distance_m, cfg.acceleration_m_s2, cfg.braking_reaction_s
                ),
            )
        if cap <= 0:
            self.hold(
                pose,
                (
                    "No observed corridor ahead"
                    if clearance is None
                    else f"Obstacle ahead at {clearance:.2f} m"
                ),
                "BLOCKED",
            )
            return
        remaining = np.linalg.norm(vector)
        exit_speed = self.limits[self.index]
        if exit_speed <= 1e-8:
            cap = min(
                cap,
                braking_speed(remaining, 0.01, cfg.acceleration_m_s2, cfg.braking_reaction_s),
            )
            if self.index < len(self.path) - 1:
                cap = min(
                    cap,
                    max(
                        cfg.turn_speed_m_s,
                        braking_speed(
                            remaining, 0.4, cfg.acceleration_m_s2, cfg.braking_reaction_s
                        ),
                    ),
                )
        else:
            cap = min(cap, math.sqrt(exit_speed**2 + 2 * 0.6 * cfg.acceleration_m_s2 * remaining))
        direction = vector / remaining if remaining > 1e-8 else np.zeros(2)
        offset = self.target - np.asarray(pose)
        along_error = float(offset @ direction)
        cross_error = math.sqrt(max(0.0, error**2 - along_error**2))
        # Slow a leading reference or lateral divergence before the hard gate.
        # A drone ahead of a braking reference must NOT make that reference
        # slower still. Use the signed error, rather than its magnitude alone.
        lag = max(0.0, along_error, cross_error)
        requested_scale = float(np.clip((0.25 - lag) / 0.15, 0, 1))
        self.tracking_scale += float(np.clip(requested_scale - self.tracking_scale, -dt, dt))
        cap *= self.tracking_scale
        self.tracking.update(
            along_error_m=along_error, cross_error_m=cross_error, speed_scale=self.tracking_scale
        )
        yaw_scale = 1.0
        if heading_error is not None:
            if not math.isfinite(heading_error):
                self.hold(pose, "Heading unavailable for turn coordination", "PAUSED")
                return
            # Shallow course changes retain speed; brake for substantial turns.
            angle = abs(math.remainder(heading_error, 2 * math.pi))
            yaw_scale = float(np.clip((math.radians(90) - angle) / math.radians(60), 0, 1))
            yaw_scale = yaw_scale * yaw_scale * (3 - 2 * yaw_scale)
        cap *= yaw_scale
        previous_speed = float(np.linalg.norm(self.velocity))
        actual_speed = previous_speed if measured_speed is None else measured_speed
        if not math.isfinite(actual_speed) or actual_speed < 0:
            self.hold(pose, "Measured speed unavailable for turn coordination", "PAUSED")
            return
        major_change = (
            previous_speed > 0.15 and float(self.velocity @ direction) < 0.7 * previous_speed
        ) or (
            actual_speed > 0.15
            and heading_error is not None
            and abs(math.remainder(heading_error, 2 * math.pi)) >= math.radians(60)
        )
        if self.turn_stage is None and major_change:
            self.turn_stage = "BRAKING"
        if self.turn_stage == "BRAKING" and max(previous_speed, actual_speed) <= 0.15:
            self.turn_stage = "ALIGNING"
        if self.turn_stage == "ALIGNING" and (
            heading_error is None
            or abs(math.remainder(heading_error, 2 * math.pi)) <= math.radians(30)
        ):
            self.turn_stage = None
        turning = self.turn_stage is not None
        self.tracking.update(turn_stage=self.turn_stage, measured_speed_m_s=actual_speed)
        if turning:
            old_clearance = clearance_ahead(pose, self.velocity, hits)
            if old_clearance is not None and old_clearance <= cfg.stop_distance_m:
                self.hold(pose, "Obstacle inside stop distance along braking direction", "BLOCKED")
                return
            # Brake along the previous velocity instead of preserving its
            # magnitude while instantly pointing it at a different destination.
            desired_velocity = np.zeros(2)
        else:
            desired_velocity = direction * cap
        change = desired_velocity - self.velocity
        change_norm = float(np.linalg.norm(change))
        if change_norm > cfg.acceleration_m_s2 * dt and change_norm > 0:
            change *= cfg.acceleration_m_s2 * dt / change_norm
        velocity = self.velocity + change
        speed = float(np.linalg.norm(velocity))
        travel_direction = velocity / speed if speed > 1e-8 else np.zeros(2)
        # Bound reference lead when the physical position controller is slow.
        # The independent 0.35 m tracking guard remains in force.
        # Intersect the forward reference ray with the allowed sphere around
        # measured position. With velocity feedforward the drone may be slightly
        # ahead of the last target: the next reference may catch up without
        # spending that backward error as if it were forward lead.
        along = float(offset @ travel_direction)
        sideways_squared = max(0.0, float(offset @ offset) - along**2)
        headroom = (
            max(0.0, -along + math.sqrt(cfg.target_lead_m**2 - sideways_squared))
            if sideways_squared <= cfg.target_lead_m**2
            else 0.0
        )
        travel = min(speed * dt, headroom)
        # Only clamp to the waypoint when moving toward it, not while braking
        # in the old direction after a route switch.
        if float(travel_direction @ direction) > 0.99:
            travel = min(travel, remaining)
        candidate = self.target + travel_direction * travel
        if motion_safe is not None and not motion_safe(self.target, candidate):
            self.hold(pose, "Turn braking path is not observed body-free space", "BLOCKED")
            return
        if dt > 0:
            self.speed = travel / dt
            self.velocity = travel_direction * self.speed
        self.target = candidate
        self.reason = (
            "Braking to near-hover for route change"
            if self.turn_stage == "BRAKING"
            else (
                "Near-hover: aligning yaw before accelerating"
                if self.turn_stage == "ALIGNING"
                else (
                    "Slowing while yaw aligns with route"
                    if yaw_scale < 0.999
                    else (
                        "Slowing for tracking lag"
                        if self.tracking_scale < 0.999
                        else "Following planned route"
                    )
                )
            )
        )
        if self.index == len(self.path) - 1 and np.linalg.norm(self.target - self.goal) < 0.015:
            self.target = self.goal.copy()
            self.phase = "SETTLING"
            self.reason = "Settling at destination"
            self.speed = 0.0
            self.velocity = np.zeros(2)

    def remaining_path(self, pose):
        return [list(map(float, pose))] + [p.tolist() for p in self.path[self.index :]]

    def heading_direction(self):
        if (
            self.phase != "NAVIGATING"
            or self.target is None
            or not 0 <= self.index < len(self.path)
        ):
            return None
        # Look along the upcoming curve rather than chasing each dense sample.
        distance = max(0.3, min(0.8, self.speed * 0.5))
        point = self.target
        for next_point in self.path[self.index :]:
            length = float(np.linalg.norm(next_point - point))
            if length >= distance and length > 1e-8:
                return point + (next_point - point) * (distance / length) - self.target
            distance -= length
            point = next_point
        return point - self.target
