"""Persistent goal tracking in world XY; independent of ROS and simulator APIs."""

import math
import numpy as np
from .settings import NavigationSettings
from .curves import speed_profile
from .braking import stopping_envelope


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

    No target (Hold/blocked/settled) or a pacing gap freezes the reference and
    clears its rate. Healthy callbacks between clock samples preserve both;
    they consume no simulation time. The caller checks clock freshness.
    """
    if desired is None or not math.isfinite(dt) or not 0 <= dt <= 0.5:
        return yaw, 0.0
    if not all(math.isfinite(v) for v in (yaw, rate, desired, maximum_rate, acceleration)):
        raise ValueError("Yaw inputs must be finite")
    if maximum_rate <= 0 or acceleration <= 0:
        raise ValueError("Yaw rate and acceleration must be positive")
    if dt == 0:
        return yaw, rate
    error = math.remainder(desired - yaw, 2 * math.pi)
    wanted = float(np.clip(2.0 * error, -maximum_rate, maximum_rate))
    rate = float(np.clip(wanted, rate - acceleration * dt, rate + acceleration * dt))
    return math.remainder(yaw + rate * dt, 2 * math.pi), rate


def acceleration_feedforward(previous, current, dt, maximum):
    """Differentiate validated vector velocity; omit discontinuous commands."""
    if not math.isfinite(dt) or not 0 < dt <= 0.5:
        return None
    acceleration = (np.asarray(current, dtype=float) - np.asarray(previous, dtype=float)) / dt
    if (
        acceleration.shape != (2,)
        or not np.isfinite(acceleration).all()
        or np.linalg.norm(acceleration) > maximum + 1e-7
    ):
        return None
    return acceleration


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

    def world_velocity(self, ned_velocity):
        return self.rotation @ np.asarray(ned_velocity, dtype=float)[::-1]

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
        self.prepared_route = None
        self.braking_envelope = None
        self.braking_settled_s = 0.0
        self.braking_cause = None
        self._start_speed_decision()

    def _start_speed_decision(self, dt=None, current=None, heading=None):
        # Replace, never mutate the preceding tick's object: history/reporting
        # workers retain snapshots while the control callback moves on.
        self.speed_decision = dict(
            schema=1,
            dt_s=dt,
            reference_before_m_s=float(self.speed),
            requested_cruise_m_s=float(self.settings.cruise_m_s),
            selected_speed_m_s=None,
            expected_speed_m_s=float(self.speed),
            current_speed_m_s=(
                float(current) if current is not None and math.isfinite(current) else None
            ),
            caps_m_s={},
            reductions=[],
            dominant_limit=None,
            yaw_error_deg=(
                math.degrees(math.remainder(heading, 2 * math.pi))
                if heading is not None and math.isfinite(heading)
                else None
            ),
            acceleration_limited=False,
            mode="not_evaluated",
        )

    def _speed_cap(self, name, value, previous):
        value = float(value)
        cap = min(previous, value)
        self.speed_decision["caps_m_s"][name] = value
        if cap < previous - 1e-9:
            self.speed_decision["reductions"].append(
                dict(source=name, from_m_s=float(previous), to_m_s=float(cap))
            )
            self.speed_decision["dominant_limit"] = name
        return cap

    def _finish_speed_decision(self, mode="following"):
        data = self.speed_decision
        before, after, dt = data["reference_before_m_s"], float(self.speed), data["dt_s"]
        change = after - before
        data.update(
            expected_speed_m_s=after,
            reference_acceleration_m_s2=change / dt if dt is not None and dt > 0 else None,
            action=(
                "waiting_for_clock"
                if dt == 0
                else (
                    "decelerating"
                    if change < -1e-6
                    else (
                        "accelerating" if change > 1e-6 else "stopped" if after < 1e-8 else "steady"
                    )
                )
            ),
            mode=mode,
            phase=self.phase,
            reason=self.reason,
            ceiling_binding=(
                data["selected_speed_m_s"] is not None
                and (
                    data["selected_speed_m_s"] < before - 1e-6
                    or abs(after - data["selected_speed_m_s"]) < 1e-6
                )
            ),
        )

    def speed_report(self, current, published):
        """Pair a decision snapshot with the latest measured and sent speeds."""
        return dict(
            self.speed_decision,
            expected_speed_m_s=float(self.speed),
            current_speed_m_s=current,
            published_feedforward_speed_m_s=float(published),
            phase=self.phase,
            reason=self.reason,
        )

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
        self.prepared_route = path if hasattr(path, "limits") and hasattr(path, "safe") else None
        self.limits = (
            self.prepared_route.limits.copy()
            if self.prepared_route is not None
            else speed_profile(self.path, self.settings.acceleration_m_s2, self.settings.cruise_m_s)
        )
        self.tracking_scale = 1.0
        self.turn_stage = None
        self.braking_envelope = None
        self.braking_settled_s = 0.0
        self.braking_cause = None
        self._start_speed_decision()

    def hold(self, xy, reason="Operator hold", phase="HOLD"):
        self._start_speed_decision()
        self.target = np.array(xy, dtype=float)
        self.speed = 0.0
        self.velocity = np.zeros(2)
        self.phase = phase
        self.reason = reason
        if phase == "COMPLETE":
            self.path = []
            self.index = 1
            self.goal = None
            self.prepared_route = None
        self.turn_stage = None
        self.braking_envelope = None
        self.braking_settled_s = 0.0
        self.braking_cause = None
        self.speed_decision.update(
            selected_speed_m_s=0.0, dominant_limit="stop", stop_reason=reason
        )
        self.speed_decision["caps_m_s"]["stop"] = 0.0
        self._finish_speed_decision("hold")

    def retarget(self, path):
        """Replace a validated route without jumping the streamed reference."""
        target = self.target.copy()
        speed = self.speed
        velocity = self.velocity.copy()
        turn_stage = self.turn_stage
        braking_envelope = self.braking_envelope
        braking_settled_s = self.braking_settled_s
        braking_cause = self.braking_cause
        tracking_scale = self.tracking_scale
        if hasattr(path, "with_start"):
            path = path.with_start(target)
        self.begin(path)
        self.target = target
        self.path[0] = target.copy()
        if self.prepared_route is None:
            self.limits = speed_profile(
                self.path, self.settings.acceleration_m_s2, self.settings.cruise_m_s
            )
        self.speed = speed
        self.velocity = velocity
        self.turn_stage = turn_stage
        self.braking_envelope = braking_envelope
        self.braking_settled_s = braking_settled_s
        self.braking_cause = braking_cause
        self.tracking_scale = tracking_scale
        self.reason = "Following updated frontier route"
        self._start_speed_decision()

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
        measured_velocity=None,
        braking_safe=None,
        tracking_reference=None,
    ):
        if self.phase != "NAVIGATING":
            return
        if not math.isfinite(dt) or not 0 <= dt <= 0.5:
            self.hold(pose, "Simulation pacing gap", "PAUSED")
            return
        self._start_speed_decision(dt, measured_speed, heading_error)
        error = float(np.linalg.norm(np.asarray(pose) - self.target))
        # /clock and telemetry can arrive in batches. Compare the measured
        # present position with this reference's present-time prediction, not
        # solely with the target from the previous simulation instant.
        prediction_dt = dt if tracking_dt is None else tracking_dt
        if not math.isfinite(prediction_dt) or not 0 <= prediction_dt <= 0.5:
            self.hold(pose, "Invalid tracking time interval", "PAUSED")
            return
        if heading_error is not None and not math.isfinite(heading_error):
            self.hold(pose, "Heading unavailable for turn coordination", "PAUSED")
            return
        predicted = (
            self.target + self.velocity * prediction_dt
            if tracking_reference is None
            else np.asarray(tracking_reference, dtype=float)
        )
        if predicted.shape != (2,) or not np.isfinite(predicted).all():
            self.hold(pose, "Invalid aligned tracking reference", "PAUSED")
            return
        tracking_error = float(np.linalg.norm(np.asarray(pose) - predicted))
        self.tracking = dict(
            error_m=tracking_error,
            previous_target_error_m=error,
            prediction_dt_s=prediction_dt,
            predicted_reference_xy=predicted.tolist(),
            reference_xy=self.target.tolist(),
            measured_xy=list(map(float, pose)),
            reference_velocity_xy=self.velocity.tolist(),
            reference_time_aligned=tracking_reference is not None,
        )
        self.speed_decision.update(
            tracking_error_m=tracking_error,
            reference_position_error_m=error,
            prediction_dt_s=prediction_dt,
        )
        # Consume reached/nearby samples before testing the onward direction.
        # A zero-length vector (or a tiny cross-track correction toward the old
        # sample) is not a major turn. Keep the final goal and true onward turns
        # subject to the normal stopping, footprint and acceleration checks.
        if self.braking_envelope is None:
            while (
                self.index < len(self.path) - 1
                and np.linalg.norm(self.path[self.index] - self.target) < 0.05
            ):
                self.index += 1
        # Live handovers have a measured velocity vector and a complete stopping
        # envelope checker. Decide to brake BEFORE the scalar tracking latch or
        # the target-lead clamp can turn a feasible deceleration into a pause.
        if measured_velocity is not None and braking_safe is not None:
            measured_velocity = np.asarray(measured_velocity, dtype=float)
            if measured_velocity.shape != (2,) or not np.isfinite(measured_velocity).all():
                self.hold(pose, "Measured velocity unavailable for braking", "PAUSED")
                return
            vector = self.path[self.index] - self.target
            length = float(np.linalg.norm(vector))
            direction = vector / length if length > 1e-8 else np.zeros(2)
            ref_speed = float(np.linalg.norm(self.velocity))
            actual_speed = float(np.linalg.norm(measured_velocity))
            self.speed_decision["current_speed_m_s"] = actual_speed
            major = (ref_speed > 0.15 and self.velocity @ direction < 0.7 * ref_speed) or (
                actual_speed > 0.15 and measured_velocity @ direction < 0.7 * actual_speed
            )
            if self.braking_envelope is not None or major or tracking_error > 0.25:
                if self.braking_envelope is None:
                    self.braking_cause = "travel_direction_change" if major else "tracking_error"
                self.braking_tick(dt, pose, measured_velocity, hits, braking_safe, heading_error)
                return
        if tracking_error > 0.35:
            self.hold(pose, "Trajectory tracking error exceeded 0.35 m", "PAUSED")
            return
        if self.index == len(self.path) - 1 and np.linalg.norm(self.target - self.goal) < 0.015:
            self.target = self.goal.copy()
            self.phase = "SETTLING"
            self.reason = "Settling at destination"
            self.speed_decision.update(selected_speed_m_s=0.0, dominant_limit="goal")
            self._finish_speed_decision("settling")
            return
        vector = self.path[self.index] - self.target
        clearance = clearance_ahead(pose, vector, hits)
        cfg = self.settings
        # An endpoint-free strip may lead through a mapped doorway. Absence
        # alone is insufficient; the caller must certify the remaining route
        # against a fresh map with the complete vehicle footprint observed.
        cruise_cap = cfg.cruise_m_s if cruise is None else min(cruise, cfg.cruise_m_s)
        self.speed_decision.update(
            requested_cruise_m_s=float(cruise_cap), clearance_m=clearance, dominant_limit="cruise"
        )
        self.speed_decision["caps_m_s"]["cruise"] = float(cruise_cap)
        if clearance is None and observed_corridor:
            # The whole remaining polyline was certified against a fresh map.
            # Goal and bend braking below still apply; invent no range return.
            cap = cruise_cap
        else:
            approach_cap = approach_speed(
                clearance,
                cruise_cap,
                cfg.stop_distance_m,
                cfg.slow_distance_m,
                cfg.minimum_approach_m_s,
            )
            stopping_cap = braking_speed(
                clearance, cfg.stop_distance_m, cfg.acceleration_m_s2, cfg.braking_reaction_s
            )
            cap = self._speed_cap("obstacle_approach", approach_cap, cruise_cap)
            cap = self._speed_cap("obstacle_braking", stopping_cap, cap)
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
        self.speed_decision.update(
            distance_to_waypoint_m=float(remaining), waypoint_exit_speed_m_s=float(exit_speed)
        )
        if exit_speed <= 1e-8:
            cap = self._speed_cap(
                "goal_braking" if self.index == len(self.path) - 1 else "corner_braking",
                braking_speed(remaining, 0.01, cfg.acceleration_m_s2, cfg.braking_reaction_s),
                cap,
            )
            if self.index < len(self.path) - 1:
                cap = self._speed_cap(
                    "corner_approach",
                    max(
                        cfg.turn_speed_m_s,
                        braking_speed(
                            remaining, 0.4, cfg.acceleration_m_s2, cfg.braking_reaction_s
                        ),
                    ),
                    cap,
                )
        else:
            cap = self._speed_cap(
                "route_profile",
                math.sqrt(exit_speed**2 + 2 * 0.6 * cfg.acceleration_m_s2 * remaining),
                cap,
            )
        direction = vector / remaining if remaining > 1e-8 else np.zeros(2)
        offset = self.target - np.asarray(pose)
        along_error = float(offset @ direction)
        cross_error = math.sqrt(max(0.0, error**2 - along_error**2))
        # Live control uses the same prediction as the hard guard. Raw and predicted errors
        # used to disagree by an entire batched clock interval at cruise speed.
        predicted_offset = predicted - np.asarray(pose)
        predicted_along = float(predicted_offset @ direction)
        predicted_cross = math.sqrt(max(0.0, tracking_error**2 - predicted_along**2))
        live_tracking = measured_velocity is not None and braking_safe is not None
        # With a velocity measurement, govern forward lead by relative motion
        # below. Scaling cruise alone can leave its ceiling ABOVE the reference
        # speed, allowing continued acceleration while the lead grows.
        lag = max(0.0, predicted_cross) if live_tracking else max(0.0, along_error, cross_error)
        requested_scale = float(np.clip((0.25 - lag) / 0.15, 0, 1))
        self.tracking_scale += float(np.clip(requested_scale - self.tracking_scale, -dt, dt))
        before_tracking = cap
        cap = self._speed_cap("tracking", cap * self.tracking_scale, cap)
        if live_tracking:
            measured_forward = max(0.0, float(measured_velocity @ direction))
            # Reserve 5 cm before the 25 cm guard. Bound the closing speed so
            # reaction travel plus relative braking fits in the remaining gap.
            # This tolerates a bounded, steady lag without forcing a full stop.
            forward_room = math.sqrt(max(0.0, 0.20**2 - predicted_cross**2))
            lead_budget = max(0.0, forward_room - max(0.0, predicted_along))
            closing_limit = braking_speed(
                lead_budget, 0.0, cfg.acceleration_m_s2, cfg.braking_reaction_s
            )
            cap = self._speed_cap("tracking_response", measured_forward + closing_limit, cap)
            self.speed_decision.update(
                measured_forward_speed_m_s=measured_forward,
                closing_speed_m_s=float(self.velocity @ direction) - measured_forward,
                allowed_closing_speed_m_s=closing_limit,
                tracking_lead_budget_m=lead_budget,
            )
        effective_tracking_scale = cap / before_tracking if before_tracking > 0 else 1.0
        self.speed_decision.update(
            along_lead_m=along_error,
            cross_error_m=cross_error,
            predicted_along_lead_m=predicted_along,
            predicted_cross_error_m=predicted_cross,
            tracking_scale=float(effective_tracking_scale),
        )
        self.tracking.update(
            along_error_m=along_error,
            cross_error_m=cross_error,
            predicted_along_lead_m=predicted_along,
            predicted_cross_error_m=predicted_cross,
            speed_scale=float(effective_tracking_scale),
        )
        yaw_limited = False
        if heading_error is not None:
            if not math.isfinite(heading_error):
                self.hold(pose, "Heading unavailable for turn coordination", "PAUSED")
                return
            # Yaw controls facing, independently of world-frame translation.
            # Large heading error caps travel speed but never requests a stop.
            angle = abs(math.remainder(heading_error, 2 * math.pi))
            yaw_limited = angle > math.radians(30)
            if yaw_limited:
                cap = self._speed_cap("yaw", cfg.yaw_misaligned_speed_m_s, cap)
        previous_speed = float(np.linalg.norm(self.velocity))
        actual_speed = previous_speed if measured_speed is None else measured_speed
        if not math.isfinite(actual_speed) or actual_speed < 0:
            self.hold(pose, "Measured speed unavailable for turn coordination", "PAUSED")
            return
        major_change = (
            previous_speed > 0.15 and float(self.velocity @ direction) < 0.7 * previous_speed
        )
        if self.turn_stage is None and major_change:
            self.turn_stage = "BRAKING"
        if self.turn_stage in ("BRAKING", "ALIGNING") and max(previous_speed, actual_speed) <= 0.15:
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
            self._speed_cap("travel_direction_change", 0.0, cap)
        else:
            desired_velocity = direction * cap
        self.speed_decision["selected_speed_m_s"] = float(np.linalg.norm(desired_velocity))
        if measured_speed is not None:
            self.speed_decision["current_speed_m_s"] = float(actual_speed)
        change = desired_velocity - self.velocity
        change_norm = float(np.linalg.norm(change))
        if change_norm > cfg.acceleration_m_s2 * dt and change_norm > 0:
            self.speed_decision["acceleration_limited"] = True
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
        if measured_velocity is not None and braking_safe is not None:
            preview = stopping_envelope(
                self.target + velocity * dt,
                velocity,
                pose,
                measured_velocity,
                cfg.acceleration_m_s2,
                cfg.braking_reaction_s,
            )
            if speed * dt > headroom + 1e-9 or not braking_safe(preview):
                self.braking_cause = (
                    "reference_lead" if speed * dt > headroom + 1e-9 else "stopping_space"
                )
                # Stop proactively while current stopping space is still clear.
                # Never cut the velocity instantaneously to enforce lead distance.
                self.braking_tick(dt, pose, measured_velocity, hits, braking_safe, heading_error)
                return
        travel = min(speed * dt, headroom)
        self.speed_decision["lead_clamped"] = bool(speed * dt > headroom + 1e-9)
        # Only clamp to the waypoint when moving toward it, not while braking
        # in the old direction after a route switch.
        if float(travel_direction @ direction) > 0.99:
            self.speed_decision["waypoint_clamped"] = bool(travel > remaining + 1e-9)
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
                "Speed capped while yaw aligns with route"
                if yaw_limited
                else (
                    "Slowing for tracking lag"
                    if effective_tracking_scale < 0.999
                    else "Following planned route"
                )
            )
        )
        if self.index == len(self.path) - 1 and np.linalg.norm(self.target - self.goal) < 0.015:
            self.target = self.goal.copy()
            self.phase = "SETTLING"
            self.reason = "Settling at destination"
            self.speed = 0.0
            self.velocity = np.zeros(2)
            self._speed_cap("goal_reached", 0.0, cap)
            self.speed_decision["selected_speed_m_s"] = 0.0
        self._finish_speed_decision(
            "settling" if self.phase == "SETTLING" else "braking" if turning else "following"
        )

    def remaining_path(self, pose):
        if self.braking_envelope is not None:
            return [list(map(float, pose)), self.braking_envelope[3].tolist()]
        return [list(map(float, pose))] + [p.tolist() for p in self.path[self.index :]]

    def braking_tick(self, dt, pose, measured_velocity, hits, braking_safe, heading_error):
        """Stop continuously in certified space, then ask for an onward route.

        A geometric route from the old XY position is deliberately not joined
        here. The planner will run again from the settled position. Tracking
        discrepancy stays observable but is not a scalar latch in this maneuver.
        """
        cfg = self.settings
        self.speed_decision.update(
            selected_speed_m_s=0.0,
            dominant_limit=self.braking_cause or "reserved_braking",
            braking_cause=self.braking_cause or "reserved_braking",
        )
        self.speed_decision["caps_m_s"]["braking"] = 0.0
        envelope = stopping_envelope(
            self.target,
            self.velocity,
            pose,
            measured_velocity,
            cfg.acceleration_m_s2,
            cfg.braking_reaction_s,
        )
        for velocity in (self.velocity, measured_velocity):
            clearance = clearance_ahead(pose, velocity, hits)
            reserve = np.linalg.norm(velocity) * (
                cfg.braking_reaction_s + np.linalg.norm(velocity) / (2 * cfg.acceleration_m_s2)
            )
            if clearance is not None and clearance <= reserve + cfg.stop_distance_m:
                self.hold(pose, "Obstacle intersects reserved braking distance", "BLOCKED")
                return
        if not braking_safe(envelope):
            self.hold(
                pose, "Braking envelope is blocked or not observed body-free space", "BLOCKED"
            )
            return
        self.braking_envelope = envelope
        speed = float(np.linalg.norm(self.velocity))
        next_speed = max(0.0, speed - cfg.acceleration_m_s2 * dt)
        self.speed_decision["acceleration_limited"] = next_speed > 1e-8 and dt > 0
        velocity = self.velocity * (next_speed / speed) if speed > 1e-8 else np.zeros(2)
        # No position clamp after acceleration limiting: reference displacement
        # and feedforward use exactly the same velocity and simulation interval.
        self.target = self.target + velocity * dt
        self.velocity = velocity
        self.speed = next_speed
        actual_speed = float(np.linalg.norm(measured_velocity))
        self.speed_decision["current_speed_m_s"] = actual_speed
        settled = (
            next_speed < 1e-8
            and actual_speed <= 0.15
            and np.linalg.norm(self.target - pose) <= 0.15
        )
        # A stopped reference is holding position while the vehicle settles;
        # crossing a telemetry threshold must not report renewed braking.
        self.turn_stage = "SETTLING" if next_speed < 1e-8 else "BRAKING"
        self.braking_settled_s = self.braking_settled_s + dt if settled else 0.0
        self.reason = (
            "Holding stopping point while measured motion settles"
            if self.turn_stage == "SETTLING"
            else "Following reserved braking segment before route handover"
        )
        self.tracking.update(
            turn_stage=self.turn_stage,
            measured_speed_m_s=actual_speed,
            tracking_warning=self.tracking["error_m"] > 0.35,
            braking_envelope_xy=envelope.tolist(),
            braking_reference_speed_m_s=next_speed,
        )
        # Planning from a stopped reference does not require a dwell or a yaw
        # aligned to the provisional goal bearing. The navigator still gates
        # planner launch on its stricter stable-hover check; the fresh route
        # supplies the heading and yaw speed limits before flight resumes.
        if settled:
            self.phase = "PLANNING"
            self.reason = "Braking complete; replanning onward route from settled position"
        self._finish_speed_decision("braking" if next_speed > 1e-8 else "settling")

    def heading_direction(self):
        if (
            self.phase != "NAVIGATING"
            or self.target is None
            or not 0 <= self.index < len(self.path)
        ):
            return None
        if self.braking_envelope is not None:
            return self.goal - self.target
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
