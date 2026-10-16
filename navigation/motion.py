"""Persistent goal tracking in world XY; independent of ROS and simulator APIs."""

import math
import numpy as np
from .settings import NavigationSettings


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
        self.phase = "HOLD"
        self.reason = "Choose a destination"

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
        self.phase = "NAVIGATING"
        self.reason = "Following planned route"

    def hold(self, xy, reason="Operator hold", phase="HOLD"):
        self.target = np.array(xy, dtype=float)
        self.speed = 0.0
        self.phase = phase
        self.reason = reason

    def retarget(self, path):
        """Replace a validated route without jumping the streamed reference."""
        target = self.target.copy()
        speed = self.speed
        self.begin(path)
        self.target = target
        self.path[0] = target.copy()
        self.speed = speed
        self.reason = "Following updated frontier route"

    def tick(self, dt, pose, hits, cruise=None, *, observed_corridor=False):
        if self.phase != "NAVIGATING":
            return
        if not math.isfinite(dt) or not 0 <= dt <= 0.5:
            self.hold(pose, "Simulation pacing gap", "PAUSED")
            return
        if np.linalg.norm(np.array(pose) - self.target) > 0.35:
            self.hold(pose, "Trajectory tracking error exceeded 0.35 m", "PAUSED")
            return
        if self.index == len(self.path) - 1 and np.linalg.norm(self.target - self.goal) < 0.015:
            self.target = self.goal.copy()
            self.phase = "SETTLING"
            self.reason = "Settling at destination"
            return
        vector = self.path[self.index] - self.target
        if np.linalg.norm(vector) < 0.03 and self.index < len(self.path) - 1:
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
        if self.index == len(self.path) - 1:
            cap = min(cap, math.sqrt(2 * cfg.acceleration_m_s2 * remaining))
        else:
            # Reach the existing 0.4 m bend zone at turn speed, rather than
            # first requesting deceleration after entering it at full cruise.
            cap = min(
                cap,
                max(
                    cfg.turn_speed_m_s,
                    braking_speed(remaining, 0.4, cfg.acceleration_m_s2, cfg.braking_reaction_s),
                ),
            )
        self.speed = (
            min(cap, self.speed + cfg.acceleration_m_s2 * dt)
            if cap >= self.speed
            else max(cap, self.speed - cfg.acceleration_m_s2 * dt)
        )
        # Bound reference lead when the physical position controller is slow.
        # The independent 0.35 m tracking guard remains in force.
        headroom = max(0.0, cfg.target_lead_m - float(np.linalg.norm(np.array(pose) - self.target)))
        travel = min(self.speed * dt, remaining, headroom)
        if dt > 0:
            self.speed = travel / dt
        if remaining > 1e-8:
            self.target += vector / remaining * travel
        if self.index == len(self.path) - 1 and np.linalg.norm(self.target - self.goal) < 0.015:
            self.target = self.goal.copy()
            self.phase = "SETTLING"
            self.reason = "Settling at destination"
            self.speed = 0.0

    def remaining_path(self, pose):
        return [list(map(float, pose))] + [p.tolist() for p in self.path[self.index :]]
