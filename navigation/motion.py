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

    def tick(self, dt, pose, hits, cruise=None):
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
        cap = approach_speed(
            clearance,
            cfg.cruise_m_s if cruise is None else cruise,
            cfg.stop_distance_m,
            cfg.slow_distance_m,
            cfg.minimum_approach_m_s,
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
        elif remaining < 0.4:
            cap = min(cap, cfg.turn_speed_m_s)  # Slow around polyline bends.
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
