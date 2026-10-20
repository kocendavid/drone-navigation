"""Validated launch-time settings shared by mapper, follower and dashboard."""

from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class NavigationSettings:
    cruise_m_s: float = 0.5
    slow_distance_m: float = 1.0
    stop_distance_m: float = 0.65
    minimum_approach_m_s: float = 0.08
    acceleration_m_s2: float = 0.25
    braking_reaction_s: float = 0.5
    turn_speed_m_s: float = 0.15
    yaw_rate_deg_s: float = 45.0
    yaw_acceleration_deg_s2: float = 90.0
    yaw_misaligned_speed_m_s: float = 0.7
    target_lead_m: float = 0.28
    body_radius_m: float = 0.3
    obstacle_margin_m: float = 0.8
    near_range_m: float = 0.5
    exploration_max_goals: int = 20
    exploration_replan_interval_s: float = 2.0
    exploration_commit_horizon_s: float = 2.0
    floor_z_m: float = -0.76965
    map_resolution_m: float = 0.1
    map_width_cells: int = 120
    map_height_cells: int = 120
    map_origin_x_m: float = -6.0
    map_origin_y_m: float = -6.0
    route_clearance_weight: float = 8.0
    route_clearance_scale_m: float = 2.0

    def validate(self):
        values = asdict(self)
        if any(
            isinstance(v, bool)
            or not isinstance(v, (float, int))
            or not math.isfinite(v)
            or (v <= 0 and name not in ("floor_z_m", "map_origin_x_m", "map_origin_y_m"))
            for name, v in values.items()
        ):
            raise ValueError("Settings must be finite numbers, with positive speeds and dimensions")
        if (
            not 0.1 <= self.near_range_m <= 1.0
            or not self.near_range_m + 0.1 <= self.stop_distance_m < self.slow_distance_m
        ):
            raise ValueError(
                "Stop distance needs at least 0.1 m beyond the LiDAR near range and must be below slow distance"
            )
        if not self.body_radius_m < self.stop_distance_m <= self.obstacle_margin_m:
            raise ValueError("Obstacle margin must cover the stop distance and vehicle radius")
        if (
            self.cruise_m_s > 4.0
            or self.minimum_approach_m_s > self.cruise_m_s
            or self.turn_speed_m_s > self.cruise_m_s
            or self.yaw_misaligned_speed_m_s > 4.0
        ):
            raise ValueError(
                "Invalid approach/turn/cruise speed; this simulation supports cruise up to 4 m/s"
            )
        if self.target_lead_m > 0.30 or self.acceleration_m_s2 > 0.5:
            raise ValueError(
                "Target lead must be at most 0.30 m; reference acceleration at most 0.5 m/s^2"
            )
        if self.yaw_rate_deg_s > 90 or self.yaw_acceleration_deg_s2 > 180:
            raise ValueError("Yaw is limited to 90 deg/s and 180 deg/s² acceleration")
        if self.braking_reaction_s > 2.0:
            raise ValueError("Braking reaction allowance must be at most two simulated seconds")
        if self.route_clearance_weight > 20 or self.route_clearance_scale_m > 10:
            raise ValueError("Route clearance weight must be at most 20 and scale at most 10 m")
        if self.exploration_commit_horizon_s > 5.0:
            raise ValueError("Exploration route commitment must be at most 5 simulated seconds")
        if (
            type(self.exploration_max_goals) is not int
            or not 1 <= self.exploration_max_goals <= 100
        ):
            raise ValueError("Exploration goal limit must be an integer from 1 to 100")
        if not 0.05 <= self.map_resolution_m <= 0.25:
            raise ValueError("Map resolution must be between 0.05 and 0.25 m")
        if (
            any(
                type(value) is not int or not 16 <= value <= 1024
                for value in (self.map_width_cells, self.map_height_cells)
            )
            or self.map_width_cells * self.map_height_cells > 131072
        ):
            raise ValueError("Map dimensions must be integers with at most 131,072 total cells")
        return self

    def map_options(self):
        return dict(
            resolution=self.map_resolution_m,
            size=(self.map_height_cells, self.map_width_cells),
            origin=(self.map_origin_x_m, self.map_origin_y_m),
            body_radius=self.body_radius_m,
            clearance=self.obstacle_margin_m,
            route_clearance_weight=self.route_clearance_weight,
            route_clearance_scale=self.route_clearance_scale_m,
        )


def load_settings(path=None):
    path = Path(path) if path else Path(__file__).parents[1] / "config" / "navigation_settings.json"
    return NavigationSettings(**json.loads(path.read_text(encoding="utf-8"))).validate()
