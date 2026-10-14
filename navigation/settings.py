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
    turn_speed_m_s: float = 0.15
    target_lead_m: float = 0.28
    body_radius_m: float = 0.3
    obstacle_margin_m: float = 0.8
    near_range_m: float = 0.5
    exploration_max_goals: int = 20

    def validate(self):
        values = asdict(self)
        if any(
            isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0
            for v in values.values()
        ):
            raise ValueError("Navigation settings must be finite positive numbers")
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
            self.cruise_m_s > 1.0
            or self.minimum_approach_m_s > self.cruise_m_s
            or self.turn_speed_m_s > self.cruise_m_s
        ):
            raise ValueError(
                "Invalid approach/turn/cruise speed; this experiment supports cruise up to 1 m/s"
            )
        if self.target_lead_m > 0.30 or self.acceleration_m_s2 > 0.5:
            raise ValueError(
                "Target lead must be at most 0.30 m; reference acceleration at most 0.5 m/s^2"
            )
        if (
            type(self.exploration_max_goals) is not int
            or not 1 <= self.exploration_max_goals <= 100
        ):
            raise ValueError("Exploration goal limit must be an integer from 1 to 100")
        return self


def load_settings(path=None):
    path = Path(path) if path else Path(__file__).parents[1] / "config" / "navigation_settings.json"
    return NavigationSettings(**json.loads(path.read_text(encoding="utf-8"))).validate()
