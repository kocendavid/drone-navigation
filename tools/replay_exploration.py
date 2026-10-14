"""Offline occluded-room replay. Imports no ROS node or PX4 publisher."""

from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from navigation.mapping import RayMap, FREE, OCCUPIED
from navigation.feed import MapSnapshot
from navigation.exploration import choose_frontier
from navigation.motion import RouteFollower
from navigation.settings import load_settings


def main():
    settings = load_settings()
    model = RayMap(clearance=settings.obstacle_margin_m, body_radius=settings.body_radius_m)
    truth = np.zeros((120, 120), dtype=bool)
    truth[20:101, 20] = truth[20:101, 100] = True
    truth[20, 20:101] = truth[100, 20:101] = True
    truth[20:71, 60] = True  # Divider ending at world y=1: the far room is occluded.
    angles = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    offsets = (
        np.stack((np.cos(angles), np.sin(angles)), axis=1)[:, None, :]
        * np.arange(0.35, 12.0, 0.05)[None, :, None]
    )

    def scan(pose):
        rays = pose + offsets
        cells = np.floor((rays - model.origin) / model.resolution).astype(int)
        inside = (
            (cells[:, :, 0] >= 0)
            & (cells[:, :, 0] < 120)
            & (cells[:, :, 1] >= 0)
            & (cells[:, :, 1] < 120)
        )
        blocked = ~inside
        blocked[inside] = truth[cells[:, :, 1][inside], cells[:, :, 0][inside]]
        first = blocked.argmax(axis=1)
        return rays[np.arange(720), first]

    def observe(pose):
        hits = scan(pose)
        points = np.column_stack((hits - pose, np.zeros(len(hits))))
        model.update(points, np.eye(3), np.array([*pose, 0.0]), pose, 0.0, settings.near_range_m)
        clearance, body_free, safe = model.geometry()
        now = time.monotonic()
        return MapSnapshot(
            model,
            clearance,
            body_free,
            safe,
            [*pose, 0.0, 0.0],
            hits,
            model.scans,
            now,
            now,
            now,
            True,
            "Offline ray replay",
            model.scans,
        )

    actual = np.array([-2.0, -2.0])
    snap = observe(actual)

    def count():
        return int(((model.grid == FREE) | (model.grid == OCCUPIED)).sum())

    initial = count()
    visited = []
    results = []
    minimum = math.inf
    elapsed = 0.0
    distance = 0.0
    finish = "Viewpoint limit"
    for _ in range(8):
        try:
            choice = choose_frontier(snap, actual, visited)
        except ValueError as exc:
            finish = str(exc)
            break
        follower = RouteFollower(settings)
        follower.begin(choice.path)
        hits = scan(actual)
        for tick in range(4000):
            if tick % 5 == 0:
                hits = scan(actual)
            follower.tick(0.05, actual, hits)
            before = actual.copy()
            actual += (follower.target - actual) * 0.05 / 0.4
            distance += float(np.linalg.norm(actual - before))
            elapsed += 0.05
            minimum = min(minimum, float(np.linalg.norm(hits - actual, axis=1).min()))
            if follower.phase not in ("NAVIGATING", "SETTLING"):
                raise AssertionError(f"Offline route stopped: {follower.reason}")
            if follower.phase == "SETTLING" and np.linalg.norm(actual - choice.goal) < 0.05:
                break
        else:
            raise AssertionError("Offline goal did not settle")
        visited.append(choice.goal)
        snap = observe(actual)
        results.append(
            dict(
                goal=choice.goal,
                observed_cells=count(),
                new_area_m2=(count() - initial) * 0.01,
                expected_new_area_m2=choice.expected_new_area_m2,
                route_length_m=choice.route_length_m,
            )
        )
        print(json.dumps(results[-1]), flush=True)
    assert len(results) >= 2, "Need repeated autonomous viewpoint choices"
    assert (
        count() - initial >= 100
    ), "Need to reveal at least 1 square metre beyond the initial view"
    assert minimum > settings.stop_distance_m, "Offline replay violated requested stop distance"
    report = dict(
        recorded_utc=datetime.now(timezone.utc).isoformat(),
        kind="Offline ideal 2-D occluded-room replay; NOT Isaac/PX4 flight",
        settings=asdict(settings),
        initial_observed_cells=initial,
        final_observed_cells=count(),
        new_observed_area_m2=(count() - initial) * 0.01,
        viewpoints=results,
        simulated_duration_s=elapsed,
        measured_travel_m=distance,
        minimum_return_distance_m=minimum,
        finish_reason=finish,
    )
    output = Path(__file__).parents[1] / "logs" / "exploration_replay.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        "PASS:",
        len(results),
        "viewpoints;",
        round(report["new_observed_area_m2"], 2),
        "m² newly observed; evidence:",
        output,
    )


if __name__ == "__main__":
    main()
