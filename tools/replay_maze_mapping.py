"""Replay recorded travel through ideal planar maze rays to isolate mapper artifacts.

This is a diagnostic model, not a reconstruction of unrecorded historical LiDAR.
"""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from navigation.mapping import RayMap, FREE, OCCUPIED
from navigation.settings import load_settings


def ideal_rays(pose, walls, bearings):
    direction = np.column_stack((np.cos(bearings), np.sin(bearings)))
    distance = np.full(len(bearings), np.inf)
    for wall in walls:
        low, high = np.array(wall["bounds_min"][:2]), np.array(wall["bounds_max"][:2])
        axis = int(np.argmin(high - low))
        other = 1 - axis
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((low[axis] + high[axis]) / 2 - pose[axis]) / direction[:, axis]
            coordinate = pose[other] + t * direction[:, other]
        valid = (t >= 0.5) & (coordinate >= low[other]) & (coordinate <= high[other])
        distance = np.minimum(distance, np.where(valid, t, np.inf))
    valid = distance <= 12
    return direction[valid] * distance[valid, None]


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=root / "logs/ideal_maze_mapping_retained.json"
    )
    args = parser.parse_args()
    audit = json.loads((root / "logs/scene_settings_audit.json").read_text())
    history = json.loads((root / "logs/recovered_navigation_history.json").read_text())
    walls = [g for g in audit["geometry"] if "/Wall" in g["path"]]
    cfg = load_settings()
    model = RayMap(**cfg.map_options())
    bearings = np.arange(720) * 2 * np.pi / 720 + 0.00001
    previous = previous_confirmed = None
    confirmed_erasures = 0
    revisions, errors = [], []
    for i, pose in enumerate(history["trail"]):
        rays = ideal_rays(np.array(pose), walls, bearings)
        xyz = np.column_stack((rays, np.zeros(len(rays))))
        try:
            model.update(
                xyz,
                np.eye(3),
                np.array([*pose, cfg.floor_z_m + 1]),
                pose,
                cfg.floor_z_m + 1,
                cfg.near_range_m,
            )
        except ValueError as exc:
            errors.append(dict(index=i, reason=str(exc)))
            continue
        if previous is not None:
            confirmed_erasures += int(
                np.count_nonzero(previous_confirmed & (model.grid != OCCUPIED))
            )
            to_wall = int(np.count_nonzero((previous == FREE) & (model.grid == OCCUPIED)))
            to_free = int(np.count_nonzero((previous == OCCUPIED) & (model.grid == FREE)))
            if to_wall or to_free:
                revisions.append(
                    dict(index=i, pose=pose, free_to_occupied=to_wall, occupied_to_free=to_free)
                )
        previous = model.grid.copy()
        previous_confirmed = model.confirmed_walls.copy()
    result = dict(
        kind="Ideal fixed planar rays over recovered XY travel; no noise or timestamp error",
        caveats="Omits cylinder, attitude and unrecorded intermediate travel; not historical sensor evidence",
        samples=len(history["trail"]),
        revisions=revisions,
        errors=errors,
        total_occupied_to_free=sum(r["occupied_to_free"] for r in revisions),
        total_free_to_occupied=sum(r["free_to_occupied"] for r in revisions),
        confirmed_wall_erasures=confirmed_erasures,
        wall_policy=model.wall_policy(),
    )
    output = args.output
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k not in ("revisions", "errors")}))
    print("Errors:", len(errors), "; revision frames:", len(revisions))


if __name__ == "__main__":
    main()
