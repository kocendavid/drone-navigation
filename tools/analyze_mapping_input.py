"""Offline cloud/TF replay and comparison against audited planar maze walls."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from navigation.mapping import RayMap, FREE, OCCUPIED
from navigation.settings import load_settings


def analyze(capture, scene):
    cfg = load_settings()
    audit = json.loads(Path(scene).read_text())
    walls = [g for g in audit["geometry"] if "/Wall" in g["path"]]
    arrays = np.load(capture)
    count = len([key for key in arrays.files if key.startswith("xyz_")])
    model = RayMap(**cfg.map_options())
    reports, positions, previous = [], [], None
    for i in range(count):
        xyz, wr, wt, br, bt = [arrays[f"{key}_{i}"] for key in ("xyz", "wr", "wt", "br", "bt")]
        body = wt - (wr @ br.T) @ bt
        positions.append(body)
        world = xyz @ wr.T + wt
        # This independent check excludes the floor and the robot's low structure.
        eligible = world[(world[:, 2] > cfg.floor_z_m + 0.3) & (world[:, 2] < cfg.floor_z_m + 2)]
        residual = np.full(len(eligible), np.inf)
        for wall in walls:
            low, high = np.array(wall["bounds_min"]), np.array(wall["bounds_max"])
            nearest = np.maximum(low, np.minimum(high, eligible))
            residual = np.minimum(residual, np.linalg.norm(eligible - nearest, axis=1))
        row = dict(
            index=i,
            body_xyz=body.tolist(),
            mapping_height_valid=bool(abs(body[2] - (cfg.floor_z_m + 1)) <= 0.2),
            elevated_returns=len(eligible),
            wall_distance_p50_p95_p99_m=(
                np.percentile(residual, [50, 95, 99]).tolist() if len(eligible) else None
            ),
            elevated_returns_over_10cm_from_planar_walls=int(np.count_nonzero(residual > 0.1)),
        )
        try:
            model.update(xyz, wr, wt, body[:2], body[2], cfg.near_range_m)
            row["map_changes"] = (
                dict(
                    free_to_occupied=int(
                        np.count_nonzero((previous == FREE) & (model.grid == OCCUPIED))
                    ),
                    occupied_to_free=int(
                        np.count_nonzero((previous == OCCUPIED) & (model.grid == FREE))
                    ),
                )
                if previous is not None
                else None
            )
            previous = model.grid.copy()
        except ValueError as exc:
            row["map_error"] = str(exc)
        reports.append(row)
    return dict(
        capture=str(capture),
        samples=reports,
        body_position_span_m=np.ptp(positions, axis=0).tolist() if positions else None,
        note="Offline diagnostic only. Planar wall residual excludes cylinder geometry; ground captures do not satisfy the navigation height gate.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--scene", type=Path, default=Path("logs/scene_settings_audit.json"))
    parser.add_argument("--output", type=Path, default=Path("logs/mapping_input_analysis.json"))
    args = parser.parse_args()
    result = analyze(args.capture, args.scene)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
