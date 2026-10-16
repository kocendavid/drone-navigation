"""Recover the last controller run's observed states/targets from the old sampled trace.

Old traces did not record grids. This tool never invents a historical sensor map.
"""

import argparse
import json
import math
from pathlib import Path


def recover(trace):
    events, targets, trail = [], [], []
    previous_s = None
    previous_state = previous_goal = None
    first_utc = last_utc = None
    settings = {}
    with Path(trace).open(encoding="utf-8") as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            simulation_s = row.get("delivered_simulation_s")
            if simulation_s is None:
                continue
            if previous_s is not None and simulation_s < previous_s - 1.0:
                events, targets, trail = [], [], []
                previous_state = previous_goal = None
                first_utc = None
            previous_s = simulation_s
            utc = row.get("recorded_utc")
            first_utc = first_utc or utc
            last_utc = utc
            settings = row.get("settings", settings)
            xy = row.get("measured_world_xy")
            if xy and len(xy) == 2 and all(math.isfinite(n) for n in xy):
                if not trail or math.dist(xy, trail[-1]) >= 0.1:
                    trail.append(xy)
            goal = row.get("goal")
            if goal != previous_goal:
                if targets and targets[-1]["status"] == "selected":
                    targets[-1]["status"] = "superseded" if goal else "cancelled"
                if goal:
                    targets.append(
                        dict(
                            id=len(targets) + 1,
                            utc=utc,
                            xy=goal,
                            status="selected",
                            reason=row.get("exploration", {}).get("selection_reason")
                            or row.get("reason"),
                        )
                    )
            if row.get("phase") == "REACHED" and targets:
                targets[-1]["status"] = "reached"
            state = (row.get("phase"), row.get("reason"))
            if state != previous_state or goal != previous_goal:
                events.append(
                    dict(
                        utc=utc,
                        phase=state[0],
                        reason=state[1],
                        goal=goal,
                        simulation_s=simulation_s,
                    )
                )
            previous_state, previous_goal = state, goal
    return dict(
        source=str(Path(trace).resolve()),
        provenance="Recovered from sampled diagnostic states; transitions between samples may be absent",
        map_frames_available=False,
        pose_source="PX4 local position converted to world; sampled at least 0.1 m apart",
        first_utc=first_utc,
        last_utc=last_utc,
        settings=settings,
        events=events,
        targets=targets,
        trail=trail,
    )


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, default=root / "logs/navigation_trace.jsonl")
    parser.add_argument(
        "--output", type=Path, default=root / "logs/recovered_navigation_history.json"
    )
    args = parser.parse_args()
    data = recover(args.trace)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            dict(
                output=str(args.output),
                targets=len(data["targets"]),
                travel_samples=len(data["trail"]),
                state_changes=len(data["events"]),
                map_frames_available=False,
            )
        )
    )


if __name__ == "__main__":
    main()
