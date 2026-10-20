"""Visualization, JSON and disk work, isolated from command generation."""

import json
from pathlib import Path


class ReportWriter:
    def __init__(self, project, z):
        import rclpy
        from .ros_map import RosMapPublisher
        from .runtime import RuntimeVersion

        self.project = Path(project)
        self.runtime = RuntimeVersion(project)
        rclpy.init()
        self.node = rclpy.create_node("isaac_navigation_visualization")
        self.visualization = RosMapPublisher(self.node, z)

    def __call__(self, item):
        report, snap = item
        self.visualization.publish(snap, report["path"], report["trail"])
        path = self.project / "logs" / "navigation_state.json"
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(report, allow_nan=False, indent=2) + "\n", encoding="utf-8")
        temp.replace(path)
        with path.with_name("navigation_trace.jsonl").open("a", encoding="utf-8") as trace:
            trace.write(json.dumps(report, allow_nan=False, separators=(",", ":")) + "\n")
        return self.runtime.report()


class PresentationJobs:
    def __call__(self, item):
        kind, value = item
        if kind == "json":
            return json.dumps(value, allow_nan=False).encode()
        if kind == "plan":
            from .planning import plan

            snap, point = value
            return plan(snap, snap.pose[:2], point)
        if kind == "map":
            import numpy as np
            from .centering import center_geometry

            snap = value
            centerline = snap.centerline
            if centerline is None:
                _, centerline = center_geometry(snap.model, snap.clearance, snap.safe)
            return dict(
                size=snap.model.grid.shape[1],
                width=snap.model.grid.shape[1],
                height=snap.model.grid.shape[0],
                origin=snap.model.origin.tolist(),
                resolution=snap.model.resolution,
                grid="".join(np.array(["?", ".", "#", "b"])[snap.model.grid.ravel()].tolist()),
                confirmed_walls="".join(
                    np.where(snap.model.confirmed_walls.ravel(), "1", "0").tolist()
                ),
                wall_policy=snap.model.wall_policy(),
                safe="".join(np.where(snap.safe.ravel(), "1", "0").tolist()),
                centerline="".join(np.where(centerline.ravel(), "1", "0").tolist()),
                planning_policy="prefer_known_space_centers_and_clearance",
                pose=snap.pose if snap.valid else None,
                hit_endpoints_xy=np.round(snap.hits, 3).tolist() if snap.valid else [],
                scans=snap.scans,
                observed_cells=int(((snap.model.grid == 1) | (snap.model.grid == 2)).sum()),
            )
        raise ValueError(f"Unknown presentation job: {kind}")


class HistoryProcessor:
    def __init__(self, path, run_id):
        from .history import RunHistory

        self.history = RunHistory()
        self.history.path = Path(path)
        self.history.run_id = run_id

    def __call__(self, item):
        try:
            if item[0] == "state":
                self.history._observe(*item[1:])
            else:
                self.history._event(*item[1:])
        except Exception as exc:
            self.history.error = str(exc)
            raise
        return self.history.report(), self.history.frame_offsets
