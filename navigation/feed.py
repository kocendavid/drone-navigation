"""Read immutable live map snapshots; pipe loss cannot retain flight permission."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
import numpy as np
from .mapping import RayMap
from .settings import NavigationSettings
from .timing import TrajectoryClock
from .diagnostic_wire import DiagnosticEmitter
from .process_worker import ProcessService


@dataclass
class MapSnapshot:
    model: RayMap
    clearance: np.ndarray
    body_free: np.ndarray
    safe: np.ndarray
    pose: list
    hits: np.ndarray
    stamp: int
    generated: float
    cloud_wall: float
    clock_wall: float
    valid: bool
    reason: str
    scans: int
    center_clearance: np.ndarray | None = None
    centerline: np.ndarray | None = None

    def fresh(self, now):
        return self.valid and all(
            0 <= now - wall <= limit
            for wall, limit in (
                (self.generated, 0.85),
                (self.cloud_wall, 1.0),
                (self.clock_wall, 1.0),
            )
        )

    def health(self, now):
        return dict(
            valid=self.valid,
            fresh=self.fresh(now),
            reason=self.reason,
            publication_age_s=now - self.generated,
            cloud_age_s=now - self.cloud_wall,
            clock_age_s=now - self.clock_wall,
            scans=self.scans,
            stamp_ns=self.stamp,
        )


class MapFeed:
    def __init__(
        self,
        floor_z=-0.76965,
        near_range=0.5,
        obstacle_margin=0.8,
        body_radius=0.3,
        settings=None,
        control_backend="python",
    ):
        self.settings = settings or NavigationSettings(
            near_range_m=near_range, obstacle_margin_m=obstacle_margin, body_radius_m=body_radius
        )
        self.floor_z = floor_z
        self.obstacle_margin = obstacle_margin
        self.body_radius = body_radius
        self.lock = threading.Lock()
        self.map_condition = threading.Condition(self.lock)
        self.pending_map = None
        self.stopping = False
        self.maps_superseded = 0
        if control_backend == "cpp":
            from .native_control import NativeTrajectoryClock

            self.trajectory_clock = NativeTrajectoryClock()
        else:
            self.trajectory_clock = TrajectoryClock()
        self.diagnostics = DiagnosticEmitter("map_feed")
        self.map_diagnostics = DiagnosticEmitter("map_geometry")
        self.snapshot = None
        self.failure = None
        self.confirmed = 0
        self.stamp = None
        self.session = str(uuid.uuid4())
        env = dict(
            os.environ, RMW_IMPLEMENTATION="rmw_zenoh_cpp", ROS_DOMAIN_ID="0", PYTHONUNBUFFERED="1"
        )
        flags = (
            subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
            if os.name == "nt"
            else 0
        )
        self.process = subprocess.Popen(
            [
                sys.executable,
                "-u",
                str(Path(__file__).parents[1] / "tools" / "navigation_map_worker.py"),
                "--stream-session",
                self.session,
                "--floor-z",
                str(floor_z),
                "--near-range",
                str(near_range),
                "--map-resolution",
                str(self.settings.map_resolution_m),
                "--map-width",
                str(self.settings.map_width_cells),
                "--map-height",
                str(self.settings.map_height_cells),
                "--map-origin-x",
                str(self.settings.map_origin_x_m),
                "--map-origin-y",
                str(self.settings.map_origin_y_m),
            ],
            stdout=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
            creationflags=flags,
        )
        self.geometry = ProcessService(
            "map_geometry", "navigation.map_geometry:MapGeometry", self.settings
        )
        self.processor = threading.Thread(target=self._process_maps, daemon=True)
        self.processor.start()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            while True:
                with self.map_condition:
                    if self.stopping:
                        return
                line = self.process.stdout.readline(524289)
                if not line:
                    break
                if len(line) > 524288 or not line.endswith("\n"):
                    raise ValueError("Map pipe record too large")
                if not line.startswith("{"):
                    print("[Map worker] " + line.strip(), file=sys.stderr, flush=True)
                    continue
                item = json.loads(line)
                now = time.monotonic()
                if item["session"] != self.session or item["frame"] != "world":
                    raise ValueError("Map session/frame mismatch")
                generated = float(item["generated_monotonic_s"])
                if not math.isfinite(generated) or not 0 <= now - generated <= 0.7:
                    raise ValueError("Buffered/future map record")
                if item.get("type") == "clock":
                    stamp = item["clock_ns"]
                    if type(stamp) is not int or stamp < 0:
                        raise ValueError("Invalid simulation clock")
                    with self.map_condition:
                        if self.stopping:
                            return
                        self.trajectory_clock.observe(stamp, generated)
                    if hasattr(self, "diagnostics"):
                        self.diagnostics.emit(
                            "feed_clock",
                            sim_ns=stamp,
                            worker_generated_wall_s=generated,
                            pipe_delay_s=now - generated,
                        )
                    continue
                self._enqueue_map(item)
        except (KeyError, ValueError, TypeError, OSError) as exc:
            self._fail(str(exc))
        finally:
            self._fail("Map worker output closed")

    def _fail(self, reason):
        # A processing result finishing after EOF/error must never restore permission.
        with self.map_condition:
            self.failure = self.failure or reason
            self.stopping = True
            self.trajectory_clock.invalidate()
            self.pending_map = None
            self.map_condition.notify_all()

    def _enqueue_map(self, item):
        # The worker already integrates every accepted scan. These are cumulative
        # snapshots: retain just the newest pending one instead of building a backlog.
        with self.map_condition:
            if not self.stopping:
                self.maps_superseded += int(self.pending_map is not None)
                self.pending_map = item
                self.map_condition.notify()

    def _process_maps(self):
        try:
            while True:
                with self.map_condition:
                    self.map_condition.wait_for(
                        lambda: self.stopping or self.pending_map is not None
                    )
                    if self.stopping:
                        return
                    item, self.pending_map = self.pending_map, None
                self._process_map(item)
        except Exception as exc:
            self._fail("Map processing failed: " + str(exc))

    def _process_map(self, item):
        generated = float(item["generated_monotonic_s"])
        now = time.monotonic()
        if not math.isfinite(generated) or not 0 <= now - generated <= 0.7:
            raise ValueError("Buffered/future map record")
        width, height = item["width"], item["height"]
        origin = item["origin"]
        resolution = float(item["resolution"])
        if (
            width != self.settings.map_width_cells
            or height != self.settings.map_height_cells
            or resolution != self.settings.map_resolution_m
            or origin != [self.settings.map_origin_x_m, self.settings.map_origin_y_m]
        ):
            raise ValueError("Unexpected map geometry")
        chars = item["grid"]
        if len(chars) != width * height or set(chars) - set("?.#b"):
            raise ValueError("Invalid map cells")
        geometry_started = time.monotonic()
        model, clearance, body_free, safe, center_clearance, centerline = self.geometry.call(item)
        if hasattr(self, "map_diagnostics"):
            self.map_diagnostics.emit(
                "feed_map",
                sim_ns=item["stamp_ns"],
                worker_generated_wall_s=generated,
                pipe_delay_s=now - generated,
                geometry_duration_s=time.monotonic() - geometry_started,
                maps_superseded=self.maps_superseded,
                cloud_age_at_worker_s=item.get("cloud_age_s"),
            )
        valid = item["fresh"] is True
        stamp = item["stamp_ns"]
        pose = item["pose"]
        hits = np.asarray(item["hits"], dtype=float).reshape(-1, 2)
        if valid:
            if (
                not isinstance(stamp, int)
                or stamp <= 0
                or len(pose) != 4
                or not np.isfinite(pose).all()
            ):
                raise ValueError("Invalid map pose/stamp")
            if not 120 <= len(hits) <= 720 or not np.isfinite(hits).all():
                raise ValueError("Invalid obstacle ray endpoints")
            if not all(
                math.isfinite(item[key]) and 0 <= item[key] <= 1.0
                for key in ("cloud_age_s", "clock_age_s")
            ):
                raise ValueError("Stale map input")
            if self.stamp is None or stamp > self.stamp:
                self.confirmed += 1
            elif stamp < self.stamp:
                self.confirmed = 0
            self.stamp = stamp
        else:
            self.confirmed = 0
        snap = MapSnapshot(
            model,
            clearance,
            body_free,
            safe,
            pose or [0.0, 0.0, 0.0, 0.0],
            hits,
            stamp or 0,
            generated,
            generated - (item["cloud_age_s"] or 0.0),
            generated - (item["clock_age_s"] or 0.0),
            valid and self.confirmed >= 3,
            (
                item["reason"]
                if not valid
                else ("Live map" if self.confirmed >= 3 else "Confirming live scans")
            ),
            item["scans"],
            center_clearance,
            centerline,
        )
        with self.lock:
            if not self.stopping:
                self.snapshot = snap
                self.map_condition.notify_all()

    def get(self):
        with self.lock:
            return self.snapshot, self.failure or (
                "Map worker exited" if self.process.poll() is not None else None
            )

    def close(self):
        self._fail("Map feed closed")
        self.geometry.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.reader.join(timeout=1)
        self.processor.join(timeout=1)
        self.process.stdout.close()
        if not self.processor.is_alive() and not self.reader.is_alive():
            self.diagnostics.close()
            self.map_diagnostics.close()
