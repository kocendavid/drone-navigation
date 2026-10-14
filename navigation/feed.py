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

    def fresh(self, now):
        return self.valid and all(
            0 <= now - wall <= limit
            for wall, limit in (
                (self.generated, 0.85),
                (self.cloud_wall, 1.0),
                (self.clock_wall, 1.0),
            )
        )


class MapFeed:
    def __init__(self, floor_z=-0.76965, near_range=0.5, obstacle_margin=0.8, body_radius=0.3):
        self.floor_z = floor_z
        self.obstacle_margin = obstacle_margin
        self.body_radius = body_radius
        self.lock = threading.Lock()
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
            ],
            stdout=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
            creationflags=flags,
        )
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            while True:
                line = self.process.stdout.readline(262145)
                if not line:
                    break
                if len(line) > 262144 or not line.endswith("\n"):
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
                size = item["size"]
                origin = item["origin"]
                resolution = float(item["resolution"])
                if size != 120 or resolution != 0.1 or len(origin) != 2 or origin != [-6.0, -6.0]:
                    raise ValueError("Unexpected map geometry")
                chars = item["grid"]
                if len(chars) != size * size or set(chars) - set("?.#b"):
                    raise ValueError("Invalid map cells")
                old = self.snapshot
                if old is not None and item["scans"] == old.scans:
                    model, clearance, body_free, safe = (
                        old.model,
                        old.clearance,
                        old.body_free,
                        old.safe,
                    )
                else:
                    model = RayMap(body_radius=self.body_radius, clearance=self.obstacle_margin)
                    model.grid = np.array(
                        [{"?": 0, ".": 1, "#": 2, "b": 3}[c] for c in chars], dtype=np.uint8
                    ).reshape(size, size)
                    clearance, body_free, safe = model.geometry()
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
                )
                with self.lock:
                    self.snapshot = snap
                    self.failure = None
        except (KeyError, ValueError, TypeError, OSError) as exc:
            with self.lock:
                self.failure = str(exc)
        finally:
            with self.lock:
                self.failure = self.failure or "Map worker output closed"

    def get(self):
        with self.lock:
            return self.snapshot, self.failure or (
                "Map worker exited" if self.process.poll() is not None else None
            )

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.reader.join(timeout=1)
        self.process.stdout.close()
