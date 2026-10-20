"""Recorded sensor maps, decisions and actual TF travel; no flight-control authority."""

from collections import deque, OrderedDict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from queue import Queue, Full
import sys
import threading
import time
import uuid
import numpy as np
from .process_worker import ProcessService


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def retained_bytes(value):
    """Conservative retained size, charging shared buffers separately per record."""
    seen = set()

    def size(item):
        if id(item) in seen:
            return 0
        seen.add(id(item))
        total = sys.getsizeof(item)
        if isinstance(item, np.ndarray):
            # Charge storage even for views; include their retained base as well.
            # Owned arrays may be double charged, intentionally overestimating.
            return total + item.nbytes + (size(item.base) if item.base is not None else 0)
        if isinstance(item, dict):
            return total + sum(size(k) + size(v) for k, v in item.items())
        if isinstance(item, (list, tuple, deque, set, frozenset)):
            return total + sum(size(v) for v in item)
        if hasattr(item, "__dict__"):
            return total + size(vars(item))
        return total

    return size(value)


class HistoryQueue(Queue):
    """FIFO bounded by both record count and conservative retained bytes."""

    def __init__(self, maxsize, max_bytes):
        if maxsize <= 0 or max_bytes <= 0:
            raise ValueError("History queue limits must be positive")
        super().__init__(maxsize)
        self.max_bytes = max_bytes
        self.bytes = self.peak_bytes = 0

    def put(self, item, block=True, timeout=None):
        # Barriers and shutdown retain no map/report data and never cross IPC.
        weight = 0 if item is None or item[0] == "barrier" else retained_bytes(item)
        if weight > self.max_bytes:
            raise Full
        with self.not_full:

            def full():
                return self._qsize() >= self.maxsize or self.bytes + weight > self.max_bytes

            if not block:
                if full():
                    raise Full
            elif timeout is None:
                while full():
                    self.not_full.wait()
            elif timeout < 0:
                raise ValueError("timeout must be non-negative")
            else:
                deadline = time.monotonic() + timeout
                while full():
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise Full
                    self.not_full.wait(remaining)
            self._put((item, weight))
            self.bytes += weight
            self.peak_bytes = max(self.peak_bytes, self.bytes)
            self.unfinished_tasks += 1
            self.not_empty.notify()

    def _get(self):
        item, weight = super()._get()
        self.bytes -= weight
        self.not_full.notify_all()
        return item

    def report(self):
        with self.mutex:
            return dict(
                records=self._qsize(),
                bytes=self.bytes,
                peak_bytes=self.peak_bytes,
                max_records=self.maxsize,
                max_bytes=self.max_bytes,
            )


class RunHistory:
    def __init__(self, path=None, *, queue_max_records=2048, queue_max_bytes=64 * 1024 * 1024):
        self.run_id = uuid.uuid4().hex
        self.path = Path(path) if path else None
        self.events = deque(maxlen=200)
        self.targets = deque(maxlen=1000)
        self.trail = deque(maxlen=5000)
        self.frames = OrderedDict()
        self.frame_offsets = OrderedDict()
        self.lock = threading.RLock()
        self.counter_lock = threading.Lock()
        self.submission_lock = threading.Lock()
        self.submitted_state = self.submitted_goal = None
        self.submitted_frame_s = self.submitted_scans = None
        self.sequence = 0
        self.last_state = self.last_goal = self.last_grid = None
        self.last_frame_s = None
        self.last_scans = None
        self.error = None
        self.dropped = 0
        self.closed = False
        self.queue = HistoryQueue(queue_max_records, queue_max_bytes)
        self.thread = None
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.touch(exist_ok=False)
            self.service = ProcessService(
                "history_writer", "navigation.report_jobs:HistoryProcessor", self.path, self.run_id
            )
            self.remote_report = None
            self.thread = threading.Thread(target=self._writer, daemon=True)
            self.thread.start()

    def _write(self, item):
        if self.path:
            with self.path.open("a", encoding="utf-8") as stream:
                offset = stream.tell()
                stream.write(json.dumps(item, allow_nan=False, separators=(",", ":")) + "\n")
                return offset

    def _writer(self):
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                if item[0] == "barrier":
                    item[1].set()
                else:
                    report, offsets = self.service.call(item)
                    with self.lock:
                        self.remote_report = report
                        self.frame_offsets = offsets
                        self.error = report["error"]
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
            finally:
                self.queue.task_done()

    def _submit(self, item):
        if self.closed:
            return False
        if not self.thread:
            if item[0] == "state":
                self._observe(*item[1:])
            else:
                self._event(*item[1:])
            return True
        try:
            self.queue.put_nowait(item)
            return True
        except Full:
            with self.counter_lock:
                self.dropped += 1
            return False

    def event(self, kind, reason, **data):
        self._submit(("event", kind, reason, data))

    def _event(self, kind, reason, data=None, **fields):
        data = dict(data or {}, **fields)
        with self.lock:
            self.sequence += 1
            event = dict(
                id=self.sequence,
                run_id=self.run_id,
                utc=utc_now(),
                kind=kind,
                reason=reason,
                **data,
            )
            self.events.append(event)
            self._write(dict(type="event", **event))
            return event

    def observe(self, report, snapshot, now):
        # Select before enqueueing: ordinary telemetry must not retain map arrays.
        # Keep this policy identical for local history and the subprocess writer.
        with self.submission_lock:
            phase_reason = report.get("phase"), report.get("reason", "")
            goal = tuple(report["goal"]) if report.get("goal") is not None else None
            simulation_s = report.get("delivered_simulation_s", 0.0)
            changed = phase_reason != self.submitted_state or goal != self.submitted_goal
            frame = snapshot is not None and (
                changed
                or self.submitted_frame_s is None
                or (
                    snapshot.scans != self.submitted_scans
                    and simulation_s - self.submitted_frame_s >= 5.0
                )
            )
            fresh = snapshot is not None and snapshot.fresh(now)
            telemetry = dict(
                fresh=fresh,
                xy=tuple(map(float, snapshot.pose[:2])) if fresh else None,
                stamp_ns=snapshot.stamp if snapshot is not None else None,
            )
            # Full report details belong to frames. Routine records need only
            # these decision fields and the capture-time pose/freshness above.
            state = (
                dict(report)
                if frame
                else {
                    key: report[key]
                    for key in (
                        "phase",
                        "reason",
                        "goal",
                        "delivered_simulation_s",
                        "trajectory_simulation_s",
                        "speed_decision",
                        "clearance_m",
                        "tracking",
                    )
                    if key in report
                }
            )
            if not frame and "exploration" in report:
                exploration = report["exploration"]
                state["exploration"] = {
                    key: exploration[key]
                    for key in ("enabled", "selection_reason")
                    if key in exploration
                }
            accepted = self._submit(("state", state, snapshot if frame else None, telemetry))
            if accepted:
                self.submitted_state, self.submitted_goal = phase_reason, goal
                if frame:
                    self.submitted_frame_s, self.submitted_scans = simulation_s, snapshot.scans
            return accepted

    def _observe(self, report, snap, telemetry):
        with self.lock:
            simulation_s = report.get("delivered_simulation_s", 0.0)
            phase, reason = report.get("phase"), report.get("reason", "")
            goal = tuple(report["goal"]) if report.get("goal") is not None else None
            changed = (phase, reason) != self.last_state or goal != self.last_goal
            speed_decision = report.get("speed_decision")
            if speed_decision is not None and (phase == "NAVIGATING" or changed):
                # Compact numeric records follow the control reports without
                # adding event-list rows or forcing expensive map frames.
                self._write(
                    dict(
                        type="speed_decision",
                        run_id=self.run_id,
                        utc=utc_now(),
                        simulation_s=simulation_s,
                        trajectory_simulation_s=report.get("trajectory_simulation_s"),
                        goal=list(goal) if goal else None,
                        speed_decision=speed_decision,
                    )
                )
            fresh = telemetry["fresh"]
            frame_id = self.sequence + 1 if snap is not None else None
            if fresh:
                xy = list(telemetry["xy"])
                if not self.trail or math.dist(xy, self.trail[-1]) >= 0.1:
                    self.trail.append(xy)
                    self._write(
                        dict(
                            type="pose",
                            run_id=self.run_id,
                            utc=utc_now(),
                            simulation_s=simulation_s,
                            source="Isaac capture-time TF",
                            xy=xy,
                            stamp_ns=telemetry["stamp_ns"],
                        )
                    )
            if frame_id is not None:
                self.sequence += 1
                grid = snap.model.grid
                changes = dict(free_to_occupied=0, occupied_to_free=0)
                if self.last_grid is not None and self.last_grid.shape == grid.shape:
                    changes = dict(
                        free_to_occupied=int(np.count_nonzero((self.last_grid == 1) & (grid == 2))),
                        occupied_to_free=int(np.count_nonzero((self.last_grid == 2) & (grid == 1))),
                    )
                self.last_grid = grid.copy()
                cell = snap.model.cell(snap.pose[:2])
                footprint = {}
                if snap.model.inside(cell):
                    footprint = dict(
                        body_observed=bool(snap.body_free[cell]),
                        preferred_margin_met=bool(snap.safe[cell]),
                        wall_clearance_m=float(snap.clearance[cell]),
                    )
                frame = dict(
                    report,
                    id=frame_id,
                    run_id=self.run_id,
                    utc=utc_now(),
                    provisional=True,
                    map_fresh=fresh,
                    map_reason=snap.reason,
                    scans=snap.scans,
                    stamp_ns=snap.stamp,
                    width=grid.shape[1],
                    height=grid.shape[0],
                    origin=snap.model.origin.tolist(),
                    resolution=snap.model.resolution,
                    grid="".join(np.array(["?", ".", "#", "b"])[grid.ravel()].tolist()),
                    confirmed_walls="".join(
                        np.where(snap.model.confirmed_walls.ravel(), "1", "0").tolist()
                    ),
                    wall_policy=snap.model.wall_policy(),
                    safe="".join(np.where(snap.safe.ravel(), "1", "0").tolist()),
                    centerline=(
                        "".join(np.where(snap.centerline.ravel(), "1", "0").tolist())
                        if getattr(snap, "centerline", None) is not None
                        else ""
                    ),
                    pose=list(snap.pose) if snap.valid else None,
                    trail=list(self.trail),
                    cell_changes=changes,
                    footprint=footprint,
                    hit_endpoints_xy=np.round(getattr(snap, "hits", np.empty((0, 2))), 3).tolist(),
                )
                self.frames[frame_id] = frame
                while len(self.frames) > 80:
                    self.frames.popitem(last=False)
                self.last_frame_s, self.last_scans = simulation_s, snap.scans
                offset = self._write(dict(type="frame", **frame))
                self.frame_offsets[frame_id] = offset
                while len(self.frame_offsets) > 10000:
                    self.frame_offsets.popitem(last=False)
            if goal != self.last_goal:
                if self.targets and self.targets[-1]["status"] == "selected":
                    self.targets[-1]["status"] = (
                        "superseded"
                        if goal
                        else "invalidated" if "no longer eligible" in reason else "cancelled"
                    )
                if goal is not None:
                    exploration = report.get("exploration", {})
                    selection = (
                        exploration.get("selection_reason") if exploration.get("enabled") else None
                    ) or reason
                    event = self._event(
                        "target_selected",
                        selection,
                        goal=list(goal),
                        phase=phase,
                        simulation_s=simulation_s,
                        frame_id=frame_id,
                    )
                    self.targets.append(
                        dict(
                            id=event["id"],
                            xy=list(goal),
                            utc=event["utc"],
                            reason=selection,
                            status="selected",
                            frame_id=frame_id,
                        )
                    )
            if phase == "REACHED" and self.targets:
                self.targets[-1]["status"] = "reached"
            if (phase, reason) != self.last_state:
                self._event(
                    "blocked" if phase == "BLOCKED" else "state",
                    reason,
                    phase=phase,
                    goal=list(goal) if goal else None,
                    simulation_s=simulation_s,
                    frame_id=frame_id,
                    clearance_ahead_m=report.get("clearance_m"),
                    speed_decision=speed_decision,
                    tracking=report.get("tracking"),
                )
            self.last_state, self.last_goal = (phase, reason), goal

    def report(self):
        with self.lock:
            if getattr(self, "remote_report", None) is not None:
                return dict(
                    self.remote_report,
                    error=self.error,
                    dropped_records=self.dropped,
                    queue=self.queue.report(),
                )
            return dict(
                run_id=self.run_id,
                log_file=str(self.path) if self.path else None,
                events=[dict(e) for e in self.events],
                targets=[dict(t) for t in self.targets],
                trail=list(self.trail),
                frames=[
                    dict(
                        id=f["id"],
                        utc=f["utc"],
                        phase=f.get("phase"),
                        reason=f.get("reason"),
                        map_fresh=f["map_fresh"],
                        cell_changes=f["cell_changes"],
                        footprint=f["footprint"],
                    )
                    for f in self.frames.values()
                ],
                error=self.error,
                frame_ids=list(self.frame_offsets),
                dropped_records=self.dropped,
                queue=self.queue.report(),
            )

    def frame(self, frame_id):
        with self.lock:
            frame = self.frames.get(frame_id)
            offset = self.frame_offsets.get(frame_id)
        if frame is not None:
            return frame
        if offset is not None and self.path:
            with self.path.open("rb") as stream:
                stream.seek(offset)
                return json.loads(stream.readline())
        return None

    def flush(self, timeout=3):
        if self.thread and not self.closed:
            barrier = threading.Event()
            self.queue.put(("barrier", barrier), timeout=timeout)
            return barrier.wait(timeout)
        return True

    def close(self):
        if not self.closed:
            self.flush()
            self.closed = True
            if self.thread:
                self.queue.put(None, timeout=3)
                self.thread.join(timeout=3)
                self.service.close()
                self.thread.join(timeout=3)


class CommandReceipts:
    def __init__(self):
        self.lock = threading.Lock()
        self.items = OrderedDict()

    def begin(self, payload):
        request_id = payload.get("request_id", uuid.uuid4().hex)
        if (
            not isinstance(request_id, str)
            or not 1 <= len(request_id) <= 80
            or not all(c.isalnum() or c in "-_" for c in request_id)
        ):
            raise ValueError("Invalid request identifier")
        fingerprint = json.dumps(
            {k: v for k, v in payload.items() if k != "request_id"}, sort_keys=True, allow_nan=False
        )
        with self.lock:
            if request_id in self.items:
                item = self.items[request_id]
                if item["fingerprint"] != fingerprint:
                    raise ValueError("Request identifier already belongs to another command")
                return dict(item), False
            item = dict(
                id=request_id,
                action=payload.get("action"),
                status="received",
                reason="Request received",
                utc=utc_now(),
                fingerprint=fingerprint,
                deadline=time.monotonic() + 5.0,
            )
            self.items[request_id] = item
            while len(self.items) > 100:
                self.items.popitem(last=False)
            return dict(item), True

    def update(self, request_id, status, reason):
        with self.lock:
            item = self.items.get(request_id)
            if item is not None:
                if item["status"] not in (
                    "expired",
                    "rejected",
                    "failed",
                    "cancelled",
                    "completed",
                    "stopped",
                ):
                    item.update(status=status, reason=reason, utc=utc_now())
            return (
                {k: v for k, v in item.items() if k not in ("fingerprint", "deadline")}
                if item
                else None
            )

    def report(self):
        with self.lock:
            for item in self.items.values():
                if item["status"] in ("received", "queued") and time.monotonic() > item["deadline"]:
                    item.update(
                        status="expired",
                        reason="Controller did not acknowledge within 5 seconds; request expired",
                    )
            return [
                {k: v for k, v in item.items() if k not in ("fingerprint", "deadline")}
                for item in self.items.values()
            ]
