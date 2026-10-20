"""C ABI adapters for native publication and source-time handling."""

import ctypes as C
from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = ROOT / ".tools" / "native_control"


def build_library_path():
    try:
        name = json.loads((BUILD_DIR / "current_build.json").read_text())["library"]
        if Path(name).name == name and name.startswith(("isaac_control_", "libisaac_control_")):
            return BUILD_DIR / name
    except (OSError, KeyError, ValueError, TypeError):
        pass
    return BUILD_DIR / ("isaac_control.dll" if os.name == "nt" else "libisaac_control.so")


LIBRARY_PATH = build_library_path()
REASONS = (
    "healthy",
    "missing_clock",
    "confirming_after_discontinuity",
    "clock_update_stale",
    "unconsumed_interval_exceeded_0.25_s",
)
DISCONTINUITIES = (None, "clock_moved_backward", "clock_step_exceeded_0.25_s")
HISTORY_MODES = (
    "source_time_exact",
    "source_time_interpolated",
    "source_time_predicted",
    "reference_history_unavailable",
    "reference_history_gap",
    "reference_history_too_old",
)
SAMPLE_REASONS = (
    None,
    "invalid_sample_timestamp",
    "clock_mapping_unavailable",
    "ambiguous_timesync_epoch",
)


def source_hash():
    hashes = [
        hashlib.sha256((ROOT / "native_control" / name).read_bytes()).hexdigest()
        for name in ("control.cpp", "time_core.hpp", "CMakeLists.txt")
    ]
    return hashlib.sha256("".join(hashes).encode()).hexdigest()


class ClockState(C.Structure):
    _fields_ = (
        [(name, C.c_int64) for name in ("stamp", "decision_stamp")]
        + [
            (name, C.c_double)
            for name in (
                "total",
                "mark",
                "received",
                "delta",
                "decision_wall",
                "decision_age",
                "decision_elapsed",
            )
        ]
        + [(name, C.c_int32) for name in ("confirming", "reason", "discontinuity", "has_decision")]
    )


class PublicationStats(C.Structure):
    _fields_ = [(name, C.c_uint64) for name in ("published", "expired", "rejected", "repeats")]


class SubmissionState(C.Structure):
    _fields_ = [
        (name, C.c_int64)
        for name in (
            "issued_ns",
            "entered_ns",
            "validated_ns",
            "completed_ns",
            "previous_issued_ns",
            "graph_ns",
            "publish_ns",
            "reason",
            "invalid_index",
            "mode_publishers",
            "target_publishers",
        )
    ]


SUBMISSION_REASONS = (
    "accepted",
    "publisher_stopped",
    "benchmark_active",
    "out_of_order_command",
    "future_command",
    "stale_command",
    "invalid_frame",
    "competing_publisher",
    "publication_error",
)


class NativeSubmissionError(RuntimeError):
    def __init__(self, submission, detail=None):
        self.submission = submission
        super().__init__(
            f"Native publisher rejected {submission['reason']}; publication stopped "
            f"(command age {submission['command_age_s'] * 1000:.1f} ms, "
            f"native lock wait {submission['lock_wait_s'] * 1000:.1f} ms)"
            + (f": {detail}" if detail else "")
        )


@lru_cache(maxsize=1)
def library():
    if not LIBRARY_PATH.exists():
        raise RuntimeError(
            "Native control is not built. Run tools/build_native_control.cmd inside the Pixi ROS environment, or select --control-backend python."
        )
    directories = []
    if os.name == "nt":
        candidates = [Path(item) for item in os.environ.get("PATH", "").split(os.pathsep) if item]
        candidates += [
            Path("C:/px4-tools/ros_ws/install/bin"),
            Path("C:/px4-tools/ros_ws/install/lib"),
        ]
        for path in dict.fromkeys(candidates):
            if path.is_dir():
                try:
                    directories.append(os.add_dll_directory(str(path.resolve())))
                except OSError:
                    pass
    dll = C.CDLL(str(LIBRARY_PATH))
    dll.directory_handles = directories
    pointer, i64, double = C.c_void_p, C.c_int64, C.c_double
    vector = C.POINTER(double)
    signatures = {
        "ic_abi_version": (C.c_int, []),
        "ic_source_hash": (C.c_char_p, []),
        "ic_error": (C.c_char_p, []),
        "ic_clock_new": (pointer, []),
        "ic_clock_delete": (None, [pointer]),
        "ic_clock_observe": (None, [pointer, i64, double]),
        "ic_clock_invalidate": (None, [pointer]),
        "ic_clock_consume": (double, [pointer, double]),
        "ic_clock_fresh": (C.c_int, [pointer, double]),
        "ic_clock_state": (None, [pointer, C.POINTER(ClockState)]),
        "ic_timeline_new": (pointer, [C.c_int]),
        "ic_timeline_delete": (None, [pointer]),
        "ic_timeline_poll": (None, [pointer, double]),
        "ic_timeline_epoch": (C.c_int, [pointer]),
        "ic_timeline_pair": (None, [pointer, C.c_char_p, i64, i64, double, double, double]),
        "ic_timeline_sync": (None, [pointer, i64, i64, C.c_int, i64, double]),
        "ic_timeline_sample": (C.c_int, [pointer, i64, i64, double, C.POINTER(i64)]),
        "ic_history_new": (pointer, []),
        "ic_history_delete": (None, [pointer]),
        "ic_history_clear": (None, [pointer]),
        "ic_history_record": (None, [pointer, i64, vector, vector, i64]),
        "ic_history_at": (C.c_int, [pointer, i64, vector]),
        "ic_publisher_new": (pointer, [C.c_int, C.c_char_p, pointer, pointer, C.c_int]),
        "ic_publisher_delete": (None, [pointer]),
        "ic_publisher_submit": (C.c_int, [pointer, vector, i64]),
        "ic_publisher_revoke": (None, [pointer]),
        "ic_publisher_readers": (C.c_size_t, [pointer, C.c_int]),
        "ic_publisher_stats": (None, [pointer, C.POINTER(PublicationStats)]),
        "ic_publisher_submission": (None, [pointer, C.POINTER(SubmissionState), C.c_int]),
        "ic_publisher_alive": (C.c_int, [pointer]),
        "ic_publisher_benchmark": (i64, [pointer, C.c_size_t]),
        "ic_publisher_benchmark_count": (C.c_size_t, [pointer]),
        "ic_publisher_trace": (C.c_int, [pointer]),
        "ic_publisher_benchmark_row": (C.c_int, [pointer, C.c_size_t, C.POINTER(i64), vector]),
    }
    for name, (result, arguments) in signatures.items():
        function = getattr(dll, name)
        function.restype, function.argtypes = result, arguments
    if dll.ic_abi_version() != 2 or dll.ic_source_hash().decode() != source_hash():
        raise RuntimeError(
            "Native control build is stale; rebuild with tools/build_native_control.cmd."
        )
    return dll


class NativeHandle:
    def close(self):
        if getattr(self, "handle", None):
            getattr(self.dll, self.delete_function)(self.handle)
            self.handle = None

    def __del__(self):
        self.close()


class NativeTrajectoryClock(NativeHandle):
    delete_function = "ic_clock_delete"

    def __init__(self):
        self.dll = library()
        self.handle = self.dll.ic_clock_new()

    def observe(self, stamp, now):
        self.dll.ic_clock_observe(self.handle, int(stamp), now)

    def invalidate(self):
        self.dll.ic_clock_invalidate(self.handle)

    def fresh(self, now):
        return bool(self.dll.ic_clock_fresh(self.handle, now))

    def consume(self, now):
        elapsed = self.dll.ic_clock_consume(self.handle, now)
        return elapsed if math.isfinite(elapsed) else None

    def snapshot(self):
        state = ClockState()
        self.dll.ic_clock_state(self.handle, C.byref(state))
        return state

    @property
    def stamp(self):
        value = self.snapshot().stamp
        return value if value >= 0 else None

    @property
    def total_s(self):
        return self.snapshot().total

    @property
    def last_decision(self):
        s = self.snapshot()
        if not s.has_decision:
            return None
        return dict(
            consume_reason=REASONS[s.reason],
            decision_wall_s=s.decision_wall,
            source_stamp_ns=s.decision_stamp if s.decision_stamp >= 0 else None,
            source_delta_s=s.delta if math.isfinite(s.delta) else None,
            source_age_wall_s=s.decision_age if math.isfinite(s.decision_age) else None,
            unconsumed_sim_s=s.decision_elapsed,
            confirming=s.confirming,
        )

    def report(self, now):
        s = self.snapshot()
        return dict(
            available=self.fresh(now),
            consume_reason=REASONS[s.reason],
            source_stamp_ns=s.stamp if s.stamp >= 0 else None,
            source_delta_s=s.delta if math.isfinite(s.delta) else None,
            source_age_wall_s=now - s.received if math.isfinite(s.received) else None,
            confirming=s.confirming,
            last_discontinuity=DISCONTINUITIES[s.discontinuity],
            freshness_limit_wall_s=0.25,
            interval_limit_sim_s=0.25,
            decision=self.last_decision,
        )


class NativeSimulatorClock:
    def __init__(self, timeline):
        self.timeline = timeline

    @property
    def epoch(self):
        return self.timeline.dll.ic_timeline_epoch(self.timeline.handle)

    def observe(self, row, now, qpc):
        if (
            type(row["px4_us"]) is not int
            or type(row["sim_ns"]) is not int
            or not isinstance(row["session"], str)
        ):
            return
        self.timeline.dll.ic_timeline_pair(
            self.timeline.handle,
            row["session"].encode(),
            row["px4_us"],
            row["sim_ns"],
            row["sent_qpc_s"],
            now,
            qpc,
        )


class NativeSampleTimeline(NativeHandle):
    delete_function = "ic_timeline_delete"

    def __init__(self, port=18789):
        self.dll = library()
        self.handle = self.dll.ic_timeline_new(-1 if port is None else port)
        self.clock = NativeSimulatorClock(self)

    def poll(self, now):
        self.dll.ic_timeline_poll(self.handle, now)

    def observe_timesync(self, message, now):
        self.dll.ic_timeline_sync(
            self.handle,
            message.timestamp,
            message.estimated_offset,
            message.source_protocol,
            message.round_trip_time,
            now,
        )

    def sample_time(self, position, now):
        stamp = C.c_int64()
        reason = self.dll.ic_timeline_sample(
            self.handle, position.timestamp, position.timestamp_sample, now, C.byref(stamp)
        )
        return (stamp.value, None) if reason == 0 else (None, SAMPLE_REASONS[reason])


class NativeReferenceHistory(NativeHandle):
    delete_function = "ic_history_delete"

    def __init__(self):
        self.dll = library()
        self.handle = self.dll.ic_history_new()
        self.epoch = None
        self._generation = 0
        self._xy = (C.c_double * 2)()
        self._velocity = (C.c_double * 2)()
        self._result = (C.c_double * 2)()

    def clear(self):
        self.dll.ic_history_clear(self.handle)

    def record(self, stamp, xy, velocity, epoch):
        if stamp is None:
            return
        if self.epoch != epoch:
            self._generation += 1
        self.epoch = epoch
        self._xy[:] = xy
        self._velocity[:] = velocity
        self.dll.ic_history_record(self.handle, stamp, self._xy, self._velocity, self._generation)

    def at(self, stamp):
        mode = self.dll.ic_history_at(self.handle, stamp, self._result)
        return (
            (np.array(self._result), HISTORY_MODES[mode])
            if mode < 3
            else (None, HISTORY_MODES[mode])
        )


class NativeReaderCount:
    def __init__(self, publisher, target):
        self.publisher, self.target = publisher, target

    def get_subscription_count(self):
        return self.publisher.dll.ic_publisher_readers(self.publisher.handle, self.target)


class NativePublisher(NativeHandle):
    delete_function = "ic_publisher_delete"

    def __init__(self, timeline=None, clock=None, domain=None, prefix="/fmu/in"):
        self.dll = library()
        self.timeline, self.clock = timeline, clock  # Keep native callback dependencies alive.
        domain = int(os.environ.get("ROS_DOMAIN_ID", "0")) if domain is None else domain
        self.handle = self.dll.ic_publisher_new(
            domain,
            prefix.encode(),
            timeline.handle if timeline else None,
            clock.handle if clock else None,
            "QueryPerformanceCounter" in time.get_clock_info("monotonic").implementation,
        )
        if not self.handle:
            raise RuntimeError(self.dll.ic_error().decode())
        self.mode, self.target = NativeReaderCount(self, 0), NativeReaderCount(self, 1)
        self._frame = (C.c_double * 8)()

    def submit(self, position, velocity, acceleration, yaw, issued_ns=None):
        self._frame[:] = (*position, *velocity, *acceleration, yaw)
        issued_ns = time.perf_counter_ns() if issued_ns is None else issued_ns
        result = self.dll.ic_publisher_submit(self.handle, self._frame, issued_ns)
        self.last_submission = self.submission_report()
        if result != 1:
            detail = (
                self.dll.ic_error().decode()
                if self.last_submission["reason"] == "publication_error"
                else None
            )
            raise NativeSubmissionError(self.last_submission, detail)

    def submission_report(self, last_rejection=False):
        state = SubmissionState()
        self.dll.ic_publisher_submission(self.handle, C.byref(state), int(last_rejection))
        if not state.entered_ns:
            return None
        return dict(
            reason=SUBMISSION_REASONS[state.reason],
            issued_qpc_ns=state.issued_ns,
            entered_qpc_ns=state.entered_ns,
            previous_issued_qpc_ns=state.previous_issued_ns,
            command_age_s=(state.validated_ns - state.issued_ns) / 1e9,
            python_before_native_s=(state.entered_ns - state.issued_ns) / 1e9,
            lock_wait_s=(state.validated_ns - state.entered_ns) / 1e9,
            graph_s=state.graph_ns / 1e9,
            publish_s=state.publish_ns / 1e9,
            completion_age_s=(state.completed_ns - state.issued_ns) / 1e9,
            invalid_index=state.invalid_index,
            invalid_field=(
                ("x", "y", "z", "vx", "vy", "ax/ay", "ax/ay", "yaw")[state.invalid_index]
                if state.invalid_index >= 0
                else None
            ),
            mode_publishers=state.mode_publishers,
            target_publishers=state.target_publishers,
        )

    def revoke(self):
        self.dll.ic_publisher_revoke(self.handle)

    def report(self):
        stats = PublicationStats()
        self.dll.ic_publisher_stats(self.handle, C.byref(stats))
        return dict(
            backend="cpp",
            running=bool(self.dll.ic_publisher_alive(self.handle)),
            lease_s=0.1,
            last_rejection=self.submission_report(last_rejection=True),
            **{name: getattr(stats, name) for name, _ in stats._fields_},
        )

    def benchmark_start(self, count):
        start = self.dll.ic_publisher_benchmark(self.handle, count)
        if start < 0:
            raise RuntimeError(
                "Synthetic benchmark is forbidden on flight topics or active commands"
            )
        return start

    def trace_start(self):
        if not self.dll.ic_publisher_trace(self.handle):
            raise RuntimeError("Publication tracing requires idle private benchmark topics")

    def benchmark_count(self):
        return self.dll.ic_publisher_benchmark_count(self.handle)

    def benchmark_rows(self):
        rows = []
        for index in range(self.benchmark_count()):
            stamp, duration = C.c_int64(), C.c_double()
            self.dll.ic_publisher_benchmark_row(
                self.handle, index, C.byref(stamp), C.byref(duration)
            )
            rows.append((stamp.value, duration.value))
        return rows
