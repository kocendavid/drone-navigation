"""Repeatable control-component measurements on an isolated ROS domain."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def distribution(values):
    values = sorted(values)
    if not values:
        return dict(samples=0)
    return dict(
        samples=len(values),
        mean=statistics.mean(values),
        p50=values[len(values) // 2],
        p95=values[min(len(values) - 1, math.ceil(len(values) * 0.95) - 1)],
        p99=values[min(len(values) - 1, math.ceil(len(values) * 0.99) - 1)],
        maximum=values[-1],
    )


def core_measurement(backend, iterations):
    from navigation.source_time import SampleTimeline, ReferenceHistory
    from navigation.timing import TrajectoryClock

    if backend != "python":
        from navigation.native_control import (
            NativeTrajectoryClock,
            NativeSampleTimeline,
            NativeReferenceHistory,
        )

        TrajectoryClock, SampleTimeline, ReferenceHistory = (
            NativeTrajectoryClock,
            NativeSampleTimeline,
            NativeReferenceHistory,
        )
    clock, timeline, history = TrajectoryClock(), SampleTimeline(port=None), ReferenceHistory()
    clock_times, alignment_times, history_times = [], [], []
    checksum = 0
    aligned = 0
    for index in range(iterations + 256):
        sim = 1_000_000_000 + index * 50_000_000
        native = sim // 1000 + 1_000_000
        now = 1000 + index * 0.05
        began = time.perf_counter_ns()
        clock.observe(sim, now)
        clock.consume(now)
        clock.fresh(now)
        elapsed = time.perf_counter_ns() - began
        if index >= 256:
            clock_times.append(elapsed / 1000)
        row = dict(session="benchmark", px4_us=native, sim_ns=sim, sent_qpc_s=now)
        timeline.clock.observe(row, now, now)
        if index % 10 == 0:
            from types import SimpleNamespace

            timeline.observe_timesync(
                SimpleNamespace(
                    timestamp=native + 1_000_000_000_000,
                    estimated_offset=-1_000_000_000_000,
                    source_protocol=2,
                    round_trip_time=0,
                ),
                now,
            )
        from types import SimpleNamespace

        position = SimpleNamespace(
            timestamp=native + 1_000_000_000_000,
            timestamp_sample=native + 1_000_000_000_000 - 20_000,
        )
        began = time.perf_counter_ns()
        stamp, _ = timeline.sample_time(position, now)
        elapsed = time.perf_counter_ns() - began
        if index >= 256:
            alignment_times.append(elapsed / 1000)
        began = time.perf_counter_ns()
        history.record(sim, [index * 0.03, 2], [0.6, 0], 0)
        point, _ = history.at(stamp if stamp is not None else sim)
        elapsed = time.perf_counter_ns() - began
        if index >= 256:
            history_times.append(elapsed / 1000)
            aligned += stamp is not None
            checksum += round(point[0] * 1_000_000)
    timeline.close()
    for item in (clock, history):
        if hasattr(item, "close"):
            item.close()
    return dict(
        units="microseconds",
        clock_observe_consume_fresh=distribution(clock_times),
        sample_clock_conversion=distribution(alignment_times),
        reference_record_and_interpolate_128=distribution(history_times),
        aligned=aligned,
        checksum=checksum,
    )


def ros_measurement(backend, seconds, loaded, timer_only=False):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
    from ros2_hover import hover_messages

    qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
    )
    node = Node("python_control_benchmark")
    native = None
    if backend == "cpp":
        from navigation.native_control import NativePublisher

        native = NativePublisher(domain=97, prefix="/benchmark")
        mode, target = native.mode, native.target
    else:
        mode = node.create_publisher(OffboardControlMode, "/benchmark/offboard_control_mode", qos)
        target = node.create_publisher(TrajectorySetpoint, "/benchmark/trajectory_setpoint", qos)
    received, issued, durations, cpu = {}, [], [], []
    completed = threading.Event()

    def receive(message):
        received.setdefault(int(message.position[0]), time.perf_counter_ns())

    node.create_subscription(TrajectorySetpoint, "/benchmark/trajectory_setpoint", receive, qos)
    deadline = time.monotonic() + 5
    while target.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    if target.get_subscription_count() == 0:
        status = native.report() if native is not None else None
        if native:
            native.close()
        raise RuntimeError(f"Benchmark subscriber did not discover publisher: {status}")
    start_cpu = time.process_time()
    start = time.perf_counter_ns()
    samples = round(seconds * 20)

    def publish():
        began = time.perf_counter_ns()
        index = len(issued)
        if native:
            native.submit([index, 0, -1], [0.6, 0], [0.25, 0], 0, issued_ns=began)
        else:
            m, t = hover_messages(0, index, 0, 1, 0)
            t.velocity = [0.6, 0.0, math.nan]
            t.acceleration = [0.25, 0.0, math.nan]
            mode.publish(m)
            target.publish(t)
        durations.append((time.perf_counter_ns() - began) / 1000)
        issued.append(began)
        if len(issued) >= samples:
            completed.set()

    timer = (
        node.create_timer(0.05, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if native is None or not timer_only
        else None
    )
    stop_load = threading.Event()

    def background():
        while not stop_load.is_set():
            until = time.perf_counter() + 0.01
            value = 1
            while time.perf_counter() < until:
                value = (value * 1664525 + 1013904223) & 0xFFFFFFFF
            stop_load.wait(0.01)

    worker = threading.Thread(target=background) if loaded else None
    if worker:
        worker.start()
    try:
        if native is not None:
            if timer_only:
                start = native.benchmark_start(samples)
            else:
                native.trace_start()
        deadline = time.monotonic() + seconds + 5
        while (
            not (
                native.benchmark_count() >= samples if native and timer_only else completed.is_set()
            )
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.01)
        if timer is not None:
            timer.cancel()
        if native is not None and not timer_only:
            native.revoke()
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        stop_load.set()
        if worker:
            worker.join()
        if native is not None:
            rows = native.benchmark_rows()
            output_stamps = [row[0] for row in rows]
            if timer_only:
                issued = output_stamps
                durations = [row[1] for row in rows]
            native.close()
        else:
            output_stamps = issued
        cpu.append(time.process_time() - start_cpu)
        node.destroy_node()
    if len(issued) != samples:
        raise RuntimeError(f"Only {len(issued)}/{samples} benchmark ticks completed")
    intervals = [(b - a) / 1e6 for a, b in zip(output_stamps, output_stamps[1:])]
    lateness = [(stamp - (start + (i + 1) * 50_000_000)) / 1e6 for i, stamp in enumerate(issued)]
    latency = [(received[i] - stamp) / 1000 for i, stamp in enumerate(issued) if i in received]
    return dict(
        scenario="background_python_cpu" if loaded else "idle",
        samples=len(issued),
        published_samples=len(output_stamps),
        received=len(latency),
        publication_us=distribution(durations),
        interval_ms=distribution(intervals),
        absolute_interval_error_ms=distribution([abs(v - 50) for v in intervals]),
        overdue_gap_ms=distribution([max(0, v - 50) for v in intervals]),
        scheduled_lateness_ms=distribution(lateness),
        dds_observation_us=distribution(latency),
        process_cpu_s=cpu[0],
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("python", "cpp"), default="python")
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--iterations", type=int, default=10000)
    parser.add_argument(
        "--timer-only",
        action="store_true",
        help="Isolate native timer capacity; do not measure Python submissions",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (
        not math.isfinite(args.seconds)
        or not 0 < args.seconds <= 120
        or not 1 <= args.iterations <= 1_000_000
    ):
        parser.error("Require 0 < seconds <= 120 and 1 <= iterations <= 1000000")
    if os.environ.get("ROS_DOMAIN_ID") != "97":
        raise RuntimeError("Benchmark requires isolated ROS_DOMAIN_ID=97")
    import rclpy

    rclpy.init()
    try:
        result = dict(
            schema=1,
            backend=args.backend,
            revision=subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
            platform=platform.platform(),
            processor=platform.processor(),
            python=sys.version,
            domain=97,
            rmw=os.environ.get("RMW_IMPLEMENTATION"),
            seconds_per_scenario=args.seconds,
            publication_mode="native_timer_only" if args.timer_only else "python_submission",
            iterations=args.iterations,
            notes=[
                "Isaac stopped; isolated replay/component benchmark, not flight validation.",
                "High-resolution perf_counter; DDS observer is Python for both backends.",
                "Background load is 10 ms Python CPU work followed by 10 ms wait.",
            ],
            core=core_measurement(args.backend, args.iterations),
            measured_utc=datetime.now(timezone.utc).isoformat(),
            harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            working_tree_dirty=bool(
                subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True)
            ),
        )
        if args.backend == "cpp":
            from navigation.native_control import library

            result["native_source_sha256"] = library().ic_source_hash().decode()
            result["notes"].append(
                "C++ uses a high-resolution waitable timer on a native thread; this compares implementations, not language alone."
            )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        result["ros"] = [
            ros_measurement(args.backend, args.seconds, loaded, args.timer_only)
            for loaded in (False, True)
        ]
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
