"""Measure the production history pipe without ROS or flight commands."""

import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import pickle
import platform
import statistics
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
import scipy
from navigation.centering import center_geometry
from navigation.feed import MapSnapshot
from navigation.history import RunHistory, retained_bytes
from navigation.mapping import RayMap, FREE, OCCUPIED
from navigation.process_worker import ProcessService
from navigation.settings import load_settings


def distribution(values):
    ordered = sorted(values)
    return dict(
        samples=len(values),
        median=statistics.median(values),
        p95=ordered[math.ceil(len(values) * 0.95) - 1],
        maximum=max(values),
        raw=values,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=240)
    parser.add_argument(
        "--output", type=Path, default=ROOT / "benchmarks/reports/history_current.json"
    )
    args = parser.parse_args()
    if args.samples < 101:
        parser.error("--samples must cover at least one five-second interval at 20 Hz")
    settings = load_settings()
    model = RayMap(**settings.map_options())
    model.grid[:] = FREE
    model.grid[[0, -1], :] = OCCUPIED
    model.grid[:, [0, -1]] = OCCUPIED
    model.confirmed_walls[:] = model.grid == OCCUPIED
    clearance, body, safe = model.geometry()
    centers, centerline = center_geometry(model, clearance, safe)
    now = time.monotonic()
    snap = MapSnapshot(
        model,
        clearance,
        body,
        safe,
        [*model.world((model.grid.shape[0] // 2, model.grid.shape[1] // 2)), 0.0, 1.0],
        np.zeros((720, 2)),
        1_000_000_000,
        now,
        now,
        now,
        True,
        "Synthetic history benchmark",
        3,
        centers,
        centerline,
    )
    report = dict(
        phase="NAVIGATING",
        reason="Following route",
        goal=[1.0, 0.0],
        delivered_simulation_s=0.0,
        trajectory_simulation_s=0.0,
        speed_decision=dict(selected_speed_m_s=0.5),
    )
    payload = ("state", report, snap, now)
    encoded = pickle.dumps(payload, protocol=5)
    pickle_ms = []
    for _ in range(args.samples):
        began = time.perf_counter_ns()
        pickle.dumps(payload, protocol=5)
        pickle_ms.append((time.perf_counter_ns() - began) / 1e6)
    cases = {}
    with tempfile.TemporaryDirectory(prefix="history-benchmark-") as folder:
        for name in ("same_snapshot", "new_scans", "changing_reason"):
            path = Path(folder) / (name + ".jsonl")
            service = ProcessService(
                "history_benchmark", "navigation.report_jobs:HistoryProcessor", path, name
            )
            latencies, requests, responses, frame_count, retained, full_maps = (
                [],
                [],
                [],
                [],
                [],
                [],
            )
            sender = RunHistory()
            result = None

            def submit(item):
                nonlocal result
                # Use production frame selection and telemetry construction;
                # only the asynchronous queue is bypassed for isolated RPC timing.
                requests.append(len(pickle.dumps(item, protocol=5)) + 8)
                retained.append(retained_bytes(item))
                full_maps.append(item[2] is not None)
                began = time.perf_counter_ns()
                result = service.call(item)
                latencies.append((time.perf_counter_ns() - began) / 1e6)
                responses.append(len(pickle.dumps((result, None), protocol=5)) + 8)
                frame_count.append(len(result[1]))
                return True

            sender._submit = submit
            try:
                # Start/import the child before measuring; event does not prime frame selection.
                service.call(("event", "benchmark", "warmup", {}))
                for i in range(args.samples):
                    state = dict(
                        report, delivered_simulation_s=i / 20, trajectory_simulation_s=i / 20
                    )
                    if name == "changing_reason":
                        state["reason"] = "Decision " + str(i % 2)
                    current = snap if name == "same_snapshot" else replace(snap, scans=i + 3)
                    assert sender.observe(state, current, now)
                summary, offsets = result
                assert summary["error"] is None
                expected = (
                    args.samples
                    if name == "changing_reason"
                    else (1 + (args.samples - 1) // 100 if name == "new_scans" else 1)
                )
                assert len(offsets) == expected, (name, len(offsets), expected)
                assert sum(full_maps) == expected
                cases[name] = dict(
                    rpc_wall_ms=distribution(latencies),
                    request_bytes=distribution(requests),
                    response_bytes=distribution(responses),
                    frame_counts=frame_count,
                    recorded_frames=len(offsets),
                    full_map_requests=sum(full_maps),
                    telemetry_requests=args.samples - sum(full_maps),
                    retained_request_bytes=distribution(retained),
                    request_mib_s_at_20_hz=statistics.mean(requests) * 20 / 2**20,
                )
            finally:
                service.close()
        # Exercise the actual queue and writer at 20 wall-clock reports/s for six seconds.
        history = RunHistory(Path(folder) / "paced.jsonl")
        try:
            history.event("benchmark", "warmup")
            assert history.flush(timeout=10)
            paced_maps, paced_bytes = [], []
            original_call = history.service.call

            def paced_call(item):
                paced_maps.append(item[2] is not None)
                paced_bytes.append(len(pickle.dumps(item, protocol=5)) + 8)
                return original_call(item)

            history.service.call = paced_call
            enqueue_ms, depth = [], []
            start = time.perf_counter()
            for i in range(120):
                time.sleep(max(0, start + i / 20 - time.perf_counter()))
                captured = time.monotonic()
                current = replace(
                    snap, scans=i + 3, generated=captured, cloud_wall=captured, clock_wall=captured
                )
                state = dict(report, delivered_simulation_s=i / 20)
                began = time.perf_counter_ns()
                history.observe(state, current, captured)
                enqueue_ms.append((time.perf_counter_ns() - began) / 1e6)
                depth.append(history.queue.qsize())
            assert history.flush(timeout=10)
            summary = history.report()
            assert summary["error"] is None and summary["dropped_records"] == 0
            assert len(summary["frame_ids"]) == 2
            assert sum(paced_maps) == 2 and len(paced_maps) == 120
            assert summary["queue"]["bytes"] == 0
            assert summary["queue"]["peak_bytes"] <= summary["queue"]["max_bytes"]
            paced = dict(
                enqueue_wall_ms=distribution(enqueue_ms),
                queue_depth=distribution(depth),
                dropped_records=summary["dropped_records"],
                recorded_frames=len(summary["frame_ids"]),
                full_map_requests=sum(paced_maps),
                telemetry_requests=len(paced_maps) - sum(paced_maps),
                request_bytes=distribution(paced_bytes),
                queue_memory=summary["queue"],
            )
        finally:
            history.close()
    sources = [
        "benchmarks/history_benchmark.py",
        "navigation/history.py",
        "navigation/report_jobs.py",
        "navigation/process_worker.py",
        "navigation/feed.py",
        "navigation/mapping.py",
        "navigation/centering.py",
        "navigation/dashboard.py",
        "ros2_navigate.py",
        "config/navigation_settings.json",
    ]
    output = dict(
        recorded_utc=datetime.now(timezone.utc).isoformat(),
        revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        dirty=bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True)),
        python=sys.version,
        numpy=np.__version__,
        scipy=scipy.__version__,
        platform=platform.platform(),
        source_sha256={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in sources},
        settings=asdict(settings),
        workload="Synthetic open map, 720 zero-valued hit endpoints, compact report; active lab shares host. RPCs use production observe/selection, bypassing only its queue; simulation time steps 0.05 s. Paced trial uses the real queue. Temporary recordings removed after measurement.",
        snapshot_pickle_bytes=len(pickle.dumps(snap, protocol=5)),
        legacy_request_pickle_bytes=len(encoded),
        legacy_request_mib_s_at_20_hz=(len(encoded) + 8) * 20 / 2**20,
        pickle_wall_ms=distribution(pickle_ms),
        cases=cases,
        paced=paced,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            dict(
                output=str(args.output),
                snapshot_bytes=output["snapshot_pickle_bytes"],
                cases={
                    k: dict(
                        frames=v["recorded_frames"],
                        full_map_requests=v["full_map_requests"],
                        rpc_median_ms=v["rpc_wall_ms"]["median"],
                        rpc_p95_ms=v["rpc_wall_ms"]["p95"],
                        mib_s=v["request_mib_s_at_20_hz"],
                    )
                    for k, v in cases.items()
                },
                paced_drops=paced["dropped_records"],
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
