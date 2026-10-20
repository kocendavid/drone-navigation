"""Offline planning kernels and real HTTP latency, with no ROS/flight publishers."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import pickle
import platform
from queue import Queue
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from unittest.mock import patch
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[variable] = "1"

import numpy as np
import scipy
from navigation.native_planning import NativePlanning as NativeKernels
from navigation.async_frontiers import FrontierAnalyzer
from navigation.centering import center_geometry, routing_geometry
from navigation.dashboard import Dashboard
from navigation.feed import MapSnapshot
from navigation.exploration import choose_frontier
from navigation.map_geometry import MapGeometry
from navigation.mapping import RayMap, FREE, OCCUPIED, UNKNOWN
from navigation.mission import assess_completion_python as assess_completion
from navigation.planning import astar_python as astar, path_cost, plan, segment_safe
from navigation.process_worker import ProcessService
from navigation.report_jobs import PresentationJobs
from navigation.settings import load_settings


def distribution(values):
    ordered = sorted(values)
    return dict(
        samples=len(ordered),
        p50=statistics.median(ordered),
        p95=ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)],
        maximum=ordered[-1],
        mean=statistics.mean(ordered),
        raw=values,
    )


def measure(function, iterations):
    function()  # Exclude initial imports, allocations and first-call caches.
    elapsed, cpu = [], []
    for _ in range(iterations):
        wall_start, cpu_start = time.perf_counter_ns(), time.process_time_ns()
        function()
        cpu.append((time.process_time_ns() - cpu_start) / 1e6)
        elapsed.append((time.perf_counter_ns() - wall_start) / 1e6)
    return dict(wall_ms=distribution(elapsed), cpu_ms=distribution(cpu))


def fixtures():
    settings = load_settings()
    h, w = settings.map_height_cells, settings.map_width_cells
    result = []
    for name in ("open", "maze", "partial_maze"):
        model = RayMap(**settings.map_options())
        model.grid[:] = OCCUPIED
        model.grid[4:-4, 4:-4] = FREE
        if name != "open":
            model.grid[4:-4, w // 2 : w // 2 + 3] = OCCUPIED
            model.grid[int(h * 0.60) : int(h * 0.72), w // 2 : w // 2 + 3] = FREE
        if name == "partial_maze":
            model.grid[30:80, w - 90 : w - 40] = UNKNOWN
        model.confirmed_walls = model.grid == OCCUPIED
        clearance, body, safe = model.geometry()
        centers, centerline = center_geometry(model, clearance, safe)
        start_cell, goal_cell = (h // 4, w // 6), (h // 3, w * 5 // 6)
        start, goal = model.world(start_cell), model.world(goal_cell)
        now = time.monotonic()
        snapshot = MapSnapshot(
            model,
            clearance,
            body,
            safe,
            [*start, 0.0, settings.floor_z_m + 1.0],
            np.empty((0, 2)),
            1_000_000_000,
            now,
            now,
            now,
            True,
            "Synthetic benchmark",
            3,
            centers,
            centerline,
        )
        if not safe[start_cell] or not safe[goal_cell]:
            raise ValueError("Benchmark endpoints require safe configured map dimensions")
        wire = dict(
            width=w,
            height=h,
            grid="".join(np.array(["?", ".", "#", "b"])[model.grid.ravel()].tolist()),
            confirmed_walls="".join(np.where(model.confirmed_walls.ravel(), "1", "0").tolist()),
        )
        result.append((name, snapshot, start, goal, wire))
    return settings, result


def validate(kernels, cases):
    rng = np.random.default_rng(20261020)
    checked = 0
    for _ in range(100):
        mask = rng.random((17, 23)) > 0.24
        mask[1, 1] = mask[-2, -2] = True
        cost = 1 + rng.random(mask.shape) * 8
        outputs = []
        for search in (astar, kernels.astar):
            try:
                outputs.append(search(mask, (1, 1), (15, 21), cost))
            except ValueError:
                outputs.append(None)
        if (outputs[0] is None) != (outputs[1] is None):
            raise AssertionError("Reachability differs between kernels")
        if outputs[0] is not None:
            np.testing.assert_allclose(
                path_cost(outputs[0], cost), path_cost(outputs[1], cost), rtol=1e-12
            )
            if not all(segment_safe(mask, a, b) for a, b in zip(outputs[1], outputs[1][1:])):
                raise AssertionError("Native search cut a corner")
        sample = cases[0][1]
        model = RayMap(resolution=0.2, size=(17, 23), origin=(0, 0))
        model.grid = rng.integers(0, 4, size=(17, 23), dtype=np.uint8)
        sample = replace(sample, model=model)
        point = model.world((int(rng.integers(17)), int(rng.integers(23))))
        if assess_completion(sample, point) != kernels.completion(sample, point):
            raise AssertionError("Completion evidence differs")
        checked += 1
    parity = {}
    for name, snap, start, goal, _ in cases:
        with patch("navigation.planning.astar", astar):
            expected = plan(snap, start, goal)
        with patch("navigation.planning.astar", kernels.astar):
            actual = plan(snap, start, goal)
        np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
        if assess_completion(snap, start) != kernels.completion(snap, start):
            raise AssertionError("Fixture completion differs")
        parity[name] = dict(
            identical_full_route=True,
            points=len(expected),
            length_m=sum(math.dist(a, b) for a, b in zip(expected, expected[1:])),
            route_sha256=hashlib.sha256(
                np.asarray(expected, dtype=np.float64).tobytes()
            ).hexdigest(),
        )
    return dict(random_search_and_completion_cases=checked, fixtures=parity)


def components(kernels, settings, cases, iterations):
    results = {}
    for name, snap, start, goal, wire in cases:
        print(f"Components: {name}", flush=True)
        _, _, cost = routing_geometry(snap)
        cached = MapGeometry(settings)
        cached(wire)
        jobs = PresentationJobs()
        result = dict(
            map_geometry_cold=measure(lambda: MapGeometry(settings)(wire), iterations),
            map_geometry_cached=measure(lambda: cached(wire), iterations),
            routing_cost=measure(lambda: routing_geometry(snap), iterations),
            map_presentation=measure(lambda: jobs(("map", snap)), iterations),
            snapshot_pickle=measure(lambda: pickle.dumps(snap, protocol=5), iterations),
            snapshot_pickle_bytes=len(pickle.dumps(snap, protocol=5)),
        )
        # Alternate backend order to reduce systematic warmup/order effects.
        for backend in (("python", "cpp") if name != "maze" else ("cpp", "python")):
            search = astar if backend == "python" else kernels.astar
            completion = assess_completion if backend == "python" else kernels.completion
            result[backend] = dict(
                weighted_astar=measure(
                    lambda: search(snap.safe, snap.model.cell(start), snap.model.cell(goal), cost),
                    iterations,
                ),
                completion=measure(lambda: completion(snap, start), iterations),
            )
            analyzer = FrontierAnalyzer()
            sequence = [10]

            def analyze_map():
                sequence[0] += 1
                return analyzer((replace(snap, stamp=sequence[0], scans=sequence[0]), None, 0))

            with patch("navigation.async_frontiers.assess_completion", completion):
                result[backend]["frontier_analysis"] = measure(analyze_map, iterations)
            with patch("navigation.planning.astar", search):
                result[backend]["full_plan"] = measure(
                    lambda: plan(snap, start, goal), max(5, iterations // 3)
                )
                if name == "partial_maze":
                    result[backend]["frontier_choice"] = measure(
                        lambda: choose_frontier(snap, start), max(3, iterations // 4)
                    )
        results[name] = result
    return results


class PreviewJobs(PresentationJobs):
    """Benchmark subprocess handler: identical production jobs plus a start marker."""

    def __init__(self, backend, marker):
        self.kernels = NativeKernels() if backend == "cpp" else None
        self.marker = Path(marker)

    def __call__(self, item):
        if item[0] != "plan":
            return super().__call__(item)
        self.marker.write_text("started", encoding="ascii")
        with patch("navigation.planning.astar", self.kernels.astar if self.kernels else astar):
            return super().__call__(item)


class SyntheticFeed:
    def __init__(self, snapshot):
        self.snapshot = snapshot

    def get(self):
        now = time.monotonic()
        return replace(self.snapshot, generated=now, cloud_wall=now, clock_wall=now), None


def http_trial(backend, split, snapshot, goal, repeats):
    commands = Queue(maxsize=32)  # No consumer/backend: commands cannot reach a vehicle.
    with tempfile.TemporaryDirectory(prefix="navigation_benchmark_") as folder:
        marker = Path(folder) / "started.txt"
        hub = Dashboard(SyntheticFeed(snapshot), commands=commands, port=0)
        original = hub.presentation
        hub.preview_planning.close()
        preview = ProcessService(
            "benchmark_preview", "benchmarks.navigation_benchmark:PreviewJobs", backend, str(marker)
        )
        hub.preview_planning = preview
        if not split:
            original.close()
            hub.presentation = preview

        def request(route, payload=None):
            data = json.dumps(payload).encode() if payload is not None else None
            headers = {"Content-Type": "application/json", "X-Navigation-Token": hub.token}
            began = time.perf_counter_ns()
            with urlopen(
                Request(hub.url + route, data=data, headers=headers), timeout=60
            ) as response:
                body = json.load(response)
            return (time.perf_counter_ns() - began) / 1e6, body

        rows = {
            name: []
            for name in (
                "idle_state_ms",
                "idle_ack_ms",
                "loaded_state_ms",
                "loaded_ack_ms",
                "preview_ms",
            )
        }
        try:
            with patch("navigation.dashboard.read_status", return_value={"recording": False}):
                request("/api/preview", {"point": goal.tolist()})
                request("/api/state")
                for index in range(repeats):
                    elapsed, _ = request("/api/state")
                    rows["idle_state_ms"].append(elapsed)
                    elapsed, _ = request(
                        "/api/command", {"action": "hold", "request_id": f"idle-{index}"}
                    )
                    rows["idle_ack_ms"].append(elapsed)
                    commands.get_nowait()
                    marker.unlink(missing_ok=True)
                    with ThreadPoolExecutor(max_workers=3) as executor:
                        pending = executor.submit(request, "/api/preview", {"point": goal.tolist()})
                        deadline = time.perf_counter() + 10
                        while not marker.exists():
                            if pending.done():
                                pending.result()  # Surface an early planning error.
                                raise RuntimeError("Preview ended before its start marker")
                            if time.perf_counter() > deadline:
                                raise TimeoutError("Preview worker failed to start")
                            time.sleep(0.001)
                        state = executor.submit(request, "/api/state")
                        acknowledgement = executor.submit(
                            request,
                            "/api/command",
                            {"action": "hold", "request_id": f"loaded-{index}"},
                        )
                        state_time, state_body = state.result()
                        ack_time, ack_body = acknowledgement.result()
                        preview_time, _ = pending.result()
                    if not state_body["map_fresh"] or ack_body["command"]["status"] != "queued":
                        raise AssertionError(
                            "HTTP fixture did not return expected live/queued state"
                        )
                    queued = commands.get_nowait()
                    if queued["action"] != "hold":
                        raise AssertionError("Unexpected fake command")
                    rows["loaded_state_ms"].append(state_time)
                    rows["loaded_ack_ms"].append(ack_time)
                    rows["preview_ms"].append(preview_time)
        finally:
            hub.close()
        return {name: distribution(values) for name, values in rows.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=24)
    parser.add_argument("--requests", type=int, default=12)
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (
        not 3 <= args.iterations <= 200
        or not 3 <= args.requests <= 100
        or not 1 <= args.rounds <= 5
    ):
        parser.error("Use 3..200 component iterations, 3..100 HTTP requests, and 1..5 rounds")
    kernels = NativeKernels()
    settings, cases = fixtures()
    print("Validating native/reference parity", flush=True)
    result = dict(
        schema=1,
        measured_utc=datetime.now(timezone.utc).isoformat(),
        platform=platform.platform(),
        processor=platform.processor(),
        python=sys.version,
        numpy=np.__version__,
        scipy=scipy.__version__,
        cpu_count=os.cpu_count(),
        revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        settings=asdict(settings),
        iterations=args.iterations,
        requests=args.requests,
        http_rounds=args.rounds,
        source_hashes={
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [
                *sorted((ROOT / "navigation").glob("*.py")),
                *sorted((ROOT / "benchmarks").glob("navigation*.py")),
                ROOT / "native_navigation/kernels.cpp",
                ROOT / "native_navigation/CMakeLists.txt",
            ]
        },
        native_source_hash=kernels.source_hash,
        scope=[
            "Deterministic synthetic grids at current configured map dimensions; warm steady-state timings.",
            "Production C++ weighted A* and completion BFS; full_plan keeps Python smoothing/safety/scoring.",
            "Real localhost HTTP handlers and ProcessService; shared case deliberately joins preview/status workers. Command acknowledgements encode inline in every case.",
            "No ROS node, flight publisher, or simulator command.",
            "Parent process_time excludes child CPU; HTTP reports wall latency only. Native component timings include ctypes/data conversion.",
            "Not a complete native planner, live-flight comparison, network-loss diagnosis, or guaranteed speedup.",
            "Prototype uses dense C++ arrays instead of Python dictionaries/sets; this compares implementations, not language alone.",
        ],
        validation=validate(kernels, cases),
    )
    result["components"] = components(kernels, settings, cases, args.iterations)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    _, snap, _, goal, _ = cases[1]
    result["http"], result["http_trials"] = {}, []
    order = [("python", False), ("cpp", True), ("cpp", False), ("python", True)]
    for round_index in range(args.rounds):
        for backend, split in order if round_index % 2 == 0 else reversed(order):
            key = f"{'split' if split else 'shared'}_{backend}"
            print(f"HTTP round {round_index + 1}: {key}", flush=True)
            trial = http_trial(backend, split, snap, goal, args.requests)
            result["http_trials"].append(dict(round=round_index + 1, variant=key, timings=trial))
            previous = result["http"].get(key, {})
            result["http"][key] = {
                metric: distribution(previous.get(metric, {}).get("raw", []) + value["raw"])
                for metric, value in trial.items()
            }
            args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {args.output.resolve()}", flush=True)


if __name__ == "__main__":
    main()
