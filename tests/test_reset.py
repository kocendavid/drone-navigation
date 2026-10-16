"""Reset sequencing, repeatability, preserved history and service failure checks."""

import json
from pathlib import Path
import subprocess
import tempfile
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from navigation.dashboard import Dashboard
from navigation.mapping import RayMap, FREE
from navigation.reset import ExperimentReset, ResetBackend
from queue import Queue
from urllib.error import HTTPError
from urllib.request import Request, urlopen


class FakeBackend:
    def __init__(self, fail=None):
        self.calls = []
        self.stopped = False
        self.fail = fail
        self.checks = iter([True, False, True, True, True])

    def inspect_scene(self):
        return dict(
            scene="test.usd",
            stopped=self.stopped,
            start_xyz=[-28.0, -17.0, -3.55] if self.stopped else [26.0, -15.0, -2.95],
            start_up=[0.0, 0.0, 1.0],
        )

    def hold(self):
        self.calls.append("hold")
        if self.fail == "hold":
            raise RuntimeError("Hold not accepted")

    def stop_scene(self):
        self.calls.append("stop_scene")
        self.stopped = True

    def service(self, action, name):
        self.calls.append((action, name))
        if self.fail == name:
            raise RuntimeError("Service failed: " + name)

    def load_bridge(self):
        self.calls.append("load_bridge")

    def play(self):
        self.calls.append("play")
        self.stopped = False

    def ground_ready(self):
        self.calls.append("ground_ready")
        return next(self.checks, True)


def snapshot():
    model = RayMap(size=20, origin=(-2.0, -2.0), resolution=0.2)
    model.grid[2:-2, 2:-2] = FREE
    return SimpleNamespace(model=model, stamp=100, pose=[0.0, 0.0, 0.0, 1.0])


class ResetTests(unittest.TestCase):
    def test_reset_requests_hold_before_a_slow_or_failed_isaac_inspection(self):
        with tempfile.TemporaryDirectory() as folder:
            backend = FakeBackend()

            def unavailable_scene():
                self.assertEqual(backend.calls, ["hold"])
                raise RuntimeError("Isaac inspection unavailable")

            backend.inspect_scene = unavailable_scene
            reset = ExperimentReset(backend, folder)
            reset.start(None, {})
            self.assertTrue(reset.done.wait(2))
            self.assertFalse(reset.success)
            self.assertFalse(reset.quiesce.is_set())
            self.assertEqual(backend.calls, ["hold"])

    def test_launcher_returns_even_when_background_child_retains_output_handles(self):
        code = (
            "import subprocess,sys; "
            "subprocess.Popen([sys.executable,'-c','import time; time.sleep(2)']); "
            "print('launcher finished', flush=True)"
        )
        # A PIPE-based communicate(timeout=0.8) waits for the child's EOF.
        result = ResetBackend().command([sys.executable, "-c", code], timeout=0.8)
        self.assertIn("launcher finished", result)

    def test_reset_waits_for_publisher_release_then_restores_disarmed_baseline(self):
        with tempfile.TemporaryDirectory() as folder:
            backend = FakeBackend()
            reset = ExperimentReset(backend, folder)
            with patch("navigation.reset.time.sleep"):
                reset.start(snapshot(), dict(trail=[[1.0, 2.0]], visited_viewpoints=[[3.0, 4.0]]))
                self.assertTrue(reset.quiesce.wait(2))
                self.assertEqual(backend.calls, ["hold"])
                with self.assertRaises(ValueError):
                    reset.start(None, {})
                reset.released.set()
                self.assertTrue(reset.done.wait(2))
            self.assertTrue(reset.success, reset.report())
            self.assertEqual(
                backend.calls[:7],
                [
                    "hold",
                    "stop_scene",
                    ("stop", "px4"),
                    ("stop", "agent"),
                    ("start", "agent"),
                    ("start", "px4"),
                    "load_bridge",
                ],
            )
            self.assertEqual(backend.calls.count("ground_ready"), 5)
            self.assertEqual(reset.report()["phase"], "READY")
            self.assertFalse(reset.report()["busy"])
            baseline = json.loads((Path(folder) / "logs/experiment_baseline.json").read_text())
            self.assertEqual(baseline["start_xyz"], [-28.0, -17.0, -3.55])
            archive = Path(reset.report()["archive"])
            self.assertTrue((archive / "map.pgm").exists())
            self.assertEqual(json.loads((archive / "run.json").read_text())["trail"], [[1.0, 2.0]])

    def test_hold_failure_never_stops_simulator_or_releases_publishers(self):
        with tempfile.TemporaryDirectory() as folder:
            backend = FakeBackend("hold")
            reset = ExperimentReset(backend, folder)
            reset.start(None, {})
            self.assertTrue(reset.done.wait(2))
            self.assertFalse(reset.quiesce.is_set())
            self.assertEqual(backend.calls, ["hold"])
            self.assertFalse(reset.success)
            self.assertEqual(reset.report()["phase"], "FAILED")

    def test_service_failure_leaves_timeline_stopped_without_play_or_arming(self):
        with tempfile.TemporaryDirectory() as folder:
            backend = FakeBackend("agent")
            reset = ExperimentReset(backend, folder)
            reset.start(None, {})
            self.assertTrue(reset.quiesce.wait(2))
            reset.released.set()
            self.assertTrue(reset.done.wait(2))
            self.assertTrue(backend.stopped)
            self.assertNotIn("play", backend.calls)
            self.assertEqual(reset.report()["phase"], "FAILED")

    def test_changed_starting_pose_is_not_silently_accepted_as_same_experiment(self):
        with tempfile.TemporaryDirectory() as folder:
            logs = Path(folder) / "logs"
            logs.mkdir()
            (logs / "experiment_baseline.json").write_text(
                json.dumps(
                    dict(
                        scene="test.usd",
                        start_xyz=[0.0, 0.0, 0.0],
                    )
                )
            )
            backend = FakeBackend()
            reset = ExperimentReset(backend, folder)
            reset.start(None, {})
            self.assertTrue(reset.quiesce.wait(2))
            reset.released.set()
            self.assertTrue(reset.done.wait(2))
            self.assertFalse(reset.success)
            self.assertIn("starting pose changed", reset.report()["reason"])
            self.assertNotIn(("start", "px4"), backend.calls)

    def test_backend_rejects_sih_before_requesting_any_mode_change(self):
        backend = ResetBackend()
        backend.px4 = lambda *args: self.fail("Must not send a mode command to SIH")
        with patch(
            "navigation.reset.subprocess.run",
            return_value=subprocess.CompletedProcess(
                [],
                0,
                "123 /wrong/build/px4_sitl_sih/bin/px4\n",
                "",
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "conflicting simulators"):
                backend.hold()

    def test_dashboard_keeps_authenticated_restart_available_when_flight_node_is_absent(self):
        reset = SimpleNamespace(
            report=lambda: dict(busy=False, phase="FAILED"),
            start=lambda snap, report: starts.append(snap),
        )
        starts = []
        hub = Dashboard(SimpleNamespace(get=lambda: (None, None)), Queue(), port=0)
        hub.reset = reset
        hub.flight_available = False
        try:

            def request(action, token):
                return Request(
                    hub.url + "/api/command",
                    json.dumps(dict(action=action)).encode(),
                    headers={"Content-Type": "application/json", "X-Navigation-Token": token},
                )

            with self.assertRaises(HTTPError) as denied:
                urlopen(request("restart", "wrong"))
            self.assertEqual(denied.exception.code, 403)
            denied.exception.read()
            self.assertEqual(json.load(urlopen(request("restart", hub.token)))["queued"], "restart")
            with self.assertRaises(HTTPError):
                urlopen(request("mission", hub.token))
            self.assertEqual(starts, [None])
            self.assertFalse(hub.state()["control_enabled"])
            self.assertTrue(hub.commands.empty())
        finally:
            hub.close()


if __name__ == "__main__":
    unittest.main()
