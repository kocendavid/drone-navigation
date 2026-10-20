"""Diagnostic evidence and request acknowledgments, isolated from flight traffic."""

import json
from pathlib import Path
from queue import Queue
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from navigation.dashboard import Dashboard
from navigation.history import RunHistory, CommandReceipts
from navigation.mapping import RayMap, FREE, OCCUPIED
from tools.recover_navigation_history import recover
from navigation.flight_controls import FlightControls


class HistorySnapshot(SimpleNamespace):
    def fresh(self, now):
        return True


def snapshot():
    model = RayMap(resolution=0.2, size=20, origin=(-2.0, -2.0))
    model.grid[:] = FREE
    clearance, body_free, safe = model.geometry()
    return HistorySnapshot(
        model=model,
        pose=[0.0, 0.0, 0.0, 1.0],
        scans=3,
        stamp=100,
        reason="Observed map",
        valid=True,
        clearance=clearance,
        body_free=body_free,
        safe=safe,
    )


def report(goal=None, phase="NAVIGATING", reason="Following route", simulation_s=10.0):
    return dict(
        goal=goal,
        phase=phase,
        reason=reason,
        delivered_simulation_s=simulation_s,
        measured_world_xy=[9.0, 9.0],
        path=[[0.0, 0.0], [5.0, 5.0]],
        exploration=dict(selection_reason="Useful unknown boundary"),
    )


class HistoryTests(unittest.TestCase):
    def test_process_writer_keeps_a_recording_error_after_later_success(self):
        from navigation.report_jobs import HistoryProcessor

        with tempfile.TemporaryDirectory() as folder:
            writer = HistoryProcessor(Path(folder) / "history.jsonl", "test")
            with patch.object(writer.history, "_write", side_effect=OSError("disk unavailable")):
                with self.assertRaisesRegex(OSError, "disk unavailable"):
                    writer(("event", "state", "failed write", {}))
            summary, _ = writer(("event", "state", "later write", {}))
            self.assertEqual(summary["error"], "disk unavailable")

    def test_explicit_flight_button_dispatches_once_and_keeps_command_receipt(self):
        hub = Dashboard(SimpleNamespace(get=lambda: (None, None)), Queue(), port=0)
        dispatched = []
        hub.report["flight"] = dict(updated_monotonic_s=time.monotonic(), can_takeoff=True)
        backend = SimpleNamespace(
            execute=lambda action, allowed: dispatched.append(action) or allowed()
        )
        hub.flight_controls = FlightControls(lambda: hub.report, hub.acknowledge_command, backend)
        try:
            payload = json.dumps(dict(action="takeoff", request_id="explicit-click")).encode()
            for _ in range(2):
                response = json.load(
                    urlopen(
                        Request(
                            hub.url + "/api/command",
                            payload,
                            headers={
                                "Content-Type": "application/json",
                                "X-Navigation-Token": hub.token,
                            },
                        )
                    )
                )
                self.assertIn(response["command"]["status"], ("accepted", "running"))
            self.assertEqual(dispatched, ["takeoff"])
            self.assertTrue(hub.commands.empty())
        finally:
            hub.close()

    def test_session_expiry_and_foreign_origin_are_rejected_before_queueing(self):
        hub = Dashboard(SimpleNamespace(get=lambda: (None, None)), Queue(), port=0)
        try:
            old_token = hub.token
            hub.token = "new-test-session"
            payload = json.dumps(dict(action="explore", request_id="same-intent")).encode()

            def post(token, origin):
                return urlopen(
                    Request(
                        hub.url + "/api/command",
                        payload,
                        headers={
                            "Content-Type": "application/json",
                            "X-Navigation-Token": token,
                            "Origin": origin,
                        },
                    )
                )

            for token, origin, code in (
                (old_token, hub.url, "session_expired"),
                (hub.token, "http://foreign.invalid", "origin_mismatch"),
            ):
                with self.assertRaises(HTTPError) as failure:
                    post(token, origin)
                self.assertEqual(failure.exception.code, 403)
                self.assertEqual(json.load(failure.exception)["code"], code)
                self.assertTrue(hub.commands.empty())
                self.assertFalse(hub.receipts.report())
            self.assertEqual(json.load(post(hub.token, hub.url))["command"]["status"], "queued")
            self.assertEqual(hub.commands.qsize(), 1)
        finally:
            hub.close()

    def test_goal_request_does_not_wait_for_route_planning_in_http_handler(self):
        hub = Dashboard(SimpleNamespace(get=lambda: (None, None)), Queue(), port=0)
        try:
            with patch.object(hub, "plan_destination", side_effect=AssertionError("Blocking A*")):
                request = Request(
                    hub.url + "/api/command",
                    json.dumps(dict(action="goal", point=[1.0, 2.0])).encode(),
                    headers={"Content-Type": "application/json", "X-Navigation-Token": hub.token},
                )
                response = json.load(urlopen(request))
            self.assertEqual(response["command"]["status"], "queued")
            self.assertEqual(hub.commands.get_nowait()["point"], [1.0, 2.0])
        finally:
            hub.close()

    def test_keeps_abandoned_targets_and_measured_travel_separate_from_planned_route(self):
        history = RunHistory()
        snap = snapshot()
        history.observe(report([1.0, 0.0]), snap, 100.0)
        first = history.report()["frame_ids"][0]
        snap.pose[0] = 0.3
        history.observe(report([-1.0, 0.0]), snap, 100.0)
        data = history.report()
        self.assertEqual([p["status"] for p in data["targets"]], ["superseded", "selected"])
        self.assertEqual(data["trail"], [[0.0, 0.0], [0.3, 0.0]])
        self.assertNotIn([5.0, 5.0], data["trail"])
        self.assertEqual(history.frame(first)["pose"][0], 0.0)

    def test_blocked_event_preserves_the_map_buffer_and_cell_revision_evidence(self):
        history = RunHistory()
        snap = snapshot()
        history.observe(report([1.0, 0.0]), snap, 100.0)
        snap.model.grid[3, 3] = OCCUPIED
        snap.model.confirmed_walls[3, 3] = True
        cell = snap.model.cell(snap.pose[:2])
        snap.safe[cell] = False
        snap.body_free[cell] = True
        snap.clearance[cell] = 0.55
        history.observe(
            report([1.0, 0.0], "BLOCKED", "Route changed; waiting to replan"), snap, 100.0
        )
        event = history.report()["events"][-1]
        frame = history.frame(event["frame_id"])
        self.assertEqual(event["kind"], "blocked")
        self.assertTrue(frame["provisional"])
        self.assertEqual(frame["cell_changes"]["free_to_occupied"], 1)
        self.assertEqual(frame["wall_policy"]["confirmed_cells"], 1)
        self.assertEqual(frame["confirmed_walls"][3 * 20 + 3], "1")
        self.assertTrue(frame["footprint"]["body_observed"])
        self.assertFalse(frame["footprint"]["preferred_margin_met"])
        self.assertAlmostEqual(frame["footprint"]["wall_clearance_m"], 0.55)

    def test_evicted_map_frames_remain_retrievable_from_the_complete_disk_log(self):
        with tempfile.TemporaryDirectory() as folder:
            history = RunHistory(Path(folder) / "decisions.jsonl")
            try:
                snap = snapshot()
                for i in range(90):
                    history.observe(report([1.0, 0.0], reason="Decision " + str(i)), snap, 100.0)
                self.assertTrue(history.flush())
                self.assertIsNone(history.error)
                ids = history.report()["frame_ids"]
                self.assertEqual(len(ids), 90)
                self.assertEqual(len(history.report()["frames"]), 80)
                self.assertEqual(history.frame(ids[0])["reason"], "Decision 0")
                rows = [json.loads(line) for line in history.path.read_text().splitlines()]
                self.assertEqual(sum(row["type"] == "frame" for row in rows), 90)
            finally:
                history.close()

    def test_duplicate_request_is_not_a_second_command_and_expired_request_stays_expired(self):
        receipts = CommandReceipts()
        with patch("navigation.history.time.monotonic", return_value=100.0):
            first, new = receipts.begin(dict(action="explore", request_id="same"))
            self.assertTrue(new)
            receipts.update(first["id"], "queued", "Awaiting controller")
            _, new = receipts.begin(dict(action="explore", request_id="same"))
            self.assertFalse(new)
            with self.assertRaises(ValueError):
                receipts.begin(dict(action="restart", request_id="same"))
        with patch("navigation.history.time.monotonic", return_value=106.0):
            self.assertEqual(receipts.report()[0]["status"], "expired")
            self.assertEqual(receipts.update("same", "accepted", "Too late")["status"], "expired")

    def test_http_receipt_distinguishes_queueing_from_controller_rejection(self):
        hub = Dashboard(SimpleNamespace(get=lambda: (None, None)), Queue(), port=0)
        try:

            def post():
                req = Request(
                    hub.url + "/api/command",
                    json.dumps(dict(action="explore", request_id="test-request")).encode(),
                    headers={"Content-Type": "application/json", "X-Navigation-Token": hub.token},
                )
                return json.load(urlopen(req))

            response = post()
            self.assertEqual(response["command"]["status"], "queued")
            self.assertEqual(hub.commands.qsize(), 1)
            post()
            self.assertEqual(hub.commands.qsize(), 1)
            hub.acknowledge_command("test-request", "rejected", "Position unavailable")
            receipt = json.load(urlopen(hub.url + "/api/command-status?id=test-request"))
            self.assertEqual(receipt["command"]["status"], "rejected")
            self.assertEqual(hub.commands.qsize(), 1)
            data = json.load(urlopen(hub.url + "/api/state"))
            self.assertEqual(data["commands"][-1]["status"], "rejected")
            self.assertEqual(data["commands"][-1]["reason"], "Position unavailable")
        finally:
            hub.close()

    def test_return_home_http_is_queued_once_without_a_client_supplied_destination(self):
        hub = Dashboard(SimpleNamespace(get=lambda: (None, None)), Queue(), port=0)
        try:
            for _ in range(2):
                req = Request(
                    hub.url + "/api/command",
                    json.dumps(dict(action="home", request_id="return-home")).encode(),
                    headers={"Content-Type": "application/json", "X-Navigation-Token": hub.token},
                )
                self.assertEqual(json.load(urlopen(req))["command"]["status"], "queued")
            self.assertEqual(hub.commands.qsize(), 1)
            self.assertEqual(hub.commands.get_nowait()["action"], "home")
        finally:
            hub.close()

    def test_home_route_preview_is_read_only_and_uses_recorded_start(self):
        snap = snapshot()
        hub = Dashboard(SimpleNamespace(get=lambda: (snap, None)), Queue(), port=0)
        try:
            hub.report["home_xy"] = [0.5, 0.0]
            response = json.load(urlopen(hub.url + "/api/home-route"))
            self.assertEqual(response["goal"], [0.5, 0.0])
            self.assertEqual(response["path"][-1], [0.5, 0.0])
            self.assertGreaterEqual(response["route_length_m"], response["straight_line_m"])
            self.assertEqual(response["run_id"], hub.history.run_id)
            self.assertTrue(hub.commands.empty())
            hub.report["home_xy"] = None
            with self.assertRaises(HTTPError) as caught:
                urlopen(hub.url + "/api/home-route")
            self.assertIn("not captured", json.load(caught.exception)["error"])
            hub.report["home_xy"] = [0.5, 0.0]
            snap.fresh = lambda now: False
            with self.assertRaises(HTTPError):
                urlopen(hub.url + "/api/home-route")
            self.assertTrue(hub.commands.empty())
        finally:
            hub.close()

    def test_frame_and_complete_download_are_read_only_and_previous_run_survives(self):
        with tempfile.TemporaryDirectory() as folder:
            history = RunHistory(Path(folder) / "decisions.jsonl")
            history.observe(report([1.0, 0.0]), snapshot(), 100.0)
            history.flush()
            hub = Dashboard(SimpleNamespace(get=lambda: (None, None)), Queue(), port=0)
            hub.previous_history = history
            try:
                frame_id = history.report()["frame_ids"][0]
                frame = json.load(
                    urlopen(hub.url + "/api/frame?id=" + str(frame_id) + "&previous=1")
                )
                self.assertEqual(frame["goal"], [1.0, 0.0])
                download = urlopen(hub.url + "/api/history/download?previous=1").read()
                self.assertIn(b'"type":"frame"', download)
                self.assertTrue(hub.commands.empty())
                with self.assertRaises(HTTPError):
                    urlopen(hub.url + "/api/frame?id=../../secrets")
            finally:
                hub.close()
                history.close()

    def test_legacy_recovery_separates_controller_runs_and_does_not_invent_maps(self):
        with tempfile.TemporaryDirectory() as folder:
            trace = Path(folder) / "trace.jsonl"
            rows = [
                dict(report([8.0, 0.0], simulation_s=50), measured_world_xy=[8.0, 0.0]),
                dict(report([1.0, 0.0], simulation_s=0), measured_world_xy=[0.0, 0.0]),
                dict(report([2.0, 0.0], simulation_s=1), measured_world_xy=[0.2, 0.0]),
            ]
            trace.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")
            result = recover(trace)
            self.assertFalse(result["map_frames_available"])
            self.assertNotIn("grid", result)
            self.assertEqual(result["trail"], [[0.0, 0.0], [0.2, 0.0]])
            self.assertEqual([t["xy"] for t in result["targets"]], [[1.0, 0.0], [2.0, 0.0]])
            self.assertEqual(result["targets"][0]["status"], "superseded")


if __name__ == "__main__":
    unittest.main()
