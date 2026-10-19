"""Independent source-time evidence, gaps and pause attribution; no flight."""

import copy
import json
from pathlib import Path
import socket
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from urllib.request import urlopen
from urllib.error import HTTPError
from navigation.diagnostic_wire import DiagnosticEmitter
from navigation.diagnostics import (
    analyze,
    interpolate,
    parse_listener,
    read_status,
    incident_window,
)
from navigation.timing import TrajectoryClock


def recording():
    alignment = dict(rotation=[[1, 0], [0, 1]], offset=[0, 0])
    rows = []
    for i in (0, 1):
        rows.append(
            dict(
                kind="isaac_truth",
                session="bridge",
                received_wall_s=1 + i,
                px4_sim_us=11000000 + i * 100000,
                isaac_sim_ns=10000000000 + i * 100000000,
                position_world_xy=[5 + 0.3 * i, 0],
            )
        )
        rows.append(
            dict(
                kind="reference",
                session="navigator",
                received_wall_s=3 + i,
                emitted_wall_s=2 + i,
                sim_ns=10000000000 + i * 100000000,
                world_xy=[5.1 + 0.3 * i, 0],
                phase="NAVIGATING",
                alignment=alignment,
            )
        )
    rows.append(
        dict(
            kind="px4_position",
            session="probe",
            received_wall_s=8,
            px4_pid="42",
            fields=dict(
                timestamp=11060000,
                timestamp_sample=11050000,
                xy_valid=True,
                x=0,
                y=5.13,
                xy_reset_counter=0,
                heading_reset_counter=0,
            ),
        )
    )
    for stamp, east in ((11040000, 5.24), (11090000, 20)):
        rows.append(
            dict(
                kind="px4_setpoint",
                session="probe",
                received_wall_s=8,
                px4_pid="42",
                fields=dict(timestamp=stamp, position=[0, east, -1]),
            )
        )
    return rows


class DiagnosticTests(unittest.TestCase):
    def test_busy_windows_status_file_does_not_abort_recording_or_truncate_previous_status(self):
        from tools.record_timing import atomic_json

        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "status.json"
            path.write_text('{"old": true}', encoding="utf-8")
            with patch.object(Path, "replace", side_effect=PermissionError("Windows reader")):
                self.assertFalse(atomic_json(path, dict(recording=True), busy_ok=True))
                self.assertEqual(json.loads(path.read_text()), dict(old=True))
                with self.assertRaises(PermissionError):
                    atomic_json(path, dict(incident="must be saved"))
            self.assertTrue(atomic_json(path, dict(recording=True), busy_ok=True))
            self.assertEqual(json.loads(path.read_text()), dict(recording=True))

    def test_read_only_download_returns_recording_and_rejects_files_outside_logs(self):
        from navigation.dashboard import Dashboard

        root = Path(__file__).parents[1]
        with tempfile.TemporaryDirectory(dir=root / "logs") as folder:
            path = Path(folder) / "events.jsonl"
            path.write_text('{"kind":"test"}\n', encoding="utf-8")
            hub = Dashboard(SimpleNamespace(get=lambda: (None, "No map")), port=0)
            try:
                with patch(
                    "navigation.dashboard.read_status", return_value=dict(log_file=str(path))
                ):
                    with urlopen(hub.url + "/api/timing/download?recording=1") as response:
                        self.assertEqual(response.read(), path.read_bytes())
                with patch(
                    "navigation.dashboard.read_status",
                    return_value=dict(log_file=str(root / "README.md")),
                ):
                    with self.assertRaises(HTTPError) as error:
                        urlopen(hub.url + "/api/timing/download?recording=1")
                    self.assertEqual(error.exception.code, 404)
            finally:
                hub.close()

    def test_pause_window_keeps_its_trigger_even_when_another_pause_arrives(self):
        rows = recording()
        first = dict(kind="stop", received_wall_s=10, reason="first")
        second = dict(kind="stop", received_wall_s=11, reason="second")
        rows.extend([first, second, dict(kind="late", received_wall_s=12.1)])
        window = incident_window(rows, first)
        self.assertNotIn(rows[-1], window)
        self.assertEqual(analyze(window, event=first)["stop"]["reason"], "first")

    def test_captured_alignment_counters_must_match_native_position_epoch(self):
        rows = recording()
        for row in rows:
            if row["kind"] == "reference":
                row["alignment"]["reset_counters"] = [2, 0]
        report = analyze(rows)
        self.assertEqual(report["estimation_error_m"]["samples"], 0)
        self.assertEqual(report["unavailable"]["frame_reset"], 1)

    def test_source_time_interpolation_separates_estimation_tracking_and_received_target(self):
        report = analyze(recording())
        self.assertAlmostEqual(report["estimation_error_m"]["maximum"], 0.02)
        self.assertAlmostEqual(report["intended_tracking_error_m"]["maximum"], 0.1)
        self.assertAlmostEqual(report["received_setpoint_error_m"]["maximum"], 0.09)
        self.assertAlmostEqual(report["controller_position_error_m"]["maximum"], 0.11)
        detail = report["aligned_samples"][0]
        self.assertEqual(detail["px4_sample_us"], 11050000)
        self.assertEqual(detail["isaac_sim_ns"], 10050000000)

    def test_delayed_arrival_cannot_change_same_source_time_errors(self):
        rows = recording()
        expected = analyze(rows)
        for r in rows:
            r["received_wall_s"] += 20
        actual = analyze(rows)
        self.assertEqual(actual["estimation_error_m"], expected["estimation_error_m"])
        self.assertEqual(actual["intended_tracking_error_m"], expected["intended_tracking_error_m"])

    def test_missing_bracket_and_invalid_estimate_are_missing_evidence_not_zero_error(self):
        for scenario in ("invalid", "missing_truth"):
            rows = recording()
            row = next(r for r in rows if r["kind"] == "px4_position")
            if scenario == "invalid":
                row["fields"]["xy_valid"] = False
            else:
                row["fields"]["timestamp_sample"] = 12000000
            report = analyze(rows)
            self.assertEqual(report["estimation_error_m"]["samples"], 0)
            self.assertIsNone(report["estimation_error_m"]["maximum"])
            self.assertEqual(
                report["unavailable"][
                    "invalid_position" if scenario == "invalid" else "missing_truth"
                ],
                1,
            )

    def test_interpolation_refuses_large_gaps_extrapolation_and_phase_boundaries(self):
        rows = [dict(t=0, v=[0, 0], segment="a"), dict(t=1, v=[1, 1], segment="a")]
        self.assertIsNone(interpolate(rows, 0.5, "t", "v", 0.2))
        self.assertIsNone(interpolate(rows, -0.1, "t", "v", 2))
        rows[1]["segment"] = "b"
        self.assertIsNone(interpolate(rows, 0.5, "t", "v", 2))

    def test_reset_counters_and_process_sessions_cannot_be_silently_joined(self):
        rows = recording()
        extra = copy.deepcopy(next(r for r in rows if r["kind"] == "px4_position"))
        extra["fields"]["xy_reset_counter"] = 1
        rows.append(extra)
        report = analyze(rows)
        self.assertEqual(report["unavailable"]["frame_reset"], 1)
        self.assertEqual(report["estimation_error_m"]["samples"], 1)
        extra["px4_pid"] = "43"
        report = analyze(rows)
        self.assertEqual(report["estimation_error_m"]["samples"], 1)

    def test_native_parser_retains_sample_time_validity_and_unavailable_nan(self):
        text = "TOPIC: trajectory_setpoint\n    timestamp: 12345 (0.01 seconds ago)\n    timestamp_sample: 12340 (5 us before timestamp)\n    xy_valid: False\n    position: [1.2, nan, -1.0]\n    y: -0.1234\n"
        fields = parse_listener(text)
        self.assertEqual(fields["timestamp_sample"], 12340)
        self.assertFalse(fields["xy_valid"])
        self.assertEqual(fields["position"], [1.2, None, -1.0])
        json.dumps(fields, allow_nan=False)

    def test_clock_report_distinguishes_stale_backward_and_large_unconsumed_interval(self):
        clock = TrajectoryClock()
        for i in range(5):
            clock.observe(i * 50000000, 1 + i * 0.05)
        clock.consume(1.2)
        self.assertIsNone(clock.consume(1.5))
        self.assertEqual(clock.report(1.5)["consume_reason"], "clock_update_stale")
        decision = clock.report(1.5)["decision"].copy()
        clock.observe(250000000, 1.55)
        self.assertTrue(clock.report(1.55)["available"])
        self.assertEqual(clock.report(1.55)["decision"], decision)
        clock.observe(100000000, 1.6)
        self.assertEqual(clock.report(1.6)["last_discontinuity"], "clock_moved_backward")
        clock = TrajectoryClock()
        for i in range(20):
            clock.observe(i * 50000000, 1 + i * 0.05)
        self.assertIsNone(clock.consume(1.951))
        self.assertEqual(
            clock.report(1.951)["consume_reason"], "unconsumed_interval_exceeded_0.25_s"
        )

    def test_nonblocking_emission_and_stale_recorder_status(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        emitter = DiagnosticEmitter("test", receiver.getsockname()[1])
        try:
            with (
                patch("navigation.diagnostic_wire.time.monotonic", return_value=12),
                patch("navigation.diagnostic_wire.time.perf_counter", return_value=1234),
            ):
                emitter.emit("test", value=1)
            record = json.loads(receiver.recv(16000))
            self.assertEqual(record["value"], 1)
            self.assertEqual(record["emitted_wall_s"], 1234)
            emitter.close()
            emitter.emit("test", value=2)  # Closed/failed diagnostics cannot stop control.
            self.assertEqual(emitter.dropped, 1)
        finally:
            receiver.close()
        with tempfile.TemporaryDirectory() as folder:
            logs = Path(folder) / "logs"
            logs.mkdir()
            (logs / "timing_diagnostics.json").write_text(
                json.dumps(dict(recording=True, updated_wall_s=time.perf_counter() - 5)),
                encoding="utf-8",
            )
            self.assertFalse(read_status(folder)["recording"])
            (logs / "timing_diagnostics.json").write_text(
                json.dumps(dict(recording=True, updated_wall_s=time.perf_counter() + 0.01)),
                encoding="utf-8",
            )
            self.assertTrue(read_status(folder)["recording"])
