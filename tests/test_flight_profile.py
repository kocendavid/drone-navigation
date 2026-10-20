"""One-second measurements use source time and preserve missing/reset evidence."""

import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from navigation.flight_profile import FlightProfile


def records(end=5.2, session="bridge", airborne_start=False):
    rows = []
    for i in range(round(end * 25) + 1):
        t = i / 25
        height = min(max(t - 1, 0), 1)
        armed = airborne_start or t >= 1
        rows.extend(
            [
                dict(
                    kind="isaac_truth",
                    session=session,
                    isaac_sim_ns=round(t * 1e9),
                    px4_sim_us=round((t + 10) * 1e6),
                    position_world_z=-3.94 + height,
                    velocity_world=[t / 10, 0, 0.1],
                    armed=armed,
                ),
                dict(
                    kind="px4_position",
                    session="probe",
                    px4_pid="42",
                    fields=dict(
                        timestamp_sample=round((t + 10) * 1e6),
                        z=-height,
                        z_valid=True,
                        vx=t / 10 + 0.1,
                        vy=0,
                        vz=-0.2,
                        v_xy_valid=True,
                        v_z_valid=True,
                        z_reset_counter=1,
                        vxy_reset_counter=0,
                        vz_reset_counter=0,
                    ),
                ),
                dict(
                    kind="px4_setpoint",
                    session="probe",
                    px4_pid="42",
                    fields=dict(timestamp=round((t + 10) * 1e6), position=[0, 0, -1]),
                ),
                dict(
                    kind="reference",
                    session="nav",
                    sim_ns=round(t * 1e9),
                    phase="NAVIGATING",
                    speed_decision=dict(
                        expected_speed_m_s=0.8,
                        selected_speed_m_s=1.0,
                        requested_cruise_m_s=1.2,
                        dominant_limit="tracking",
                    ),
                ),
            ]
        )
    return rows


class FlightProfileTests(unittest.TestCase):
    def profile(self, rows, **options):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        profile = FlightProfile(temporary.name, -4, **options)
        for row in rows:
            profile.observe(row)
        profile.finish()
        with profile.files[-1].open(newline="", encoding="utf-8") as stream:
            output = list(csv.DictReader(stream))
        return profile, output

    def test_exact_simulation_seconds_common_height_datum_and_horizontal_speeds(self):
        _, rows = self.profile(records())
        self.assertEqual([float(r["simulation_s"]) for r in rows], list(range(6)))
        row = rows[3]
        self.assertAlmostEqual(float(row["simulation_height_m"]), 1.06)
        self.assertAlmostEqual(float(row["estimated_height_m"]), 1.06)
        self.assertAlmostEqual(float(row["commanded_height_m"]), 1.06)
        self.assertAlmostEqual(float(row["px4_local_height_m"]), 1.0)
        self.assertAlmostEqual(float(row["actual_speed_m_s"]), 0.3)
        self.assertAlmostEqual(float(row["estimated_speed_m_s"]), 0.4)
        self.assertAlmostEqual(float(row["expected_speed_m_s"]), 0.8)
        self.assertAlmostEqual(float(row["actual_vertical_speed_m_s"]), 0.1)
        self.assertAlmostEqual(float(row["estimated_vertical_speed_m_s"]), 0.2)

    def test_arrival_delay_does_not_shift_source_aligned_samples(self):
        rows = records()
        delayed = []
        pending = []
        for row in rows:
            if row["kind"] == "isaac_truth":
                delayed.append(row)
                if len(pending) > 12:
                    delayed.extend(pending[:4])
                    pending = pending[4:]
            else:
                pending.append(row)
        delayed.extend(pending)
        _, expected = self.profile(rows)
        _, actual = self.profile(delayed)
        for index in (2, 3, 4):
            for key in (
                "simulation_height_m",
                "estimated_height_m",
                "actual_speed_m_s",
                "expected_speed_m_s",
            ):
                self.assertEqual(expected[index][key], actual[index][key])

    def test_wall_timestamps_follow_capture_clock_through_slow_simulation_and_pauses(self):
        rows = records()
        epoch = 1_792_486_800.0
        for row in rows:
            if row["kind"] == "isaac_truth":
                t = row["isaac_sim_ns"] / 1e9
                row["emitted_wall_s"] = 500 + 2 * t + (10 if t >= 3 else 0)
        _, output = self.profile(rows, wall_clock_offset_s=epoch - 500)
        self.assertAlmostEqual(float(output[2]["timestamp_unix_s"]), epoch + 4)
        self.assertAlmostEqual(float(output[3]["timestamp_unix_s"]), epoch + 16)
        _, missing = self.profile(records())
        self.assertEqual(missing[3]["timestamp_unix_s"], "")

    def test_recorded_unix_clock_pair_corrects_transport_delay_and_beats_fallback_anchor(self):
        rows = records()
        for row in rows:
            if row["kind"] == "isaac_truth":
                t = row["isaac_sim_ns"] / 1e9
                row.update(
                    emitted_wall_s=500 + 2 * t,
                    received_wall_s=500.4 + 2 * t,
                    received_unix_s=1_792_486_800.4 + 2 * t,
                )
        _, output = self.profile(rows, wall_clock_offset_s=0)
        self.assertAlmostEqual(float(output[3]["timestamp_unix_s"]), 1_792_486_806)

    def test_clock_anchor_is_saved_for_replay_and_not_guessed_for_unrelated_recordings(self):
        from tools.record_flight_profile import recording_clock_offset

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "logs").mkdir()
            source = root / "events.jsonl"
            status = root / "logs" / "timing_diagnostics.json"
            status.write_text(
                json.dumps(
                    dict(recording=True, log_file=str(source), updated_wall_s=time.perf_counter())
                )
            )
            with patch("tools.record_flight_profile.PROJECT", root):
                offset = recording_clock_offset(source)
                self.assertAlmostEqual(offset, time.time() - time.perf_counter(), places=2)
                self.assertTrue((root / "wall_clock.json").exists())
                with patch("time.perf_counter", return_value=1):
                    self.assertEqual(recording_clock_offset(source), offset)
                (root / "wall_clock.json").unlink()
                status.unlink()
                self.assertIsNone(recording_clock_offset(source))

    def test_gaps_are_missing_and_simulation_pause_cannot_create_wall_time_samples(self):
        rows = [
            r
            for r in records()
            if r["kind"] != "px4_position" or r["fields"]["timestamp_sample"] < 12_500_000
        ]
        _, result = self.profile(rows)
        self.assertEqual(result[3]["estimated_speed_m_s"], "")
        self.assertEqual(result[3]["estimated_height_m"], "")
        self.assertEqual(result[3]["estimate_available"], "False")
        _, repeated = self.profile(records() + [records()[-4]] * 100)
        self.assertEqual(len(repeated), 6)

    def test_new_bridge_session_separates_runs_and_requires_disarmed_origin(self):
        profile, rows = self.profile(records() + records(session="newbridge", airborne_start=True))
        self.assertEqual(len(profile.files), 2)
        self.assertTrue(rows[3]["px4_local_height_m"])
        self.assertEqual(rows[3]["estimated_height_m"], "")
        self.assertEqual(rows[3]["commanded_height_m"], "")

    def test_native_height_reset_invalidates_calibration_without_mixing_epochs(self):
        rows = records()
        for row in rows:
            if row["kind"] == "px4_position" and row["fields"]["timestamp_sample"] >= 13_000_000:
                row["fields"]["z_reset_counter"] = 2
        _, output = self.profile(rows)
        self.assertEqual(output[4]["estimated_height_m"], "")
        self.assertTrue(output[4]["px4_local_height_m"])

    def test_latest_run_seek_retains_its_ground_calibration_records(self):
        from tools.record_flight_profile import latest_run_offset

        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "events.jsonl"
            old = records(session="old")
            new = records(session="new")
            path.write_bytes(b"".join((json.dumps(r) + "\n").encode() for r in old + new))
            with path.open("rb") as stream:
                stream.seek(latest_run_offset(path))
                first = json.loads(stream.readline())
            self.assertEqual(first["session"], "new")
            self.assertFalse(first["armed"])

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "Graph rendering needs matplotlib")
    def test_replay_cli_writes_pngs_and_metadata(self):
        from tools.record_flight_profile import plot_profile

        profile, _ = self.profile(records())
        self.assertTrue(profile.files[-1].with_suffix(".json").exists())
        for graph in plot_profile(profile.files[-1]):
            self.assertTrue(Path(graph).read_bytes().startswith(b"\x89PNG"))

    def test_live_follower_publishes_captured_samples_without_waiting_for_png(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "events.jsonl"
            output = Path(folder) / "profiles"
            rows = records(end=8.2)
            split = len(records(end=5.2))
            source.write_bytes(b"".join((json.dumps(r) + "\n").encode() for r in rows[:split]))
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(Path(__file__).parents[1] / "tools" / "record_flight_profile.py"),
                    str(source),
                    "--output",
                    str(output),
                    "--floor-z",
                    "-4",
                    "--follow",
                    "--ui-port",
                    "0",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            try:

                def wait_for_sample(second):
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline:
                        if process.poll() is not None:
                            self.fail(process.stderr.read().decode())
                        try:
                            data = json.loads((output / "status.json").read_text())
                            if (data.get("latest") or {}).get("simulation_s", -1) >= second:
                                return data
                        except (OSError, ValueError):
                            pass
                        time.sleep(0.02)
                    self.fail(f"Captured sample {second} was not published within three seconds")

                first = wait_for_sample(3)
                began = time.monotonic()
                with source.open("ab") as stream:
                    stream.write(b"".join((json.dumps(r) + "\n").encode() for r in rows[split:]))
                second = wait_for_sample(6)
                self.assertLess(time.monotonic() - began, 2)
                self.assertGreater(second["samples"], first["samples"])
                self.assertEqual(second["graphs"], [])
                self.assertFalse(list(output.glob("*.png")))
            finally:
                process.terminate()
                process.wait(timeout=5)
                process.stderr.close()

    def test_busy_windows_status_file_is_retried_without_stopping_capture(self):
        from tools.record_flight_profile import publish_status

        profile, _ = self.profile(records())
        original_replace = Path.replace
        attempts = []

        def busy_once(path, target):
            attempts.append(path)
            if len(attempts) == 1:
                raise PermissionError("Viewer is reading the status file")
            return original_replace(path, target)

        with patch.object(Path, "replace", busy_once), patch("time.sleep"):
            self.assertTrue(publish_status(profile, "events.jsonl", 123))
        status = json.loads((profile.folder / "status.json").read_text())
        self.assertEqual(status["samples"], profile.samples)
        self.assertEqual(len(attempts), 2)
        with patch.object(Path, "replace", side_effect=PermissionError), patch("time.sleep"):
            self.assertFalse(publish_status(profile, "events.jsonl", 456))


if __name__ == "__main__":
    unittest.main()
