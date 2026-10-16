"""Explicit flight-button intent and cancellation, using fake simulator outputs only."""

import threading
import time
import unittest
from unittest.mock import patch

from navigation.flight_controls import FlightControls, NativeFlightBackend
from navigation.reset import ResetBackend, PX4


class FakeBackend:
    def __init__(self):
        self.calls = []
        self.entered = threading.Event()
        self.release = threading.Event()
        self.release.set()
        self.finished = threading.Event()

    def execute(self, action, still_allowed):
        self.entered.set()
        self.release.wait(2)
        try:
            if still_allowed():
                self.calls.append(action)
                return True
            return False
        finally:
            self.finished.set()


def fixture():
    report = dict(
        flight=dict(
            updated_monotonic_s=time.monotonic(),
            can_takeoff=True,
            can_enable_navigation=False,
            can_land=False,
            can_hold=True,
        )
    )
    feedback = []
    backend = FakeBackend()
    flight = FlightControls(lambda: report, lambda *args: feedback.append(args), backend)
    return flight, backend, report, feedback


def wait_phase(flight, phase):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if flight.report()["phase"] == phase:
            return
        time.sleep(0.005)
    raise AssertionError(flight.report())


class FlightButtonTests(unittest.TestCase):
    def test_slow_prechecks_cannot_dispatch_an_old_takeoff_click(self):
        flight, backend, report, feedback = fixture()
        with patch("navigation.flight_controls.threading.Thread"):
            flight.start("takeoff", "slow-click")
        later = time.monotonic() + 6
        report["flight"]["updated_monotonic_s"] = later
        with patch("navigation.flight_controls.time.monotonic", return_value=later):
            flight.run(flight.generation, "takeoff", "slow-click")
        self.assertEqual(backend.calls, [])
        self.assertEqual(feedback[-1][1], "failed")
        self.assertIn("expired", feedback[-1][2])

    def test_reset_gate_is_rechecked_after_native_prechecks(self):
        flight, backend, _, feedback = fixture()
        with patch("navigation.flight_controls.threading.Thread"):
            flight.start("takeoff", "reset-race")
        flight.enabled = lambda: False
        flight.run(flight.generation, "takeoff", "reset-race")
        self.assertEqual(backend.calls, [])
        self.assertEqual(feedback[-1][1], "failed")

    def test_startup_does_not_arm_and_takeoff_requires_observed_hover_to_complete(self):
        flight, backend, report, feedback = fixture()
        self.assertEqual(backend.calls, [])
        flight.start("takeoff", "takeoff-click")
        wait_phase(flight, "OBSERVING")
        self.assertEqual(backend.calls, ["takeoff"])
        flight.observe(report)
        self.assertTrue(flight.report()["busy"])
        self.assertNotEqual(feedback[-1][1], "completed")
        report["flight"]["can_enable_navigation"] = True
        flight.observe(report)
        self.assertEqual(feedback[-1][1], "completed")
        self.assertFalse(flight.report()["busy"])

    def test_stale_report_or_wrong_phase_refuses_before_dispatch(self):
        flight, backend, report, _ = fixture()
        with self.assertRaises(ValueError):
            flight.start("offboard", "wrong-phase")
        report["flight"]["updated_monotonic_s"] -= 3
        with self.assertRaisesRegex(ValueError, "fresh flight telemetry"):
            flight.start("takeoff", "stale")
        self.assertEqual(backend.calls, [])

    def test_hold_or_reset_cancellation_prevents_late_takeoff(self):
        flight, backend, _, feedback = fixture()
        backend.release.clear()
        flight.start("takeoff", "pending-takeoff")
        self.assertTrue(backend.entered.wait(2))
        flight.cancel("Experiment reset requested")
        backend.release.set()
        self.assertTrue(backend.finished.wait(2))
        self.assertEqual(backend.calls, [])
        self.assertEqual(feedback[-1][1], "cancelled")

    def test_offboard_and_landing_wait_for_their_actual_flight_states(self):
        flight, _, report, feedback = fixture()
        report["flight"]["can_enable_navigation"] = True
        flight.start("offboard", "enable")
        wait_phase(flight, "OBSERVING")
        flight.observe(report)
        self.assertTrue(flight.report()["busy"])
        report["ready"] = True
        flight.observe(report)
        self.assertEqual(feedback[-1][1], "completed")
        report["flight"]["can_land"] = True
        flight.start("land", "land")
        wait_phase(flight, "OBSERVING")
        flight.observe(report)
        self.assertTrue(flight.report()["busy"])
        report["flight"]["landed_disarmed"] = True
        flight.observe(report)
        self.assertEqual(feedback[-1][1], "completed")

    def test_unconfirmed_native_action_times_out_without_another_command(self):
        flight, backend, report, feedback = fixture()
        flight.start("takeoff", "takeoff")
        wait_phase(flight, "OBSERVING")
        with patch("navigation.flight_controls.time.monotonic", return_value=time.monotonic() + 61):
            flight.observe(report)
        self.assertEqual(feedback[-1][1], "failed")
        self.assertEqual(backend.calls, ["takeoff"])

    def test_native_backend_rejects_wrong_height_and_sih_without_takeoff(self):
        for height in ("5.0", "nan"):
            backend = NativeFlightBackend()
            backend.command = lambda args, timeout=8: "123 " + PX4 if args[-1] == "px4" else height
            commands = []
            backend.px4 = lambda *args: commands.append(args) or "INFO  [commander] Disarmed\n"
            with self.assertRaises(ValueError):
                backend.execute("takeoff", lambda: True)
            self.assertEqual(commands, [("status",)])
        backend = NativeFlightBackend()
        backend.command = lambda *args, **kwargs: "123 /wrong/px4_sitl_sih/bin/px4"
        backend.px4 = lambda *args: self.fail("Must not command SIH")
        with self.assertRaises(RuntimeError):
            backend.execute("takeoff", lambda: True)

    def test_reset_hold_serializes_after_an_already_dispatched_native_command(self):
        backend = NativeFlightBackend()
        backend.verify_simulator = lambda: None
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        calls = []

        def send(*args):
            calls.append("mode_started")
            entered.set()
            release.wait(2)
            calls.append("mode_finished")

        backend.px4 = send
        with patch.object(ResetBackend, "hold", lambda self: calls.append("reset_hold")):
            worker = threading.Thread(target=backend.execute, args=("offboard", lambda: True))
            worker.start()
            self.assertTrue(entered.wait(2))
            hold = threading.Thread(target=lambda: (backend.hold(), finished.set()))
            hold.start()
            self.assertFalse(finished.wait(0.03))
            release.set()
            worker.join(2)
            hold.join(2)
        self.assertEqual(calls, ["mode_started", "mode_finished", "reset_hold"])


if __name__ == "__main__":
    unittest.main()
