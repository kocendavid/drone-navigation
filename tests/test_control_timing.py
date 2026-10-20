"""Timings attribute CPU and waiting time, including unpublished callbacks."""

import unittest
from unittest.mock import patch
from types import SimpleNamespace

from navigation.control_timing import ControlTiming
from navigation.diagnostics import analyze


class ControlTimingTests(unittest.TestCase):
    def test_late_callback_and_slow_native_publication_are_deadline_misses(self):
        timing = ControlTiming()
        timing.previous_command_published = True
        timing.callback_gap_s = 0.15
        timing.submission = dict(reason="accepted", completion_age_s=0.002)
        self.assertEqual(timing.report()["deadline_cause"], "callback_arrival_late")
        timing.callback_gap_s = 0.05
        timing.submission["completion_age_s"] = 0.12
        self.assertEqual(timing.report()["deadline_cause"], "native_publication_overrun")
        timing.previous_command_published = False
        timing.submission = None
        self.assertFalse(timing.report()["deadline_missed"])

    def test_stage_wall_and_cpu_are_separate_and_reports_do_not_mutate(self):
        with (
            patch(
                "navigation.control_timing.time.perf_counter_ns",
                side_effect=[0, 120_000_000, 125_000_000, 130_000_000, 130_000_000],
            ),
            patch(
                "navigation.control_timing.time.thread_time_ns",
                side_effect=[0, 5_000_000, 6_000_000, 8_000_000, 8_000_000],
            ),
        ):
            timing = ControlTiming()
            timing.mark("route_validation")
            report = timing.report()
            timing.mark("publish")
            final = timing.report()
        self.assertEqual(report["stages"]["route_validation"], dict(wall_s=0.12, cpu_s=0.005))
        self.assertNotIn("publish", report["stages"])
        self.assertAlmostEqual(final["off_cpu_s"], 0.122)
        self.assertEqual(final["slowest_stage"], "route_validation")

    def test_early_return_still_emits_a_control_timing_record(self):
        from ros2_navigate import GoalNavigator
        from test_navigation import navigator_fixture

        node, _, targets, _ = navigator_fixture()
        node.trajectory_clock.report = lambda now: dict(available=True)
        rows = []
        node.diagnostics = SimpleNamespace(
            emit=lambda kind, **fields: rows.append(dict(kind=kind, **fields))
        )
        node.state.last_progress = None
        with patch("ros2_navigate.time.monotonic", return_value=100):
            GoalNavigator.publish_target(node)
        self.assertFalse(targets)
        record = next(row for row in rows if row["kind"] == "control_timing")
        self.assertEqual(record["outcome"], "no_command")
        self.assertIsNone(record["submission"])

    def test_incident_analysis_includes_rejected_callback_and_stage_statistics(self):
        rows = [
            dict(
                kind="control_timing",
                received_wall_s=1,
                deadline_missed=True,
                stages=dict(motion=dict(wall_s=0.12, cpu_s=0.03)),
            ),
            dict(
                kind="control_rejection",
                received_wall_s=1,
                submission=dict(reason="stale_command", command_age_s=0.12),
            ),
        ]
        summary = analyze(rows)
        self.assertEqual(summary["timing_stages"]["control_motion_wall_s"]["maximum"], 0.12)
        self.assertEqual(summary["timing_stages"]["control_motion_cpu_s"]["maximum"], 0.03)
        self.assertEqual(len(summary["deadline_misses"]), 1)
        self.assertEqual(summary["control_rejections"][0]["submission"]["reason"], "stale_command")
