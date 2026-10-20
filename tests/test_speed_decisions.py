"""Speed decisions distinguish ceiling reductions, commands and measurements."""

import json
import math
from pathlib import Path
import tempfile
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from navigation.history import RunHistory
from navigation.motion import RouteFollower
from navigation.settings import load_settings
from ros2_navigate import GoalNavigator
from test_history import snapshot as history_snapshot, report as history_report
from test_navigation import navigator_fixture


class SpeedDecisionTests(unittest.TestCase):
    def follower(self, speed=0.5):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [100, 0]])
        follower.speed = speed
        follower.velocity = np.array([speed, 0.0])
        return follower

    def tick(self, follower, pose=(0, 0), current=0.4, **kwargs):
        follower.tick(
            kwargs.pop("dt", 0.02),
            pose,
            np.empty((0, 2)),
            observed_corridor=True,
            measured_velocity=[current, 0],
            braking_safe=kwargs.pop("braking_safe", lambda _: True),
            **kwargs,
        )
        json.dumps(follower.speed_decision, allow_nan=False)
        return follower.speed_decision

    def test_yaw_ceiling_records_real_acceleration_limited_deceleration(self):
        follower = self.follower(1.2)
        decision = self.tick(follower, current=1.1, heading_error=math.pi)
        self.assertEqual(decision["requested_cruise_m_s"], 1.2)
        self.assertEqual(decision["caps_m_s"]["yaw"], 0.7)
        self.assertEqual(decision["selected_speed_m_s"], 0.7)
        self.assertAlmostEqual(decision["expected_speed_m_s"], 1.195)
        self.assertEqual(decision["current_speed_m_s"], 1.1)
        self.assertEqual(decision["dominant_limit"], "yaw")
        self.assertEqual(decision["action"], "decelerating")
        self.assertTrue(decision["ceiling_binding"])
        self.assertTrue(decision["acceleration_limited"])

    def test_lag_restriction_can_lower_ceiling_while_command_still_accelerates(self):
        follower = self.follower(0.48)
        follower.tracking_scale = 0.53
        decision = self.tick(follower, pose=[-0.17, 0], current=0.47, heading_error=0)
        self.assertAlmostEqual(decision["along_lead_m"], 0.17)
        self.assertLess(decision["caps_m_s"]["tracking"], 1.2)
        self.assertEqual(decision["dominant_limit"], "tracking_response")
        self.assertAlmostEqual(decision["expected_speed_m_s"], 0.485)
        self.assertEqual(decision["current_speed_m_s"], 0.47)
        self.assertEqual(decision["action"], "accelerating")
        self.assertFalse(decision["ceiling_binding"])

    def test_growing_lag_reduces_command_using_measured_speed_and_predicted_lead(self):
        follower = self.follower(0.479)
        # Just before the recorded 22:38:25 stop: cruise scaling left room to
        # accelerate despite ~12 cm of lead and a slower physical vehicle.
        decision = self.tick(follower, dt=0.032, pose=[-0.119, 0], current=0.331)
        self.assertGreater(decision["predicted_along_lead_m"], decision["along_lead_m"])
        self.assertLess(decision["caps_m_s"]["tracking_response"], 0.479)
        self.assertEqual(decision["dominant_limit"], "tracking_response")
        self.assertEqual(decision["action"], "decelerating")
        self.assertAlmostEqual(decision["expected_speed_m_s"], 0.479 - 0.25 * 0.032)
        closing = decision["allowed_closing_speed_m_s"]
        self.assertAlmostEqual(
            closing * 0.5 + closing**2 / (2 * 0.25), decision["tracking_lead_budget_m"]
        )
        self.assertIsNone(follower.braking_envelope)

    def test_steady_bounded_lag_at_cruise_does_not_force_slowdown(self):
        follower = self.follower(1.2)
        decision = self.tick(follower, pose=[-0.12, 0], current=1.2)
        self.assertEqual(decision["selected_speed_m_s"], 1.2)
        self.assertEqual(decision["expected_speed_m_s"], 1.2)
        self.assertEqual(decision["action"], "steady")
        self.assertEqual(follower.reason, "Following planned route")
        self.assertIsNone(follower.braking_envelope)

    def test_tracking_turn_and_stopping_space_braking_record_distinct_causes(self):
        for cause in ("tracking_error", "travel_direction_change", "stopping_space"):
            follower = self.follower(1)
            if cause == "travel_direction_change":
                follower.retarget([[0, 0], [-100, 0]])
            decision = self.tick(
                follower,
                pose=[-0.3, 0] if cause == "tracking_error" else [0, 0],
                current=1,
                braking_safe=(
                    (lambda p: p[0, 0] <= 0) if cause == "stopping_space" else (lambda _: True)
                ),
            )
            self.assertEqual(decision["braking_cause"], cause)
            self.assertEqual(decision["selected_speed_m_s"], 0)
            self.assertAlmostEqual(decision["expected_speed_m_s"], 0.995)
            self.assertEqual(decision["action"], "decelerating")

    def test_zero_clock_and_stop_are_finite_and_preserve_earlier_snapshots(self):
        follower = self.follower()
        decision = self.tick(follower, dt=0, current=0.4)
        encoded = json.dumps(decision)
        self.assertEqual(decision["action"], "waiting_for_clock")
        self.assertEqual(decision["expected_speed_m_s"], 0.5)
        follower.hold([0, 0], "Operator hold")
        self.assertEqual(json.dumps(decision), encoded)
        stopped = follower.speed_report(0.4, 0)
        self.assertEqual(stopped["expected_speed_m_s"], 0)
        self.assertEqual(stopped["current_speed_m_s"], 0.4)
        self.assertEqual(stopped["published_feedforward_speed_m_s"], 0)
        self.assertEqual(stopped["dominant_limit"], "stop")
        json.dumps(stopped, allow_nan=False)

    def test_missing_measurement_is_not_replaced_with_reference_speed(self):
        follower = self.follower()
        follower.tick(0.02, [0, 0], np.empty((0, 2)), observed_corridor=True)
        self.assertIsNone(follower.speed_decision["current_speed_m_s"])

    def test_every_navigation_sample_is_saved_without_extra_event_rows_or_map_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            history = RunHistory(Path(folder) / "decisions.jsonl")
            try:
                snap = history_snapshot()
                for index in range(3):
                    report = history_report([1, 0], simulation_s=10 + index * 0.05)
                    report["speed_decision"] = dict(
                        expected_speed_m_s=0.5, current_speed_m_s=index * 0.1
                    )
                    history.observe(report, snap, 100)
                self.assertTrue(history.flush())
                rows = [json.loads(line) for line in history.path.read_text().splitlines()]
                speeds = [r for r in rows if r["type"] == "speed_decision"]
                self.assertEqual(len(speeds), 3)
                self.assertAlmostEqual(speeds[-1]["speed_decision"]["current_speed_m_s"], 0.2)
                self.assertEqual(sum(r["type"] == "frame" for r in rows), 1)
                states = [e for e in history.report()["events"] if e["kind"] == "state"]
                self.assertEqual(len(states), 1)
                self.assertEqual(states[0]["speed_decision"]["current_speed_m_s"], 0)
            finally:
                history.close()

    def test_navigator_exports_decision_to_diagnostics_and_dashboard(self):
        node, _, _, _ = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.alignment_counters = [0, 0]
        node.trajectory_clock.stamp = 1_000_000_000
        node.trajectory_clock.report = lambda _: dict(available=True)
        diagnostics, reports = [], []
        node.diagnostics = SimpleNamespace(
            emit=lambda kind, **fields: diagnostics.append(dict(kind=kind, **fields))
        )
        node.update_dashboard = MethodType(GoalNavigator.update_dashboard, node)
        node.dashboard.observe = lambda report, *_: reports.append(report)
        node.pacer.total_s += 0.05
        with patch("ros2_navigate.time.monotonic", return_value=100):
            GoalNavigator.publish_target(node)
        decision = reports[-1]["speed_decision"]
        reference = next(r for r in diagnostics if r["kind"] == "reference")
        self.assertEqual(reference["speed_decision"], decision)
        self.assertEqual(decision["current_speed_m_s"], 0)
        self.assertGreater(decision["expected_speed_m_s"], 0)
        self.assertAlmostEqual(
            decision["expected_speed_m_s"], decision["published_feedforward_speed_m_s"]
        )
        self.assertLess(len(json.dumps(reference).encode()), 16_000)


if __name__ == "__main__":
    unittest.main()
