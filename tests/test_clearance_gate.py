"""Live-data freshness, conservative decisions and flight-stop wiring."""

from copy import deepcopy
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1]))
from lidar_tools.flight_gate import ClearanceGate, GateDecision
from lidar_tools.clearance_feed import ClearanceFeed
from ros2_step import StepPublisher
from test_step import node_fixture


def record(stamp, wall, distance=4.0):
    return dict(
        session="test",
        generated_monotonic_s=wall,
        report=dict(
            state="HOLD" if distance <= 2.0 else "CLEAR",
            reason="Closest observed surface",
            body_frame="chassis",
            world_frame="world",
            floor_ready=True,
            cloud_age_wall_s=0.1,
            clock_age_wall_s=0.02,
            stamp_ns=stamp,
            forward_distance_m=distance,
            nearest_body_xyz_m=[distance, 0.1, 0.2],
            corridor_points=30,
            limits=dict(
                hold_m=2.0,
                min_range_m=1.0,
                max_range_m=30.0,
                forward_min_m=0.2,
                forward_max_m=15.0,
                half_width_m=0.5,
                body_z_min_m=-0.3,
                body_z_max_m=0.5,
                floor_margin_m=0.12,
                floor_z_world_m=-0.76965,
            ),
        ),
    )


def confirmed_gate():
    gate = ClearanceGate("test")
    for i in range(3):
        gate.observe(record(1000000000 + i * 100000000, 100.0 + i * 0.2), 100.0 + i * 0.2)
    return gate


class GateTests(unittest.TestCase):
    def test_three_distinct_fresh_scans_required_before_allowing_move(self):
        gate = ClearanceGate("test")
        for i in range(3):
            gate.observe(record(1000000000 + i * 100000000, 100.0 + i * 0.2), 100.0 + i * 0.2)
            self.assertEqual(gate.decision(100.0 + i * 0.2).allowed, i == 2)

    def test_buffered_or_future_records_never_allow_move(self):
        for wall in (99.0, 101.0):
            gate = confirmed_gate()
            gate.observe(record(1300000000, wall), 100.5)
            self.assertFalse(gate.decision(100.5).allowed)

    def test_regular_worker_messages_cannot_mask_frozen_cloud_stamp(self):
        gate = confirmed_gate()
        for i in range(1, 7):
            wall = 100.4 + i * 0.2
            gate.observe(record(1200000000, wall), wall)
        self.assertFalse(gate.decision(101.6).allowed)

    def test_measurement_age_includes_pipe_and_controller_delay(self):
        gate = confirmed_gate()
        item = record(1300000000, 100.5)
        item["report"]["cloud_age_wall_s"] = 0.8
        gate.observe(item, 100.5)
        self.assertFalse(gate.decision(100.8).allowed)

    def test_stopped_worker_records_expire(self):
        self.assertFalse(confirmed_gate().decision(101.3).allowed)

    def test_stop_reserve_applies_before_the_red_two_metre_zone(self):
        for distance in (2.2, 2.1, 2.0, 1.4):
            gate = confirmed_gate()
            gate.observe(record(1300000000, 100.5, distance), 100.5)
            decision = gate.decision(100.5)
            self.assertFalse(decision.allowed)
            self.assertEqual(decision.state, "HOLD")

    def test_unknown_discards_confirmation_and_requires_new_scans(self):
        gate = confirmed_gate()
        item = record(1300000000, 100.5)
        item["report"].update(state="UNKNOWN", reason="Capture-time TF missing")
        gate.observe(item, 100.5)
        self.assertIn("TF missing", gate.decision(100.5).reason)
        gate.observe(record(1400000000, 100.6), 100.6)
        self.assertFalse(gate.decision(100.6).allowed)

    def test_clock_reset_requires_reconfirmation_without_using_old_clear(self):
        gate = confirmed_gate()
        gate.observe(record(100000000, 100.5), 100.5)
        self.assertFalse(gate.decision(100.5).allowed)
        for i in range(3):
            gate.observe(record(200000000 + i * 100000000, 100.6 + i * 0.1), 100.6 + i * 0.1)
        self.assertTrue(gate.decision(100.8).allowed)

    def test_wrong_session_frame_floor_settings_or_points_are_rejected(self):
        changes = [
            lambda m: m.update(session="old"),
            lambda m: m["report"].update(body_frame="base_link"),
            lambda m: m["report"].update(floor_ready=False),
            lambda m: m["report"]["limits"].update(half_width_m=0.1),
            lambda m: m["report"].update(forward_distance_m=float("nan")),
            lambda m: m["report"].update(nearest_body_xyz_m=[4.0, 5.0, 0.2]),
            lambda m: m["report"].update(state="CLEAR", forward_distance_m=1.5),
            lambda m: m["report"].update(stamp_ns=None),
        ]
        for change in changes:
            gate = confirmed_gate()
            item = deepcopy(record(1300000000, 100.5))
            change(item)
            gate.observe(item, 100.5)
            self.assertFalse(gate.decision(100.5).allowed)

    def test_long_scan_gap_requires_three_new_scans(self):
        gate = confirmed_gate()
        gate.observe(record(1300000000, 102.0), 102.0)
        self.assertFalse(gate.decision(102.0).allowed)

    def test_worker_exit_overrides_cached_clear(self):
        feed = ClearanceFeed.__new__(ClearanceFeed)
        feed.lock = threading.Lock()
        feed.failure = None
        feed.process = SimpleNamespace(poll=lambda: 1)
        feed.gate = confirmed_gate()
        self.assertFalse(feed.decision(100.5).allowed)
        feed.failure = "LiDAR worker output closed"
        feed.process.poll = lambda: None
        self.assertFalse(feed.decision(100.5).allowed)


class FlightGateTests(unittest.TestCase):
    def test_refused_move_remains_unused_and_can_be_requested_after_clear(self):
        node, _, _, notices = node_fixture()
        node.clearance_feed = SimpleNamespace(
            decision=lambda now: GateDecision(False, "HOLD", "Box at 1.8 m", 1.8)
        )
        node.commands.put("move")
        node.handle_commands(100.0)
        self.assertFalse(node.plan.used)
        self.assertIn("refused by LiDAR", notices[-1])
        node.clearance_feed.decision = lambda now: GateDecision(
            True, "CLEAR", "Three fresh scans", 4.0
        )
        node.commands.put("move")
        node.handle_commands(100.0)
        self.assertEqual(node.plan.phase, "MOVING")

    def test_close_or_unknown_lidar_stops_motion_without_stopping_heartbeat(self):
        for state in ("HOLD", "UNKNOWN"):
            node, modes, targets, notices = node_fixture()
            node.plan.start(99.9)
            node.position.x, node.position.y = 2.1, 3.02
            node.clearance_feed = SimpleNamespace(
                decision=lambda now: GateDecision(False, state, "Blocked observation")
            )
            with patch("ros2_step.time.monotonic", return_value=100.0):
                StepPublisher.publish_target(node)
            self.assertEqual(node.plan.phase, "HOLD")
            self.assertEqual(list(targets[-1].position), [2.1, 3.02, -1.0])
            self.assertEqual(modes[-1].timestamp, 0)
            self.assertIn("STEP STOPPED: LiDAR", notices[-1])
            node.clearance_feed.decision = lambda now: GateDecision(
                True, "CLEAR", "Obstacle moved", 4.0
            )
            node.pacer.total_s = 100.1
            with patch("ros2_step.time.monotonic", return_value=100.1):
                StepPublisher.publish_target(node)
            self.assertEqual(node.plan.phase, "HOLD")  # No automatic restart.

    def test_lidar_can_stop_final_settling_before_reached(self):
        node, _, _, _ = node_fixture()
        node.plan.start(99.0)
        node.plan.phase = "TARGET_COMPLETE"
        node.position.x = 2.5
        node.plan.target = node.plan.endpoint
        node.clearance_feed = SimpleNamespace(
            decision=lambda now: GateDecision(False, "UNKNOWN", "Cloud stopped")
        )
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "HOLD")

    def test_heading_change_stops_forward_sensing_experiment(self):
        node, _, _, notices = node_fixture()
        node.plan.start(99.9)
        node.position.heading = 0.25
        node.clearance_feed = SimpleNamespace(
            decision=lambda now: GateDecision(True, "CLEAR", "Fresh", 4.0)
        )
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "HOLD")
        self.assertIn("heading changed", notices[-1])


if __name__ == "__main__":
    unittest.main()
