"""Bounded trajectory and operator gates; no ROS context or flight publishers."""

import math
from pathlib import Path
from queue import Queue
import sys
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from px4_msgs.msg import VehicleStatus
from ros2_hover import HoverState
from ros2_step import SimulationPacer, StepPlan, StepPublisher, step_duration


def position(**changes):
    return SimpleNamespace(
        **{
            **dict(
                timestamp=1000000,
                x=2.0,
                y=3.0,
                z=-1.0,
                heading=0.0,
                xy_valid=True,
                z_valid=True,
                v_xy_valid=True,
                v_z_valid=True,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                xy_reset_counter=0,
                z_reset_counter=0,
                heading_reset_counter=0,
            ),
            **changes,
        }
    )


def node_fixture(**changes):
    state = HoverState()
    p = position()
    state.observe(p, 100.0)
    pacer = SimulationPacer()
    pacer.total_s = 100.0
    pacer.last_progress = 100.0
    modes, targets, notices = [], [], []
    node = SimpleNamespace(
        state=state,
        position=p,
        plan=StepPlan(state.anchor, 0.5),
        distance=0.5,
        height=1.0,
        ready_samples=50,
        status=SimpleNamespace(
            arming_state=VehicleStatus.ARMING_STATE_ARMED,
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            failsafe=False,
        ),
        status_received=100.0,
        commands=Queue(),
        last_report=100.0,
        settled_since=None,
        pacer=pacer,
        last_callback_wall=None,
        clearance_feed=None,
        clearance_decision=None,
        count_publishers=lambda _: 1,
        say=notices.append,
        mode_pub=SimpleNamespace(publish=modes.append, get_subscription_count=lambda: 1),
        target_pub=SimpleNamespace(publish=targets.append, get_subscription_count=lambda: 1),
        **changes,
    )
    for method in (
        "flight_active",
        "stable_hover",
        "cancel_step",
        "handle_commands",
        "clearance_check",
    ):
        setattr(node, method, MethodType(getattr(StepPublisher, method), node))
    return node, modes, targets, notices


class StepTests(unittest.TestCase):
    def test_simulation_pacer_uses_periods_not_timestamp_offset_or_packet_gaps(self):
        pacer = SimulationPacer()
        for stamp in (1000000, 1004000, 9000000):
            pacer.observe(SimpleNamespace(timestamp=stamp, gyro_integral_dt=4000), 100.0)
        self.assertAlmostEqual(pacer.total_s, 0.008)
        self.assertTrue(pacer.fresh(100.1))

    def test_duplicate_sensor_packets_do_not_advance_or_refresh_pacer(self):
        pacer = SimulationPacer()
        for stamp, wall in ((1000000, 100.0), (1004000, 100.1), (1004000, 101.2)):
            pacer.observe(SimpleNamespace(timestamp=stamp, gyro_integral_dt=4000), wall)
        self.assertAlmostEqual(pacer.total_s, 0.004)
        self.assertFalse(pacer.fresh(101.2))

    def test_clock_correction_discards_uncertain_periods_without_catch_up(self):
        pacer = SimulationPacer()
        for stamp in (1000000, 1004000, 900000, 904000, 908000, 912000):
            pacer.observe(SimpleNamespace(timestamp=stamp, gyro_integral_dt=4000), 100.0)
        self.assertAlmostEqual(pacer.total_s, 0.004)
        pacer.observe(SimpleNamespace(timestamp=916000, gyro_integral_dt=4000), 100.1)
        self.assertAlmostEqual(pacer.total_s, 0.008)
        self.assertTrue(pacer.fresh(100.1))

    def test_invalid_simulation_period_cannot_advance_movement(self):
        for stamp, period in ((0, 4000), (1004000, 0), (1004000, 1000000)):
            pacer = SimulationPacer()
            pacer.last_progress = 100.0
            pacer.observe(SimpleNamespace(timestamp=stamp, gyro_integral_dt=period), 100.0)
            self.assertEqual(pacer.total_s, 0.0)
            self.assertFalse(pacer.fresh(100.0))

    def test_forward_follows_captured_heading_in_ned(self):
        for yaw, expected in ((0, (2.5, 3)), (math.pi / 2, (2, 3.5)), (math.pi, (1.5, 3))):
            plan = StepPlan((2, 3, yaw), 0.5)
            for actual, wanted in zip(plan.endpoint, expected):
                self.assertAlmostEqual(actual, wanted)

    def test_reference_speed_acceleration_and_endpoints(self):
        plan = StepPlan((0, 0, 0), 0.5)
        self.assertEqual(plan.advance(100), (0, 0))  # No motion before operator command.
        plan.start(0)
        dt = plan.duration / 1000
        samples = [0.0]
        for i in range(1, 1002):
            samples.append(plan.advance(i * dt)[0])
        speeds = [(b - a) / dt for a, b in zip(samples, samples[1:])]
        accelerations = [(b - a) / dt for a, b in zip(speeds, speeds[1:])]
        self.assertLessEqual(max(speeds), 0.200001)
        self.assertLessEqual(max(map(abs, accelerations)), 0.200001)
        self.assertTrue(all(0 <= v <= 0.5 for v in samples))
        self.assertEqual(plan.target, (0.5, 0))
        self.assertEqual(plan.phase, "TARGET_COMPLETE")

    def test_late_callback_freezes_instead_of_jumping_forward(self):
        plan = StepPlan((0, 0, 0), 0.5)
        plan.start(10)
        target = plan.advance(10.1)
        self.assertEqual(plan.advance(11), target)
        self.assertEqual(plan.phase, "HOLD")
        self.assertFalse(plan.start(12))

    def test_hold_cannot_restart_or_accumulate_steps(self):
        plan = StepPlan((0, 0, 0), 0.5)
        plan.start(10)
        plan.hold((0.1, 0.02))
        self.assertEqual(plan.advance(100), (0.1, 0.02))
        self.assertFalse(plan.start(100))

    def test_actual_displacement_resolves_forward_and_sideways(self):
        plan = StepPlan((2, 3, math.pi / 2), 0.5)
        forward, lateral = plan.displacement(position(x=1.9, y=3.5))
        self.assertAlmostEqual(forward, 0.5)
        self.assertAlmostEqual(lateral, 0.1)

    def test_rejects_unbounded_or_nonfinite_requests(self):
        for distance in (0, -1, 1.01, math.nan, math.inf):
            with self.assertRaises(ValueError):
                StepPlan((0, 0, 0), distance)
        with self.assertRaises(ValueError):
            step_duration(0.5, speed=0)

    def test_move_requires_armed_offboard_fresh_status_and_stable_hover(self):
        for failure in (
            "disarmed",
            "not_offboard",
            "stale_status",
            "wrong_height",
            "moving",
            "not_ready",
            "yaw",
        ):
            with self.subTest(failure=failure):
                node, _, _, _ = node_fixture()
                if failure == "disarmed":
                    node.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
                if failure == "not_offboard":
                    node.status.nav_state = 4
                if failure == "stale_status":
                    node.status_received = 97
                if failure == "wrong_height":
                    node.position.z = 0
                if failure == "moving":
                    node.position.vx = 0.3
                if failure == "not_ready":
                    node.ready_samples = 10
                if failure == "yaw":
                    node.position.heading = math.pi / 2
                node.commands.put("move")
                node.handle_commands(100.0)
                self.assertEqual(node.plan.phase, "HOVER")
                self.assertFalse(node.plan.used)

    def test_operator_trigger_publishes_heading_trajectory_and_px4_timestamp(self):
        node, modes, targets, _ = node_fixture()
        node.commands.put("move")
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "MOVING")
        self.assertEqual(list(targets[-1].position), [2.0, 3.0, -1.0])
        node.pacer.total_s = 100.1
        with patch("ros2_step.time.monotonic", return_value=100.1):
            StepPublisher.publish_target(node)
        self.assertGreater(targets[-1].position[0], 2)
        self.assertEqual(targets[-1].position[1], 3)
        self.assertEqual(targets[-1].position[2], -1)
        self.assertEqual(targets[-1].timestamp, 0)
        self.assertTrue(modes[-1].position)
        self.assertTrue(all(math.isnan(v) for v in targets[-1].velocity))

    def test_short_clock_correction_holds_reference_without_losing_heartbeat(self):
        node, modes, targets, _ = node_fixture()
        node.plan.start(99.9)
        node.state.observe(position(timestamp=900000), 100.01)
        elapsed = node.plan.elapsed
        with patch("ros2_step.time.monotonic", return_value=100.1):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "MOVING")
        self.assertEqual(node.plan.elapsed, elapsed)
        self.assertEqual(list(targets[-1].position), [2, 3, -1])
        self.assertEqual(modes[-1].timestamp, 0)
        node.state.observe(position(timestamp=910000), 100.15)
        node.state.observe(position(timestamp=920000), 100.2)
        node.state.observe(position(timestamp=930000), 100.25)
        node.pacer.total_s = 100.2
        with patch("ros2_step.time.monotonic", return_value=100.3):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "MOVING")
        self.assertAlmostEqual(node.plan.elapsed, 0.2)
        self.assertGreater(targets[-1].position[0], 2)

    def test_slow_simulation_advances_reference_by_physics_time_only(self):
        node, _, targets, _ = node_fixture()
        node.commands.put("move")
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        for i in range(1, 11):
            wall = 100.0 + i * 0.1
            node.state.last_progress = node.status_received = node.pacer.last_progress = wall
            node.last_report = wall
            node.pacer.total_s = 100.0 + i * 0.03  # Physics runs at 0.3 x real time.
            with patch("ros2_step.time.monotonic", return_value=wall):
                StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "MOVING")
        self.assertAlmostEqual(node.plan.elapsed, 0.3)
        expected = StepPlan(node.state.anchor, 0.5)
        expected.start(0.0)
        self.assertAlmostEqual(targets[-1].position[0], expected.advance(0.3)[0])

    def test_missing_sensor_periods_freeze_reference_but_keep_heartbeat(self):
        node, modes, targets, _ = node_fixture()
        node.plan.start(99.9)
        node.pacer.last_progress = 98.0
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "MOVING")
        self.assertEqual(node.plan.elapsed, 0.0)
        self.assertEqual(modes[-1].timestamp, 0)
        self.assertEqual(list(targets[-1].position), [2, 3, -1])
        node.pacer.total_s = 100.004
        node.pacer.last_progress = 100.1
        with patch("ros2_step.time.monotonic", return_value=100.1):
            StepPublisher.publish_target(node)
        self.assertAlmostEqual(node.plan.elapsed, 0.004)

    def test_move_requires_progressing_simulation_periods(self):
        node, _, _, notices = node_fixture()
        node.pacer.last_progress = None
        node.commands.put("move")
        node.handle_commands(100.0)
        self.assertEqual(node.plan.phase, "HOVER")
        self.assertIn("simulated IMU periods required", notices[-1])

    def test_wall_callback_stall_holds_even_if_simulation_time_advanced_little(self):
        node, _, targets, _ = node_fixture()
        node.plan.start(100.0)
        node.pacer.total_s = 100.01
        node.last_callback_wall = 99.0
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "HOLD")
        self.assertEqual(list(targets[-1].position), [2, 3, -1])

    def test_move_during_clock_confirmation_is_refused_with_specific_reason(self):
        node, _, _, notices = node_fixture()
        node.state.observe(position(timestamp=900000), 100.01)
        node.commands.put("move")
        node.handle_commands(100.1)
        self.assertEqual(node.plan.phase, "HOVER")
        self.assertIn("confirming samples: 3", notices[-1])

    def test_stale_position_aborts_step_and_publishes_nothing(self):
        node, modes, targets, _ = node_fixture()
        node.plan.start(99)
        node.state.last_progress = 98
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "HOLD")
        self.assertEqual(modes, [])
        self.assertEqual(targets, [])

    def test_competing_target_publisher_prevents_any_output(self):
        node, modes, targets, _ = node_fixture()
        node.count_publishers = lambda _: 2
        node.commands.put("move")
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "HOVER")
        self.assertEqual(modes, [])
        self.assertEqual(targets, [])

    def test_tracking_error_stops_at_current_xy(self):
        node, _, targets, _ = node_fixture()
        node.plan.start(99.9)
        node.position.x = 2.4
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "HOLD")
        self.assertEqual(list(targets[-1].position), [2.4, 3.0, -1.0])

    def test_actual_arrival_requires_settling_and_armed_offboard(self):
        node, _, _, notices = node_fixture()
        node.plan.start(95)
        node.plan.target = node.plan.endpoint
        node.plan.phase = "TARGET_COMPLETE"
        node.position.x = 2.5
        with patch("ros2_step.time.monotonic", return_value=100.0):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "TARGET_COMPLETE")
        node.state.last_progress = node.status_received = node.last_report = 101.1
        node.last_callback_wall = 101.0
        with patch("ros2_step.time.monotonic", return_value=101.1):
            StepPublisher.publish_target(node)
        self.assertEqual(node.plan.phase, "REACHED")
        self.assertTrue(any("REACHED" in n for n in notices))


if __name__ == "__main__":
    unittest.main()
