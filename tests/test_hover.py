"""Flight-output construction and clock/reset handling; no ROS publishers."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_hover import HoverPublisher, HoverState, hover_messages


def position(**changes):
    return SimpleNamespace(
        **{
            **dict(
                timestamp=1_000_000,
                x=2.0,
                y=3.0,
                z=-1.0,
                heading=0.2,
                xy_valid=True,
                z_valid=True,
                xy_reset_counter=0,
                z_reset_counter=0,
                heading_reset_counter=0,
            ),
            **changes,
        }
    )


class HoverTests(unittest.TestCase):
    def test_ned_target_and_inactive_feedforward_fields(self):
        mode, target = hover_messages(1_000_000, 2, 3, 1, 0.2)
        self.assertEqual(list(target.position), [2, 3, -1])
        self.assertEqual(mode.timestamp, target.timestamp)
        self.assertTrue(mode.position)
        self.assertFalse(
            any(
                (
                    mode.velocity,
                    mode.acceleration,
                    mode.attitude,
                    mode.body_rate,
                    mode.thrust_and_torque,
                    mode.direct_actuator,
                )
            )
        )
        self.assertTrue(
            all(
                math.isnan(v)
                for values in (target.velocity, target.acceleration, target.jerk)
                for v in values
            )
        )

    def test_duplicates_do_not_refresh_stale_state(self):
        state = HoverState()
        self.assertTrue(state.observe(position(), 10))
        self.assertFalse(state.observe(position(), 10.9))
        self.assertTrue(state.fresh(10.9))
        self.assertFalse(state.fresh(11.01))

    def test_invalid_state_pauses_and_valid_progress_recovers_original_anchor(self):
        state = HoverState()
        state.observe(position(), 10)
        state.observe(position(timestamp=1_100_000, xy_valid=False), 10.1)
        self.assertFalse(state.fresh(10.1))
        state.observe(position(timestamp=1_200_000, x=8, heading=0.4), 10.2)
        self.assertTrue(state.fresh(10.2))
        self.assertEqual(state.anchor, (2, 3, 0.2))

    def test_every_coordinate_reset_latches_a_fault(self):
        for counter in ("xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
            with self.subTest(counter=counter):
                state = HoverState()
                state.observe(position(), 10)
                state.observe(position(timestamp=1_100_000, **{counter: 1}), 10.1)
                self.assertIsNotNone(state.fault)
                self.assertFalse(state.fresh(10.1))
                self.assertFalse(state.observe(position(timestamp=1_200_000), 10.2))

    def test_backward_mapped_clock_recovers_after_three_valid_advances(self):
        state = HoverState()
        state.observe(position(), 10)
        state.observe(position(timestamp=900_000), 10.1)
        self.assertIsNone(state.fault)
        self.assertTrue(state.fresh(10.1))
        self.assertFalse(state.motion_ready(10.1))
        self.assertEqual(state.clock_rebases, 1)
        state.observe(position(timestamp=910_000), 10.2)
        state.observe(position(timestamp=920_000), 10.3)
        self.assertTrue(state.fresh(10.3))
        self.assertFalse(state.motion_ready(10.3))
        state.observe(position(timestamp=930_000), 10.4)
        self.assertTrue(state.fresh(10.4))
        self.assertTrue(state.motion_ready(10.4))
        self.assertEqual(state.anchor, (2, 3, 0.2))

    def test_duplicate_and_oscillating_old_packets_cannot_complete_clock_recovery(self):
        state = HoverState()
        state.observe(position(), 10)
        for i, stamp in enumerate((900_000, 900_000, 910_000, 900_000, 910_000, 900_000)):
            state.observe(position(timestamp=stamp), 10.1 + i * 0.1)
            self.assertFalse(state.motion_ready(10.1 + i * 0.1))
        self.assertFalse(state.fresh(11.01))

    def test_real_origin_reset_during_clock_recovery_still_latches_fault(self):
        state = HoverState()
        state.observe(position(), 10)
        state.observe(position(timestamp=900_000), 10.1)
        state.observe(position(timestamp=910_000, z_reset_counter=1), 10.2)
        self.assertIsNotNone(state.fault)
        self.assertFalse(state.fresh(10.2))

    def test_invalid_position_restarts_three_sample_clock_recovery(self):
        state = HoverState()
        state.observe(position(), 10)
        state.observe(position(timestamp=900_000), 10.1)
        state.observe(position(timestamp=910_000), 10.2)
        state.observe(position(timestamp=920_000, xy_valid=False), 10.3)
        for i, stamp in enumerate((930_000, 940_000)):
            state.observe(position(timestamp=stamp), 10.4 + i * 0.1)
            self.assertFalse(state.fresh(10.4 + i * 0.1))
        state.observe(position(timestamp=950_000), 10.6)
        self.assertTrue(state.fresh(10.6))

    def test_nonfinite_or_zero_initial_state_never_creates_anchor(self):
        for changes in ({"timestamp": 0}, {"heading": math.nan}, {"z": math.inf}):
            state = HoverState()
            state.observe(position(**changes), 10)
            self.assertIsNone(state.anchor)
            self.assertFalse(state.fresh(10))

    def test_publish_requests_px4_receipt_time_independent_of_mapped_clock(self):
        import time

        state = HoverState()
        state.observe(position(timestamp=714_000_000_000), time.monotonic())
        modes, targets = [], []
        node = SimpleNamespace(
            state=state,
            height=1,
            ready_samples=0,
            say=lambda _: None,
            mode_pub=SimpleNamespace(publish=modes.append, get_subscription_count=lambda: 1),
            target_pub=SimpleNamespace(publish=targets.append, get_subscription_count=lambda: 1),
        )
        HoverPublisher.publish_target(node)
        self.assertEqual(modes[0].timestamp, 0)
        self.assertEqual(targets[0].timestamp, modes[0].timestamp)
        self.assertEqual(list(targets[0].position), [2, 3, -1])

    def test_short_clock_correction_keeps_hover_heartbeat_with_original_target(self):
        import time

        now = time.monotonic()
        state = HoverState()
        state.observe(position(timestamp=714_000_000_000), now)
        state.observe(position(timestamp=713_993_000_000, x=2.01), now + 0.01)
        messages = []
        node = SimpleNamespace(
            state=state,
            height=1,
            ready_samples=50,
            say=lambda _: None,
            mode_pub=SimpleNamespace(publish=messages.append, get_subscription_count=lambda: 1),
            target_pub=SimpleNamespace(publish=messages.append, get_subscription_count=lambda: 1),
        )
        from unittest.mock import patch

        with patch("ros2_hover.time.monotonic", return_value=now + 0.1):
            HoverPublisher.publish_target(node)
        self.assertEqual(len(messages), 2)
        self.assertEqual(messages[0].timestamp, 0)
        self.assertEqual(list(messages[1].position), [2, 3, -1])
        with patch("ros2_hover.time.monotonic", return_value=now + 1.01):
            HoverPublisher.publish_target(node)
        self.assertEqual(len(messages), 2)  # No heartbeat after freshness expires.

    def test_stale_state_publishes_neither_target_nor_heartbeat(self):
        import time

        state = HoverState()
        state.observe(position(), time.monotonic() - 2)
        messages = []
        node = SimpleNamespace(
            state=state,
            height=1,
            ready_samples=50,
            say=lambda _: None,
            mode_pub=SimpleNamespace(publish=messages.append),
            target_pub=SimpleNamespace(publish=messages.append),
        )
        HoverPublisher.publish_target(node)
        self.assertEqual(messages, [])
        self.assertEqual(node.ready_samples, 0)


if __name__ == "__main__":
    unittest.main()
