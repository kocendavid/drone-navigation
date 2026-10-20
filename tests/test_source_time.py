"""Source-clock conversion, telemetry delays and acceleration feedforward; no flight."""

import json
import math
import socket
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from navigation.motion import RouteFollower, FrameAlignment, acceleration_feedforward
from navigation.settings import load_settings
from navigation.source_time import (
    ClockPairEmitter,
    SampleTimeline,
    SimulatorClock,
    ReferenceHistory,
)


def pair(index=0, session="bridge"):
    return dict(
        session=session,
        px4_us=11_000_000 + index * 40_000,
        sim_ns=10_000_000_000 + index * 40_000_000,
        sent_qpc_s=100 + index * 0.04,
    )


def synced_timeline():
    timeline = SampleTimeline(port=None)
    for index in range(3):
        timeline.clock.observe(pair(index), 100 + index * 0.04, 100 + index * 0.04)
    timeline.observe_timesync(
        SimpleNamespace(
            timestamp=11_000_000 + 1_000_000_000_000,
            estimated_offset=-1_000_000_000_000,
            source_protocol=2,
            round_trip_time=0,
        ),
        100,
    )
    return timeline


class SourceTimeTests(unittest.TestCase):
    def test_clock_mapping_requires_confirmation_and_rejects_duplicates_and_stale_pairs(self):
        clock = SimulatorClock()
        for index in range(2):
            clock.observe(pair(index), 100 + index * 0.04, 100 + index * 0.04)
        self.assertIsNone(clock.convert(11_040_000, 100.04))
        clock.observe(pair(2), 100.08, 100.08)
        self.assertEqual(clock.convert(11_050_000, 100.1), 10_050_000_000)
        duplicate = dict(pair(2), sent_qpc_s=100.2)
        clock.observe(duplicate, 100.2, 100.2)
        self.assertIsNone(clock.convert(11_080_000, 100.34))
        clock.observe(pair(3), 100.5, 100.5)  # Buffered for >250 ms.
        self.assertIsNone(clock.convert(11_080_000, 100.5))
        self.assertIsNone(clock.convert(10_000_000, 100.1))

    def test_reset_session_jump_and_mapping_change_require_new_confirmation(self):
        for change in ("session", "reset", "offset"):
            clock = synced_timeline().clock
            row = pair(3)
            if change == "session":
                row["session"] = "new_bridge"
            elif change == "reset":
                row["sim_ns"] = 1
            else:
                row["px4_us"] += 5000
            clock.observe(row, 100.12, 100.12)
            self.assertEqual(clock.epoch, 1)
            self.assertIsNone(clock.convert(row["px4_us"], 100.12))

    def test_xrce_offset_is_removed_before_isaac_sample_time_conversion(self):
        timeline = synced_timeline()
        position = SimpleNamespace(timestamp=1_000_011_090_000, timestamp_sample=1_000_011_050_000)
        self.assertEqual(timeline.sample_time(position, 100.1), (10_050_000_000, None))
        self.assertEqual(timeline.sample_time(position, 100.4)[1], "clock_mapping_unavailable")
        position.timestamp_sample = position.timestamp + 1
        self.assertEqual(timeline.sample_time(position, 100.1)[1], "invalid_sample_timestamp")

    def test_timesync_change_uses_its_native_epoch_and_ignores_other_protocols(self):
        timeline = synced_timeline()
        timeline.observe_timesync(
            SimpleNamespace(
                timestamp=2_000_011_060_000,
                estimated_offset=-2_000_000_000_000,
                source_protocol=2,
                round_trip_time=0,
            ),
            100.06,
        )
        for native_stamp, sample, adjustment in (
            (11_040_000, 11_020_000, 1_000_000_000_000),
            (11_090_000, 11_050_000, 2_000_000_000_000),
        ):
            p = SimpleNamespace(
                timestamp=native_stamp + adjustment, timestamp_sample=sample + adjustment
            )
            self.assertEqual(timeline.sample_time(p, 100.1), ((sample - 1_000_000) * 1000, None))
        length = len(timeline.sync)
        timeline.observe_timesync(SimpleNamespace(source_protocol=1), 100.1)
        self.assertEqual(len(timeline.sync), length)

    def test_clock_wire_carries_only_clocks_and_malformed_packets_are_ignored(self):
        timeline = SampleTimeline(port=0)
        self.addCleanup(timeline.close)
        sender = ClockPairEmitter(port=timeline.socket.getsockname()[1])
        self.addCleanup(sender.close)
        sender.socket.sendto(b"not json", sender.peer)
        for index in range(3):
            with patch("navigation.source_time.time.perf_counter", return_value=100 + index * 0.04):
                sender.emit(pair(index)["px4_us"], pair(index)["sim_ns"])
                timeline.poll(100 + index * 0.04)
        self.assertEqual(timeline.clock.convert(11_050_000, 100.1), 10_050_000_000)
        sender.socket.sendto(json.dumps(dict(pair(4), px4_us="invalid")).encode(), sender.peer)
        timeline.poll(100.1)
        self.assertEqual(timeline.clock.confirmed, 3)

    def test_port_conflict_keeps_alignment_unavailable_without_blocking_navigation(self):
        occupied = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        occupied.bind(("127.0.0.1", 0))
        self.addCleanup(occupied.close)
        timeline = SampleTimeline(port=occupied.getsockname()[1])
        self.assertIsNone(timeline.socket)
        timeline.poll(100)
        timeline.close()

    def test_reference_history_interpolates_and_bounds_prediction_gaps_and_resets(self):
        history = ReferenceHistory()
        history.record(0, [0, 0], [1, 0], "a")
        history.record(100_000_000, [0.1, 0], [1, 0], "a")
        np.testing.assert_allclose(history.at(50_000_000)[0], [0.05, 0])
        self.assertEqual(history.at(50_000_000)[1], "source_time_interpolated")
        np.testing.assert_allclose(history.at(180_000_000)[0], [0.18, 0])
        self.assertEqual(history.at(180_000_000)[1], "source_time_predicted")
        self.assertIsNone(history.at(250_000_000)[0])
        history.record(500_000_000, [0.5, 0], [1, 0], "a")
        self.assertIsNone(history.at(300_000_000)[0])
        history.record(600_000_000, [9, 0], [0, 0], "b")
        self.assertIsNone(history.at(500_000_000)[0])
        history.record(50, [0, 0], [0, 0], "b")
        self.assertEqual(len(history.rows), 1)

    def test_aligned_tracking_avoids_false_stop_but_retains_real_tracking_stop(self):
        for displacement, expected_stop in ((0, False), (-0.3, True)):
            follower = RouteFollower(load_settings())
            follower.begin([[0, 0], [100, 0]])
            follower.target = np.array([0.2, 0.0])
            follower.velocity = np.array([0.6, 0.0])
            follower.speed = 0.6
            # Telemetry is from an earlier source instant. The old comparison
            # adds another 6 cm to the present reference, crossing the guard.
            follower.tick(
                0.1,
                [displacement, 0],
                np.empty((0, 2)),
                observed_corridor=True,
                measured_velocity=[0.6, 0],
                braking_safe=lambda _: True,
                tracking_reference=[0, 0],
            )
            self.assertAlmostEqual(follower.tracking["error_m"], abs(displacement))
            self.assertEqual(follower.braking_envelope is not None, expected_stop)

    def test_acceleration_feedforward_preserves_vector_bounds_and_ned_frame(self):
        alignment = FrameAlignment([99, 88], 0, [1, 2], 0, position_yaw=0.6)
        acceleration = acceleration_feedforward([0.5, 0], [0.503, 0.004], 0.02, 0.25)
        self.assertAlmostEqual(np.linalg.norm(acceleration), 0.25)
        np.testing.assert_allclose(
            alignment.world_velocity(alignment.ned_velocity(acceleration)), acceleration
        )
        for dt, velocity in ((0, [1, 0]), (0.6, [1, 0]), (0.02, [4, 0]), (0.02, [math.nan, 0])):
            self.assertIsNone(acceleration_feedforward([0, 0], velocity, dt, 0.25))

    def test_acceleration_feedforward_reduces_lag_in_velocity_response_model(self):
        peaks = []
        for enabled in (False, True):
            target = speed = actual = velocity = peak = 0.0
            dt, response = 0.01, 0.5
            for index in range(1000):
                acceleration = 0.25 if index < 400 else 0.0
                speed += acceleration * dt
                target += speed * dt
                command = speed + 0.95 * (target - actual)
                velocity += (
                    (command - velocity) / response + (acceleration if enabled else 0)
                ) * dt
                actual += velocity * dt
                peak = max(peak, abs(target - actual))
            peaks.append(peak)
        self.assertLess(peaks[1], peaks[0] * 0.25)

    def test_navigator_uses_sample_aligned_history_and_logs_consumed_clock_stamp(self):
        from ros2_navigate import GoalNavigator
        from test_navigation import navigator_fixture

        node, _, targets, _ = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.follower.target = np.array([2, 3.2])
        node.follower.velocity = np.array([0, 0.6])
        node.follower.speed = 0.6
        node.position.vx = 0.6
        node.position.timestamp = 1_000_011_090_000
        node.position.timestamp_sample = 1_000_011_050_000
        node.sample_timeline = synced_timeline()
        epoch = (0, "NAVIGATING", (), 0)
        node.reference_history.record(10_000_000_000, [2, 2.97], [0, 0.6], epoch)
        node.reference_history.record(10_100_000_000, [2, 3.03], [0, 0.6], epoch)
        node.trajectory_clock.consume = lambda _: 0.1
        node.trajectory_clock.last_decision = dict(source_stamp_ns=10_250_000_000)
        node.trajectory_clock.stamp = 10_300_000_000  # New clock arrives after consumption.
        node.trajectory_clock.report = lambda _: dict(available=True)
        records = []
        node.diagnostics = SimpleNamespace(
            emit=lambda kind, **fields: records.append(dict(kind=kind, **fields))
        )
        with patch("ros2_navigate.time.monotonic", return_value=100.1):
            GoalNavigator.publish_target(node)
        self.assertIsNone(node.follower.braking_envelope)
        self.assertAlmostEqual(node.follower.tracking["error_m"], 0)
        self.assertEqual(node.tracking_alignment["mode"], "source_time_interpolated")
        self.assertEqual(node.tracking_alignment["sample_sim_ns"], 10_050_000_000)
        reference = next(r for r in records if r["kind"] == "reference")
        self.assertEqual(reference["sim_ns"], 10_250_000_000)
        self.assertLessEqual(np.linalg.norm(targets[-1].acceleration[:2]), 0.25 + 1e-7)
        self.assertTrue(math.isnan(targets[-1].acceleration[2]))
        json.dumps(reference, allow_nan=False)

    def test_navigator_braking_acceleration_persists_between_samples_and_clears_on_hold(self):
        from ros2_navigate import GoalNavigator
        from test_navigation import navigator_fixture

        node, _, targets, _ = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.follower.velocity = np.array([0, 0.2])
        node.follower.speed = 0.2
        node.follower.braking_envelope = np.tile([2.0, 3.0], (4, 1))
        node.feed.get()[0].hits = np.empty((0, 2))  # Map-certified open stopping strip.
        node.trajectory_clock.consume = lambda _: 0.05
        with patch("ros2_navigate.time.monotonic", return_value=100):
            GoalNavigator.publish_target(node)
            self.assertEqual(node.follower.phase, "NAVIGATING", node.follower.reason)
            np.testing.assert_allclose(targets[-1].acceleration[:2], [-0.25, 0], atol=1e-7)
            node.trajectory_clock.consume = lambda _: 0
            GoalNavigator.publish_target(node)
            np.testing.assert_allclose(targets[-1].acceleration[:2], [-0.25, 0], atol=1e-7)
            node.commands.put(dict(action="hold"))
            GoalNavigator.publish_target(node)
        np.testing.assert_array_equal(targets[-1].acceleration[:2], [0, 0])
        np.testing.assert_array_equal(targets[-1].velocity[:2], [0, 0])


if __name__ == "__main__":
    unittest.main()
