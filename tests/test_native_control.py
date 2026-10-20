"""Differential timestamp tests and real native ROS publication on domain 97."""

import math
import os
import random
import socket
import sys
import threading
import tempfile
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

import numpy as np

from navigation.native_control import (
    LIBRARY_PATH,
    NativeTrajectoryClock,
    NativeSampleTimeline,
    NativeReferenceHistory,
    NativePublisher,
)
from navigation.source_time import SampleTimeline, ReferenceHistory, ClockPairEmitter
from navigation.timing import TrajectoryClock


@unittest.skipUnless(LIBRARY_PATH.exists(), "Build native_control to run native equivalence checks")
class NativeTimeTests(unittest.TestCase):
    def test_trajectory_clock_matches_python_across_gaps_duplicates_and_resets(self):
        python, native = TrajectoryClock(), NativeTrajectoryClock()
        self.addCleanup(native.close)
        stamp, now = 1_000_000_000, 100.0
        generator = random.Random(81)
        for index in range(3000):
            now += generator.choice([0.01, 0.05, 0.1, 0.3])
            stamp += generator.choice(
                [0, 0, 20_000_000, 50_000_000, 80_000_000, -10_000_000, 400_000_000]
            )
            if index % 79 == 0:
                python.invalidate()
                native.invalidate()
            python.observe(stamp, now)
            native.observe(stamp, now)
            self.assertEqual(python.consume(now), native.consume(now))
            self.assertEqual(python.fresh(now + 0.1), native.fresh(now + 0.1))
            self.assertEqual(python.report(now), native.report(now))

    def timelines(self):
        python, native = SampleTimeline(port=None), NativeSampleTimeline(port=None)
        self.addCleanup(native.close)
        return python, native

    def test_sample_clock_conversion_matches_offset_changes_ambiguity_staleness_and_reset(self):
        python, native = self.timelines()
        for index in range(400):
            now = 100 + index * 0.05
            sim = 10_000_000_000 + index * 50_000_000
            raw = sim // 1000 + 1_000_000
            offset = -1_000_000_000_000 + (index // 17) * 10_000
            for timeline in (python, native):
                timeline.clock.observe(
                    dict(
                        session="a" if index < 200 else "b", px4_us=raw, sim_ns=sim, sent_qpc_s=now
                    ),
                    now,
                    now,
                )
                if index % 17 == 0:
                    timeline.observe_timesync(
                        SimpleNamespace(
                            timestamp=raw - offset,
                            estimated_offset=offset,
                            source_protocol=2,
                            round_trip_time=0,
                        ),
                        now,
                    )
            for shift in (-40_000, 0, 10_000, 300_000):
                position = SimpleNamespace(
                    timestamp=raw - offset, timestamp_sample=raw - offset + shift
                )
                for elapsed in (0, 0.1, 0.3):
                    self.assertEqual(
                        python.sample_time(position, now + elapsed),
                        native.sample_time(position, now + elapsed),
                    )
            self.assertEqual(python.clock.epoch, native.clock.epoch)

    def test_reference_history_matches_dense_interpolation_and_epoch_changes(self):
        python, native = ReferenceHistory(), NativeReferenceHistory()
        self.addCleanup(native.close)
        generator = random.Random(13)
        stamp = 0
        for index in range(2000):
            stamp += generator.choice([0, 20_000_000, 50_000_000, 400_000_000])
            if index % 200 == 0:
                stamp = 0
            point = [index * 0.03, math.sin(index / 20)]
            for history in (python, native):
                history.record(stamp, point, [0.6, 0], index // 123)
                if index % 101 == 0:
                    history.clear()
            for delta in (-200_000_000, -10_000_000, 0, 80_000_000, 150_000_000):
                a, ma = python.at(stamp + delta)
                b, mb = native.at(stamp + delta)
                self.assertEqual(ma, mb)
                if a is None:
                    self.assertIsNone(b)
                else:
                    np.testing.assert_allclose(a, b, atol=1e-12)

    def test_native_udp_clock_receiver_uses_same_bridge_wire(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        timeline = NativeSampleTimeline(port=port)
        self.addCleanup(timeline.close)
        emitter = ClockPairEmitter(port)
        self.addCleanup(emitter.close)
        emitter.socket.sendto(b"invalid", emitter.peer)
        for index in range(3):
            emitter.emit(11_000_000 + index * 40_000, 10_000_000_000 + index * 40_000_000)
            timeline.poll(time.monotonic())
        now = time.monotonic()
        timeline.observe_timesync(
            SimpleNamespace(
                timestamp=1_000_011_000_000,
                estimated_offset=-1_000_000_000_000,
                source_protocol=2,
                round_trip_time=0,
            ),
            now,
        )
        self.assertEqual(
            timeline.sample_time(
                SimpleNamespace(timestamp=1_000_011_090_000, timestamp_sample=1_000_011_050_000),
                now,
            ),
            (10_050_000_000, None),
        )


@unittest.skipUnless(
    LIBRARY_PATH.exists() and os.environ.get("ROS_DOMAIN_ID") == "97",
    "Native ROS checks require a build and isolated ROS_DOMAIN_ID=97",
)
class NativePublisherTests(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode

        self.rclpy = rclpy
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = Node("native_test_observer_" + uuid.uuid4().hex[:8])
        self.addCleanup(self.node.destroy_node)
        self.prefix = "/native_test/n" + uuid.uuid4().hex
        self.publisher = NativePublisher(domain=97, prefix=self.prefix)
        self.addCleanup(self.publisher.close)
        self.targets, self.modes = [], []
        self.node.create_subscription(
            TrajectorySetpoint,
            self.prefix + "/trajectory_setpoint",
            self.targets.append,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            OffboardControlMode,
            self.prefix + "/offboard_control_mode",
            self.modes.append,
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + 5
        while (
            not (
                self.publisher.target.get_subscription_count()
                and self.publisher.mode.get_subscription_count()
            )
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertGreater(self.publisher.target.get_subscription_count(), 0)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.005)

    def test_cpp_fields_and_lease_expiry_stop_repeating_stale_commands(self):
        self.publisher.submit([2, 3, -1], [0.6, 0.1], [0.25, 0], 0.4)
        self.spin(0.16)
        self.assertTrue(self.targets and self.modes)
        target, mode = self.targets[-1], self.modes[-1]
        self.assertEqual(target.timestamp, 0)
        self.assertEqual(mode.timestamp, 0)
        self.assertTrue(mode.position)
        self.assertFalse(mode.velocity or mode.acceleration)
        np.testing.assert_array_equal(target.position, [2, 3, -1])
        np.testing.assert_allclose(target.velocity[:2], [0.6, 0.1])
        np.testing.assert_allclose(target.acceleration[:2], [0.25, 0])
        self.assertTrue(math.isnan(target.velocity[2]) and math.isnan(target.acceleration[2]))
        self.assertTrue(all(math.isnan(v) for v in target.jerk))
        first = self.publisher.report()
        self.assertEqual(first["expired"], 1)
        self.spin(0.12)
        self.assertEqual(self.publisher.report()["published"], first["published"])

    def test_report_process_initializes_ros_and_writes_only_its_test_directory(self):
        import json
        from navigation.process_worker import ProcessService
        from test_navigation import snapshot

        with tempfile.TemporaryDirectory() as folder:
            project = Path(folder)
            (project / "navigation").mkdir()
            (project / "config").mkdir()
            (project / "logs").mkdir()
            for name in (
                "ros2_navigate.py",
                "ros2_hover.py",
                "ros2_step.py",
                "config/navigation_settings.json",
            ):
                (project / name).write_text("test")
            service = ProcessService(
                "test_report", "navigation.report_jobs:ReportWriter", project, 1.0
            )
            try:
                runtime = service.call(
                    (dict(path=[[2, 3], [2, 4]], trail=[], phase="HOLD"), snapshot())
                )
                self.assertFalse(runtime["reload_pending"])
                report = json.loads((project / "logs" / "navigation_state.json").read_text())
                self.assertEqual(report["phase"], "HOLD")
                self.assertEqual(
                    len((project / "logs" / "navigation_trace.jsonl").read_text().splitlines()), 1
                )
            finally:
                service.close()

    def test_native_publication_continues_while_python_holds_the_gil(self):
        self.publisher.submit([0, 0, -1], [0.6, 0], [0, 0], 0)
        previous = sys.getswitchinterval()
        sys.setswitchinterval(0.2)
        try:
            until = time.perf_counter() + 0.09
            value = 0
            while time.perf_counter() < until:
                value = (value * 1664525 + 1013904223) & 0xFFFFFFFF
        finally:
            sys.setswitchinterval(previous)
        self.assertGreaterEqual(self.publisher.report()["published"], 2)

    def test_old_invalid_and_revoked_commands_cannot_extend_motion_permission(self):
        for position, issued, reason in (
            ([0, 0, -1], time.perf_counter_ns() - 200_000_000, "stale_command"),
            ([math.nan, 0, -1], None, "invalid_frame"),
            ([0, 0, -1], time.perf_counter_ns() + 1_000_000_000, "future_command"),
        ):
            with self.assertRaises(RuntimeError) as failure:
                self.publisher.submit(position, [0.6, 0], [0, 0], 0, issued_ns=issued)
            self.assertEqual(failure.exception.submission["reason"], reason)
            self.assertGreaterEqual(failure.exception.submission["lock_wait_s"], 0)
        rejection = self.publisher.report()["last_rejection"]
        self.publisher.submit([0, 0, -1], [0.6, 0], [0, 0], 0)
        self.assertEqual(self.publisher.report()["last_rejection"], rejection)
        self.publisher.revoke()
        published = self.publisher.report()["published"]
        self.spin(0.12)
        self.assertEqual(self.publisher.report()["published"], published)

    def test_real_command_generation_meets_lease_during_child_cpu_stalls(self):
        from ros2_navigate import GoalNavigator
        from test_navigation import navigator_fixture
        from test_process_isolation import probe_service

        services = [probe_service(), probe_service()]
        for service in services:
            self.addCleanup(service.close)
            service.call(("echo", None))
        threads = [threading.Thread(target=s.call, args=(("busy", 0.8),)) for s in services]
        for thread in threads:
            thread.start()
        node, _, _, _ = navigator_fixture()
        node.native_control = self.publisher
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        timings = []
        end = time.perf_counter() + 0.35
        while time.perf_counter() < end:
            node.pacer.total_s += 0.02
            with patch("ros2_navigate.time.monotonic", return_value=100):
                GoalNavigator.publish_target(node)
            timings.append(node.last_control_timing)
            self.spin(0.015)
        self.assertGreaterEqual(len(timings), 6)
        self.assertEqual(self.publisher.report()["rejected"], 0)
        self.assertEqual(node.follower.phase, "NAVIGATING")
        self.assertTrue(all(t["submission"]["command_age_s"] < 0.1 for t in timings))
        self.assertTrue(any(t.is_alive() for t in threads))
        for thread in threads:
            thread.join(3)
        self.assertTrue(self.targets)

    def test_deadline_miss_records_the_slow_stage_and_survives_subsequent_hold(self):
        from ros2_navigate import GoalNavigator
        from test_navigation import navigator_fixture

        node, _, _, _ = navigator_fixture()
        node.native_control = self.publisher
        node.trajectory_clock.report = lambda now: dict(available=True)
        rows = []
        node.diagnostics = SimpleNamespace(
            emit=lambda kind, **fields: rows.append(dict(kind=kind, **fields))
        )
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.pacer.total_s += 0.05

        def delayed_validation(*args):
            time.sleep(0.12)
            return True

        with (
            patch("ros2_navigate.time.monotonic", return_value=100),
            patch("ros2_navigate.remaining_safe", side_effect=delayed_validation),
        ):
            GoalNavigator.publish_target(node)
        rejection = next(r for r in rows if r["kind"] == "control_rejection")
        self.assertEqual(rejection["submission"]["reason"], "stale_command")
        self.assertGreaterEqual(rejection["stages"]["remaining_route_validation"]["wall_s"], 0.12)
        self.assertGreaterEqual(rejection["submission"]["python_before_native_s"], 0.12)
        self.assertTrue(rejection["deadline_missed"])
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertEqual(len(node.reference_history.rows), 0)
        stop = next(r for r in rows if r["kind"] == "stop")
        self.assertEqual(stop["control_timing"]["submission"]["reason"], "stale_command")
        self.publisher.submit([0, 0, -1], [0, 0], [0, 0], 0)
        self.assertEqual(self.publisher.report()["last_rejection"]["reason"], "stale_command")
        self.assertTrue(any(r["kind"] == "control_timing" for r in rows))

    def test_python_navigator_submits_actual_reference_through_native_publisher(self):
        from ros2_navigate import GoalNavigator
        from test_navigation import navigator_fixture

        node, _, _, _ = navigator_fixture()
        node.native_control = self.publisher
        node.mode_pub, node.target_pub = self.publisher.mode, self.publisher.target
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.pacer.total_s += 0.05
        with patch("ros2_navigate.time.monotonic", return_value=100):
            GoalNavigator.publish_target(node)
        self.spin(0.03)
        self.assertTrue(self.targets)
        np.testing.assert_allclose(self.targets[-1].position[:2], node.reference)
        np.testing.assert_allclose(self.targets[-1].velocity[:2], node.velocity_reference_ned)
        np.testing.assert_allclose(
            self.targets[-1].acceleration[:2], node.acceleration_reference_ned
        )
        node.state.last_progress = None
        with patch("ros2_navigate.time.monotonic", return_value=100):
            GoalNavigator.publish_target(node)
        before = self.publisher.report()["published"]
        self.spin(0.12)
        self.assertEqual(self.publisher.report()["published"], before)

    def test_cpp_timesync_subscription_receives_status_without_python_dispatch(self):
        from px4_msgs.msg import TimesyncStatus
        from rclpy.qos import qos_profile_sensor_data

        native = NativeSampleTimeline(port=None)
        self.addCleanup(native.close)
        publisher = NativePublisher(timeline=native, domain=97, prefix=self.prefix + "/time")
        self.addCleanup(publisher.close)
        source = self.node.create_publisher(
            TimesyncStatus, "/fmu/out/timesync_status", qos_profile_sensor_data
        )
        deadline = time.monotonic() + 5
        while not source.get_subscription_count() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
        now = time.monotonic()
        for index in range(3):
            native.clock.observe(
                dict(
                    session="native",
                    px4_us=11_000_000 + index * 40_000,
                    sim_ns=10_000_000_000 + index * 40_000_000,
                    sent_qpc_s=now + index * 0.001,
                ),
                now,
                now + index * 0.001,
            )
        message = TimesyncStatus()
        message.timestamp = 1_000_011_000_000
        message.estimated_offset = -1_000_000_000_000
        message.source_protocol = 2
        result = (None, None)
        deadline = time.monotonic() + 1
        index = 3
        while result[0] is None and time.monotonic() < deadline:
            now = time.monotonic()
            native.clock.observe(
                dict(
                    session="native",
                    px4_us=11_080_000 + index * 1000,
                    sim_ns=10_080_000_000 + index * 1_000_000,
                    sent_qpc_s=now,
                ),
                now,
                now,
            )
            source.publish(message)
            self.spin(0.01)
            result = native.sample_time(
                SimpleNamespace(timestamp=1_000_011_090_000, timestamp_sample=1_000_011_050_000),
                time.monotonic(),
            )
            index += 1
        self.assertEqual(result, (10_050_000_000, None))

    def test_slow_python_validation_pauses_without_recording_an_unpublished_reference(self):
        from ros2_navigate import GoalNavigator
        from test_navigation import navigator_fixture

        node, _, _, _ = navigator_fixture()
        node.native_control = self.publisher
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.pacer.total_s += 0.05
        old = time.perf_counter_ns() - 200_000_000
        with (
            patch("ros2_navigate.time.monotonic", return_value=100),
            patch("ros2_navigate.time.perf_counter_ns", return_value=old),
        ):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertIn("Native publisher rejected", node.follower.reason)
        self.assertEqual(self.publisher.report()["published"], 0)
        self.assertEqual(len(node.reference_history.rows), 0)
        self.assertEqual(node.ready_samples, 0)

    def test_real_cpp_navigator_has_one_native_target_publisher_and_no_python_duplicate(self):
        from queue import Queue
        from navigation.dashboard import Dashboard
        from navigation.settings import load_settings
        from ros2_navigate import GoalNavigator

        class IsolatedNavigator(GoalNavigator):
            def publish_notice(self, item):
                pass

            def publish_reports(self, item):
                pass

        clock = NativeTrajectoryClock()
        self.addCleanup(clock.close)
        feed = SimpleNamespace(get=lambda: (None, "No test flight input"), trajectory_clock=clock)
        commands = Queue()
        dashboard = Dashboard(feed, commands, port=0)
        self.addCleanup(dashboard.close)
        goal = IsolatedNavigator(
            feed, dashboard, commands, load_settings(), control_backend="cpp", timeline_port=0
        )
        self.addCleanup(goal.destroy_node)
        end = time.monotonic() + 2
        publishers = []
        while not publishers and time.monotonic() < end:
            self.rclpy.spin_once(goal, timeout_sec=0.01)
            publishers = goal.get_publishers_info_by_topic("/fmu/in/trajectory_setpoint")
        self.assertEqual(len(publishers), 1)
        from navigation.native_control import NativeReaderCount

        self.assertIsInstance(goal.mode_pub, NativeReaderCount)
        self.assertFalse(
            [
                p
                for p in goal.publishers
                if p.topic_name in ("/fmu/in/trajectory_setpoint", "/fmu/in/offboard_control_mode")
            ]
        )
        self.assertEqual(goal.native_control.report()["published"], 0)
        with self.assertRaises(RuntimeError):
            goal.native_control.benchmark_start(1)  # Synthetic setpoints never use flight topics.
        relative = NativePublisher(domain=97, prefix="fmu/in")
        self.addCleanup(relative.close)
        with self.assertRaises(RuntimeError):
            relative.benchmark_start(1)  # Relative ROS names must not bypass that protection.
        rejected = False
        deadline = time.monotonic() + 2
        while not rejected and time.monotonic() < deadline:
            try:
                goal.native_control.submit([0, 0, -1], [0, 0], [0, 0], 0)
            except RuntimeError:
                rejected = True
            self.spin(0.01)
        self.assertTrue(rejected, "Native publisher must reject a competing flight-topic publisher")
        self.assertGreater(goal.native_control.report()["rejected"], 0)

    def test_native_clock_guard_clears_motion_feedforward_even_with_a_new_command(self):
        self.publisher.close()
        clock = NativeTrajectoryClock()
        self.addCleanup(clock.close)
        publisher = NativePublisher(clock=clock, domain=97, prefix=self.prefix)
        self.addCleanup(publisher.close)
        end = time.monotonic() + 5
        while not publisher.target.get_subscription_count() and time.monotonic() < end:
            self.spin(0.01)

        def wait_for_target(x):
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                match = next((target for target in self.targets if target.position[0] == x), None)
                if match is not None:
                    return match
                self.rclpy.spin_once(self.node, timeout_sec=0.005)
            self.fail(f"No target at x={x} received within one second")

        now = time.monotonic()
        for index in range(5):
            clock.observe(index * 50_000_000, now)
        publisher.submit([41, 0, -1], [0.6, 0], [0.25, 0], 0)
        self.assertAlmostEqual(wait_for_target(41).velocity[0], 0.6)
        clock.invalidate()
        publisher.submit([42, 0, -1], [0.6, 0], [0.25, 0], 0)
        target = wait_for_target(42)
        np.testing.assert_array_equal(target.velocity[:2], [0, 0])
        np.testing.assert_array_equal(target.acceleration[:2], [0, 0])


if __name__ == "__main__":
    unittest.main()
