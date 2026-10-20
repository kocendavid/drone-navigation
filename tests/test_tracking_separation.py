"""Background stalls cannot own flight publication or bypass current-map checks."""

from collections import deque
from dataclasses import replace
import threading
from types import MethodType
import unittest
from unittest.mock import patch

import numpy as np

from navigation.background import LatestWorker
from navigation.async_frontiers import AsyncFrontiers
from navigation.curves import speed_profile
from navigation.motion import RouteFollower
from navigation.prepared_route import PreparedRoute, remaining_safe, route_safe
from navigation.settings import load_settings
from navigation.mapping import FREE, UNKNOWN
from ros2_navigate import GoalNavigator
from test_navigation import snapshot, navigator_fixture


class TrackingSeparationTests(unittest.TestCase):
    def analyzed_frontiers(self, node, snap):
        ledger = AsyncFrontiers()
        self.addCleanup(ledger.close)
        node.exploration.frontier_ledger = ledger
        ledger.observe_map(snap)
        with ledger.worker.condition:
            self.assertTrue(
                ledger.worker.condition.wait_for(
                    lambda: ledger.worker.result and ledger.worker.result[0] == snap.stamp, 2
                )
            )
        return ledger

    def test_enclosed_map_completes_before_final_arrival_and_keeps_hover_stream(self):
        for continuous in (False, True):
            with self.subTest(continuous=continuous):
                node, modes, targets, _ = navigator_fixture()
                snap = node.feed.get()[0]
                node.exploration.start(snap, continuous=continuous)
                node.exploration.phase = "TRAVELLING"
                node.position.vx = 0.2  # Completion does not require arrival or stable hover.
                self.assertGreater(np.linalg.norm(node.target_goal - node.world_pose), 0.5)
                ledger = self.analyzed_frontiers(node, snap)
                self.assertTrue(ledger.completion_for(snap).complete)
                old_generation = node.plan_generation
                # A planner still finishing the old destination cannot restart travel.
                node.plan_results.put((old_generation, [[2, 3], [2, 2]], None, None))
                replies = []
                node.active_ui_command = dict(action="explore", request_id="explore")
                node.dashboard.acknowledge_command = lambda req, status, reason: replies.append(
                    status
                )
                with patch("ros2_navigate.time.monotonic", return_value=100.0):
                    for _ in range(2):
                        GoalNavigator.publish_target(node)
                node.update_command_progress()
                self.assertEqual(node.exploration.phase, "COMPLETE")
                self.assertEqual(node.follower.phase, "COMPLETE")
                self.assertFalse(node.exploration.enabled)
                self.assertIsNone(node.target_goal)
                self.assertEqual(node.follower.path, [])
                self.assertIsNone(node.follower.prepared_route)
                self.assertEqual(node.exploration.reached, 0)
                self.assertEqual(node.exploration.visited, [])
                self.assertGreater(node.plan_generation, old_generation)
                self.assertEqual(replies, ["completed"])
                self.assertEqual(len(modes), 2)
                self.assertEqual(len(targets), 2)
                np.testing.assert_array_equal(node.follower.target, node.world_pose)
                np.testing.assert_array_equal(node.follower.velocity, [0, 0])
                self.assertEqual(node.mission.phase, "IDLE")

    def test_completed_routes_publish_an_empty_path_while_active_routes_remain_visible(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        reports = []
        node.dashboard.observe = lambda report, snap, now: reports.append(report)
        node.trajectory_clock.report = lambda now: dict(available=True)
        node.velocity_reference_ned = np.zeros(2)
        for phase in ("COMPLETE", "REACHED", "NAVIGATING"):
            node.follower.phase = phase
            GoalNavigator.update_dashboard(node, 100.0, snap, True)
            self.assertEqual(bool(reports[-1]["path"]), phase == "NAVIGATING")
        self.assertTrue(node.follower.path)

    def test_unknown_boundary_or_open_map_edge_keeps_exploration_incomplete(self):
        for scenario in ("small_unknown_hole", "map_edge"):
            with self.subTest(scenario=scenario):
                node, _, _, _ = navigator_fixture()
                snap = node.feed.get()[0]
                if scenario == "small_unknown_hole":
                    snap.model.grid[30, 30] = UNKNOWN
                else:
                    snap.model.grid[:] = FREE
                node.exploration.start(snap)
                node.exploration.phase = "TRAVELLING"
                ledger = self.analyzed_frontiers(node, snap)
                self.assertFalse(ledger.completion_for(snap).complete)
                node.exploration_tick(100.0, snap, True)
                self.assertTrue(node.exploration.enabled)
                self.assertEqual(node.follower.phase, "NAVIGATING")

    def test_completion_requires_current_valid_map_and_an_active_session(self):
        for scenario in ("newer_scan", "invalid", "map_not_ready", "hold", "paused"):
            with self.subTest(scenario=scenario):
                node, _, _, _ = navigator_fixture()
                snap = node.feed.get()[0]
                node.exploration.start(snap)
                node.exploration.phase = "TRAVELLING"
                ledger = self.analyzed_frontiers(node, snap)
                if scenario == "newer_scan":
                    snap.stamp += 1
                    snap.model.grid[30, 30] = UNKNOWN
                    self.assertIsNone(ledger.completion_for(snap))
                elif scenario == "invalid":
                    snap.valid = False
                    self.assertIsNone(ledger.completion_for(snap))
                elif scenario == "hold":
                    node.stop("Operator hold", "HOLD", True)
                elif scenario == "paused":
                    node.stop("Position unavailable")
                node.exploration_tick(100.0, snap, scenario != "map_not_ready")
                self.assertNotEqual(node.exploration.phase, "COMPLETE")
                self.assertNotEqual(node.follower.phase, "COMPLETE")

    def test_enclosure_during_travel_starts_mission_verification_before_arrival(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.mission.start()
        node.exploration.start(snap)
        node.exploration.phase = "TRAVELLING"
        self.analyzed_frontiers(node, snap)
        node.exploration_tick(100.0, snap, True)
        self.assertIsNone(node.target_goal)
        self.assertFalse(node.exploration.enabled)
        self.assertEqual(node.mission.phase, "VERIFYING")
        self.assertEqual(node.mission.confirmations, 1)
        self.assertIsNone(node.mission.saved_map)
        self.assertEqual(node.mission.land_attempts, 0)

    def worker(self, handler):
        worker = LatestWorker("test_tracking_work", handler)
        self.addCleanup(worker.close)
        return worker

    def test_slow_worker_is_bounded_latest_only_and_discards_result_after_close(self):
        entered, release = threading.Event(), threading.Event()
        calls = []

        def work(value):
            calls.append(value)
            entered.set()
            release.wait(2)
            return value

        worker = self.worker(work)
        self.addCleanup(release.set)
        worker.submit(1, "first")
        self.assertTrue(entered.wait(1))
        worker.submit(2, "middle")
        worker.submit(3, "latest")
        with worker.condition:
            self.assertEqual(worker.pending, (3, "latest"))
            self.assertEqual(worker.superseded, 1)
        release.set()
        with worker.condition:
            self.assertTrue(worker.condition.wait_for(lambda: worker.result == (3, "latest"), 1))
        self.assertEqual(calls, ["first", "latest"])
        worker.close()
        worker.submit(4, "after-close")
        self.assertEqual(calls, ["first", "latest"])

    def test_prepared_footprints_equal_segment_checks_including_corner_cutting(self):
        snap = snapshot()
        path = [[-3, 0], [0, 2], [2, -2], [3, 0]]
        prepared = PreparedRoute(path, snap.model, load_settings())
        rng = np.random.default_rng(93)
        for _ in range(80):
            snap.body_free[:] = True
            y = rng.integers(0, snap.body_free.shape[0], 4)
            x = rng.integers(0, snap.body_free.shape[1], 4)
            snap.body_free[y, x] = False
            self.assertEqual(route_safe(snap, prepared), route_safe(snap, path))
        snap.body_free[:] = True
        self.assertTrue(prepared.safe(snap))
        diagonal = [snap.model.world((10, 10)), snap.model.world((11, 11))]
        diagonal_route = PreparedRoute(diagonal, snap.model, load_settings())
        snap.body_free[10, 11] = False
        self.assertFalse(route_safe(snap, diagonal))
        self.assertFalse(route_safe(snap, diagonal_route))
        changed = replace(snap)
        changed.model = type(snap.model)(size=20, resolution=0.2)
        self.assertFalse(prepared.safe(changed))

    def test_rebased_speed_profile_matches_full_rebuild_and_old_route_stays_immutable(self):
        snap = snapshot()
        settings = load_settings()
        paths = (
            [[-3, 0], [0, 0]],
            [[-3, 0], [0, 0], [1, 0.1], [3, 1]],
            [[-3, 0], [0, 0], [0, 3]],
        )
        for path in paths:
            prepared = PreparedRoute(path, snap.model, settings)
            original_limits = prepared.limits.copy()
            for start in ([-2.8, 0.1], [-3.2, -0.1], [-3, 0]):
                moved = prepared.with_start(start)
                rebuilt = speed_profile(
                    [np.asarray(p) for p in moved],
                    settings.acceleration_m_s2,
                    settings.cruise_m_s,
                )
                np.testing.assert_allclose(moved.limits, rebuilt)
                np.testing.assert_array_equal(prepared.limits, original_limits)
                self.assertEqual(prepared[0], path[0])

    def test_cached_remaining_route_detects_new_obstacle_without_rebuilding_geometry(self):
        snap = snapshot()
        prepared = PreparedRoute([[-3, 0], [0, 0], [3, 0]], snap.model, load_settings())
        follower = RouteFollower(load_settings())
        with patch("navigation.motion.speed_profile", side_effect=AssertionError("Hot rebuild")):
            follower.begin(prepared)
            follower.retarget(prepared)
        with patch(
            "navigation.prepared_route.footprint", side_effect=AssertionError("Hot footprint")
        ):
            self.assertTrue(remaining_safe(follower, snap, [-3, 0]))
            snap.body_free[snap.model.cell([2, 0])] = False
            self.assertFalse(remaining_safe(follower, snap, [-3, 0]))
        follower.index = 2
        follower.target = np.array([0.0, 0.0])
        snap.body_free[:] = True
        snap.body_free[snap.model.cell([-2, 0])] = False  # Traversed prefix is irrelevant.
        self.assertTrue(remaining_safe(follower, snap, [0, 0]))

    def test_blocked_frontier_and_reporting_workers_do_not_block_real_setpoint_callback(self):
        node, modes, targets, _ = navigator_fixture()
        snap = node.feed.get()[0]
        entered_frontier, entered_report, release = (
            threading.Event(),
            threading.Event(),
            threading.Event(),
        )
        ledger = AsyncFrontiers()
        self.addCleanup(ledger.close)
        self.addCleanup(release.set)
        analyze = ledger.worker.handler

        def slow_frontier(value):
            entered_frontier.set()
            release.wait(2)
            return analyze(value)

        def slow_report(value):
            entered_report.set()
            release.wait(2)

        ledger.worker.handler = slow_frontier
        node.exploration.frontier_ledger = ledger
        node.reporting = self.worker(slow_report)
        node.trajectory_clock.report = lambda now: dict(available=True)
        node.dashboard.observe = lambda report, snap, now: None
        node.update_dashboard = MethodType(GoalNavigator.update_dashboard, node)
        prepared = PreparedRoute([[2, 3], [2, 4]], snap.model, node.settings)
        node.follower.begin(prepared)
        node.target_goal = np.array([2, 4])
        node.pose_history = deque([(100.0, [2.0, 3.0], 0.0)])
        ledger.observe_map(snap)
        node.reporting.submit(-1, None)
        self.assertTrue(entered_frontier.wait(1))
        self.assertTrue(entered_report.wait(1))
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            for _ in range(6):
                node.pacer.total_s += 0.05
                GoalNavigator.publish_target(node)
        self.assertFalse(release.is_set())
        self.assertEqual(len(modes), 6)
        self.assertEqual(len(targets), 6)
        self.assertEqual(node.follower.phase, "NAVIGATING")
        self.assertGreater(np.linalg.norm(targets[-1].velocity[:2]), 0)
        release.set()

    def test_changed_notice_is_queued_without_console_or_file_io_on_callback(self):
        from pathlib import Path
        from ros2_hover import HoverPublisher

        node, _, _, _ = navigator_fixture()
        node.notice = None
        node.output_path = Path("unused-status.json")
        pending = []
        node.status_sink = lambda message, report, path: pending.append((message, report, path))
        with patch.object(Path, "write_text", side_effect=AssertionError("Synchronous file I/O")):
            HoverPublisher.say(node, "Queued notice")
        self.assertEqual(pending[0][0], "Queued notice")
        self.assertEqual(pending[0][1]["message"], "Queued notice")

    def test_frontier_analysis_failure_is_exposed_instead_of_a_successful_empty_catalog(self):
        ledger = AsyncFrontiers()
        self.addCleanup(ledger.close)
        ledger.worker.handler = lambda snap: (_ for _ in ()).throw(ValueError("analysis failed"))
        ledger.observe_map(snapshot())
        with ledger.worker.condition:
            self.assertTrue(ledger.worker.condition.wait_for(lambda: ledger.worker.failure, 1))
        with self.assertRaisesRegex(ValueError, "analysis failed"):
            ledger.observe_map(snapshot())

    def test_frontier_results_require_current_stamp_and_retain_real_resolution_evidence(self):
        from test_frontier_ledger import maze

        ledger = AsyncFrontiers()
        self.addCleanup(ledger.close)
        snap = maze()

        def processed(snap):
            ledger.observe_map(snap)
            with ledger.worker.condition:
                self.assertTrue(
                    ledger.worker.condition.wait_for(
                        lambda: ledger.worker.result and ledger.worker.result[0] == snap.stamp, 1
                    )
                )

        self.assertFalse(ledger.ready_for(snap))
        processed(snap)
        self.assertTrue(ledger.ready_for(snap))
        self.assertGreater(ledger.report()["pending"], 0)
        from navigation.exploration import choose_frontier

        choice = choose_frontier(snap, snap.pose[:2], catalog=ledger.snapshot())
        ledger.apply_choice(choice)
        for index in range(2, 5):
            # Each new snapshot is immutable while the worker uses it.
            closed = maze()
            closed.stamp = index
            closed.model.grid[closed.model.grid == 0] = 2
            self.assertFalse(ledger.ready_for(closed))
            processed(closed)
            if index < 4:
                self.assertEqual(ledger.report()["resolved"], 0)
        self.assertGreater(ledger.report()["resolved"], 0)

    def test_worker_close_cannot_publish_a_late_result(self):
        entered, release = threading.Event(), threading.Event()

        def work(value):
            entered.set()
            release.wait(2)
            return value

        worker = self.worker(work)
        self.addCleanup(release.set)
        worker.submit(1, "late")
        self.assertTrue(entered.wait(1))
        closer = threading.Thread(target=worker.close)
        closer.start()
        with worker.condition:
            self.assertTrue(worker.condition.wait_for(lambda: worker.stopping, 1))
        release.set()
        closer.join(1)
        self.assertFalse(closer.is_alive())
        self.assertIsNone(worker.latest()[0])

    def test_real_ros_node_constructs_and_closes_background_workers_without_flight_input(self):
        import time
        from queue import Queue
        from types import SimpleNamespace
        import rclpy
        from navigation.dashboard import Dashboard
        from navigation.timing import TrajectoryClock

        # Test launcher uses domain 97. These outputs never reach the lab's PX4.
        class IsolatedNavigator(GoalNavigator):
            def publish_notice(self, item):
                return None  # Preserve live log files during the lifecycle check.

            def publish_reports(self, item):
                return None

        feed = SimpleNamespace(
            get=lambda: (None, "No test sensor input"), trajectory_clock=TrajectoryClock()
        )
        commands = Queue()
        hub = Dashboard(feed, commands, port=0)
        node = None
        rclpy.init()
        try:
            node = IsolatedNavigator(feed, hub, commands, load_settings())
            end = time.monotonic() + 0.15
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertFalse(hub.report["ready"])
            self.assertEqual(node.ready_samples, 0)
            node.destroy_node()
            self.assertFalse(node.reporting.thread.is_alive())
            self.assertFalse(node.notices.thread.is_alive())
            self.assertFalse(node.exploration.frontier_ledger.worker.thread.is_alive())
            node = None
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
            hub.close()


if __name__ == "__main__":
    unittest.main()
