"""Meaningful mapping, detour, persistent control and local-dashboard checks."""

import json
import math
from pathlib import Path
from queue import Queue
import sys
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from navigation.mapping import RayMap, distance_field, FREE, OCCUPIED, UNKNOWN, START_AREA
from navigation.planning import plan, segment_safe, astar, reachable_mask
from navigation.motion import RouteFollower, FrameAlignment, approach_speed
from navigation.feed import MapSnapshot
from navigation.dashboard import Dashboard
from navigation.exploration import (
    frontiers,
    visible_unknown,
    choose_frontier,
    ExplorationSession,
    ExplorationChoice,
)
from navigation.settings import NavigationSettings, load_settings
from ros2_navigate import GoalNavigator
from test_step import node_fixture


def snapshot(box=False):
    model = RayMap()
    model.grid[15:105, 15:105] = FREE
    model.grid[15:105, 15] = model.grid[15:105, 104] = OCCUPIED
    model.grid[15, 15:105] = model.grid[104, 15:105] = OCCUPIED
    if box:
        model.grid[52:68, 57:65] = OCCUPIED
    clearance, body_free, safe = model.geometry()
    theta = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    hits = np.column_stack((4.4 * np.cos(theta), 4.4 * np.sin(theta)))
    return MapSnapshot(
        model,
        clearance,
        body_free,
        safe,
        [2.0, 3.0, math.pi / 2, 0.25],
        hits,
        1000000000,
        100.0,
        100.0,
        100.0,
        True,
        "Live",
        3,
    )


class MappingTests(unittest.TestCase):
    def test_distance_transform_matches_exact_obstacle_distances(self):
        mask = np.zeros((8, 10), dtype=bool)
        mask[3, 5] = True
        padded = np.pad(mask, 1, constant_values=True)
        sites = np.argwhere(padded) - 1
        actual = distance_field(mask, 0.1)
        for y in range(8):
            for x in range(10):
                expected = max(
                    0.0,
                    float(np.linalg.norm(sites - [y, x], axis=1).min()) * 0.1 - 0.1 / math.sqrt(2),
                )
                self.assertAlmostEqual(actual[y, x], expected)

    def circle(self, radius):
        a = np.linspace(0, 2 * math.pi, 720, endpoint=False)
        return np.column_stack((radius * np.cos(a), radius * np.sin(a), np.zeros_like(a)))

    def test_rays_clear_before_hits_not_blind_area_or_occluded_space(self):
        model = RayMap()
        model.update(self.circle(4), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertEqual(model.grid[model.cell([0.2, 0])], START_AREA)
        self.assertEqual(model.grid[model.cell([1.0, 0])], FREE)
        self.assertEqual(model.grid[model.cell([4.5, 0])], UNKNOWN)
        self.assertGreater(int((model.grid == OCCUPIED).sum()), 100)

    def test_start_prior_never_erases_observed_obstacles(self):
        model = RayMap()
        model.update(self.circle(0.55), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertEqual(model.grid[model.cell([0.55, 0])], OCCUPIED)

    def test_new_valid_rays_can_clear_a_removed_obstacle(self):
        model = RayMap()
        model.update(self.circle(2), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertEqual(model.grid[model.cell([2.0, 0])], OCCUPIED)
        model.update(self.circle(4), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertEqual(model.grid[model.cell([2.0, 0])], FREE)

    def test_sparse_or_wrong_height_returns_do_not_create_free_space(self):
        for points in (self.circle(4)[:20], self.circle(4) + [0, 0, 5.0]):
            model = RayMap()
            with self.assertRaises(ValueError):
                model.update(points, np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
            self.assertTrue((model.grid == UNKNOWN).all())


class PlanningTests(unittest.TestCase):
    def test_route_detours_around_inflated_box(self):
        snap = snapshot(True)
        route = plan(snap, [-3.0, 0.0], [3.0, 0.0])
        self.assertGreater(max(abs(p[1]) for p in route), 1.5)
        self.assertTrue(
            all(
                segment_safe(snap.body_free, snap.model.cell(a), snap.model.cell(b))
                for a, b in zip(route, route[1:])
            )
        )

    def test_unknown_barrier_is_not_treated_as_free(self):
        snap = snapshot()
        snap.model.grid[:, 60] = UNKNOWN
        snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
        with self.assertRaises(ValueError):
            plan(snap, [-3.0, 0.0], [3.0, 0.0])

    def test_start_escape_increases_clearance_from_near_wall(self):
        snap = snapshot()
        route = plan(snap, [-4.0, 0.0], [2.0, 0.0])
        self.assertGreater(
            snap.clearance[snap.model.cell(route[1])], snap.clearance[snap.model.cell(route[0])]
        )
        self.assertTrue(
            all(
                segment_safe(snap.body_free, snap.model.cell(a), snap.model.cell(b))
                for a, b in zip(route, route[1:])
            )
        )

    def test_cannot_cut_diagonal_between_two_obstacles(self):
        mask = np.array([[True, False], [False, True]])
        with self.assertRaises(ValueError):
            astar(mask, (0, 0), (1, 1))

    def test_unknown_or_obstacle_margin_destination_is_rejected(self):
        snap = snapshot(True)
        for point in ([5.0, 5.0], [0.0, 0.0], [-4.0, 0.0]):
            with self.assertRaises(ValueError):
                plan(snap, [-3.0, 0.0], point)


class MotionTests(unittest.TestCase):
    def test_faster_distance_bands_slow_below_one_metre_and_stop_at_point_65(self):
        self.assertEqual(approach_speed(0.65), 0.0)
        self.assertEqual(approach_speed(None), 0.0)
        self.assertAlmostEqual(approach_speed(1.0), 0.5)
        self.assertAlmostEqual(approach_speed(3.0), 0.5)
        self.assertLess(approach_speed(0.8), approach_speed(1.0))

    def test_world_px4_rotation_and_offset_round_trip(self):
        alignment = FrameAlignment([-3.0, -0.4], 0.1, [2.0, 3.0], 1.6)
        np.testing.assert_allclose(alignment.world([2.0, 3.0]), [-3.0, -0.4])
        for point in ([0.0, 0.0], [3.0, -2.0], [-2.0, 4.0]):
            np.testing.assert_allclose(alignment.world(alignment.ned(point)), point, atol=1e-12)

    def test_estimator_heading_bias_does_not_rotate_gps_position_axes(self):
        alignment = FrameAlignment([-3.0, -0.4], 0.2, [2.0, 3.0], 1.6)
        np.testing.assert_allclose(alignment.world([3.0, 3.0]), [-3.0, 0.6], atol=1e-12)
        np.testing.assert_allclose(alignment.world([2.0, 4.0]), [-2.0, -0.4], atol=1e-12)

    def test_reference_uses_simulated_elapsed_time_and_stops_for_obstacle(self):
        follower = RouteFollower()
        follower.begin([[0.0, 0.0], [2.0, 0.0]])
        for _ in range(10):
            follower.tick(0.03, [0.0, 0.0], np.array([[4.0, 0.0]]))
        self.assertLess(follower.target[0], 0.02)
        follower.tick(0.03, [0.0, 0.0], np.array([[0.6, 0.0]]))
        self.assertEqual(follower.phase, "BLOCKED")
        np.testing.assert_array_equal(follower.target, [0.0, 0.0])

    def test_tracking_error_and_clock_gap_pause_without_target_jump(self):
        for dt, pose in ((0.8, [0.0, 0.0]), (0.1, [0.4, 0.0])):
            follower = RouteFollower()
            follower.begin([[0.0, 0.0], [2.0, 0.0]])
            follower.tick(dt, pose, np.array([[4.0, 0.0]]))
            self.assertEqual(follower.phase, "PAUSED")
            np.testing.assert_array_equal(follower.target, pose)

    def test_goal_already_here_settles_without_moving(self):
        follower = RouteFollower()
        follower.begin([[0.0, 0.0], [0.0, 0.0]])
        follower.tick(0.03, [0.0, 0.0], np.empty((0, 2)))
        self.assertEqual(follower.phase, "SETTLING")

    def test_repeated_goals_do_not_require_new_program(self):
        follower = RouteFollower()
        for point in ([2.0, 0.0], [0.0, 2.0], [-2.0, 0.0]):
            follower.begin([[0.0, 0.0], point])
            self.assertEqual(follower.phase, "NAVIGATING")
            np.testing.assert_array_equal(follower.goal, point)
            follower.hold([0.0, 0.0])

    def test_whole_detour_completes_with_first_order_control_lag(self):
        snap = snapshot(True)
        start = [-3.0, 0.0]
        goal = [3.0, 0.0]
        route = plan(snap, start, goal)
        occupied = np.argwhere(snap.model.grid == OCCUPIED)
        hits = np.array([snap.model.world(cell) for cell in occupied])
        follower = RouteFollower()
        follower.begin(route)
        actual = np.array(start, dtype=float)
        minimum = math.inf
        for _ in range(12000):
            follower.tick(0.02, actual, hits)
            actual += (follower.target - actual) * 0.02  # One-second response lag.
            minimum = min(minimum, float(np.linalg.norm(hits - actual, axis=1).min()))
            if follower.phase == "SETTLING" and np.linalg.norm(actual - goal) < 0.1:
                break
            self.assertIn(follower.phase, ("NAVIGATING", "SETTLING"))
        self.assertEqual(follower.phase, "SETTLING")
        self.assertLess(float(np.linalg.norm(actual - goal)), 0.1)
        self.assertGreater(minimum, 0.9)


def navigator_fixture():
    node, modes, targets, notices = node_fixture()
    snap = snapshot()
    node.feed = SimpleNamespace(get=lambda: (snap, None))
    node.dashboard = SimpleNamespace(note="", report={})
    node.status_wall = 100.0
    node.commands = Queue()
    node.alignment = FrameAlignment([2.0, 3.0], math.pi / 2, [2.0, 3.0], 0.0)
    node.world_pose = np.array([2.0, 3.0])
    node.reference = np.array([2.0, 3.0])
    node.hold_yaw = 0.0
    node.follower = RouteFollower()
    node.follower.begin([[2.0, 3.0], [2.0, 2.0]])
    node.target_goal = np.array([2.0, 2.0])
    node.pacer_mark = 100.0
    node.last_callback = 99.9
    node.last_retry = 100.0
    node.last_map_scans = 3
    node.plan_results = Queue()
    node.plan_generation = 0
    node.plan_thread = None
    node.last_report = 100.0
    node.exploration = ExplorationSession()
    node.settings = NavigationSettings()
    for name in (
        "flight_active",
        "stable",
        "stop",
        "commands_tick",
        "launch_plan",
        "launch_frontier",
        "exploration_tick",
    ):
        setattr(node, name, MethodType(getattr(GoalNavigator, name), node))
    node.update_dashboard = lambda *args: None
    return node, modes, targets, notices


class RuntimeTests(unittest.TestCase):
    def test_map_failure_pauses_goal_but_retains_receipt_stamped_hover(self):
        node, modes, targets, _ = navigator_fixture()
        node.feed.get = lambda: (None, "Map worker exited")
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertEqual(modes[-1].timestamp, 0)
        self.assertEqual(list(targets[-1].position), [2.0, 3.0, -1.0])

    def test_coordinate_reset_stops_all_targets(self):
        node, modes, targets, _ = navigator_fixture()
        node.state.fault = "Origin reset"
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(modes, [])
        self.assertEqual(targets, [])

    def test_native_hold_cancels_goal_and_recaptures_hover(self):
        node, modes, targets, _ = navigator_fixture()
        node.status.nav_state = 4
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertIsNone(node.target_goal)
        self.assertEqual(node.follower.phase, "HOLD")
        self.assertEqual(list(targets[-1].position), [2.0, 3.0, -1.0])

    def test_competing_publisher_emits_no_flight_output(self):
        node, modes, targets, _ = navigator_fixture()
        node.count_publishers = lambda _: 2
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(modes, [])
        self.assertEqual(targets, [])

    def test_delayed_callback_holds_without_advancing_goal(self):
        node, modes, targets, _ = navigator_fixture()
        node.last_callback = 99.0
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertEqual(list(targets[-1].position), [2.0, 3.0, -1.0])

    def test_invalid_velocity_pauses_motion_without_killing_heartbeat(self):
        node, modes, targets, _ = navigator_fixture()
        node.position.v_xy_valid = False
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertEqual(modes[-1].timestamp, 0)


class DashboardTests(unittest.TestCase):
    def test_invalid_payloads_return_json_errors_without_queueing_flight_commands(self):
        snap = snapshot()
        commands = Queue()
        hub = Dashboard(SimpleNamespace(get=lambda: (snap, None)), commands, port=0)
        try:
            with patch("navigation.dashboard.time.monotonic", return_value=100.0):
                for endpoint in ("preview", "command"):
                    for payload in (
                        [],
                        None,
                        {"action": "goal", "point": [True, 2]},
                        {"action": "goal", "point": ["2", 2]},
                        {"action": "goal", "point": [float("inf"), 2]},
                    ):
                        request = Request(
                            hub.url + "/api/" + endpoint,
                            json.dumps(payload).encode(),
                            headers={
                                "Content-Type": "application/json",
                                "X-Navigation-Token": hub.token,
                            },
                        )
                        with self.assertRaises(HTTPError) as failure:
                            urlopen(request)
                        self.assertEqual(failure.exception.code, 400)
                        self.assertIn("error", json.loads(failure.exception.read()))
                self.assertTrue(commands.empty())
        finally:
            hub.close()

    def test_observer_previews_route_but_rejects_flight_commands_and_cross_site_posts(self):
        snap = snapshot()
        hub = Dashboard(SimpleNamespace(get=lambda: (snap, None)), port=0)
        try:
            with patch("navigation.dashboard.time.monotonic", return_value=100.0):
                request = Request(
                    hub.url + "/api/preview",
                    json.dumps(dict(point=[2.0, 2.0])).encode(),
                    headers={"Content-Type": "application/json", "X-Navigation-Token": hub.token},
                )
                result = json.loads(urlopen(request).read())
                self.assertGreaterEqual(len(result["path"]), 2)
                for headers in (
                    {"Content-Type": "application/json", "X-Navigation-Token": hub.token},
                    {
                        "Content-Type": "application/json",
                        "X-Navigation-Token": hub.token,
                        "Origin": "http://outside.invalid",
                    },
                ):
                    for action in ("goal", "explore"):
                        request = Request(
                            hub.url + "/api/command",
                            json.dumps(dict(action=action, point=[2.0, 2.0])).encode(),
                            headers=headers,
                        )
                        with self.assertRaises(HTTPError):
                            urlopen(request)
        finally:
            hub.close()


def frontier_snapshot():
    snap = snapshot()
    snap.model.grid[:, 60:] = UNKNOWN
    snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
    snap.pose = [-2.0, 0.0, 0.0, 0.25]
    return snap


class ExplorationTests(unittest.TestCase):
    def test_frontiers_only_use_observed_free_not_start_prior_or_obstacles(self):
        grid = np.array([[OCCUPIED, FREE, UNKNOWN], [FREE, START_AREA, UNKNOWN]], dtype=np.uint8)
        edge = frontiers(grid)
        self.assertTrue(edge[0, 1])
        self.assertFalse(edge[1, 1])
        self.assertFalse(edge[0, 0])

    def test_selects_reachable_known_viewpoint_instead_of_unknown_destination(self):
        snap = frontier_snapshot()
        choice = choose_frontier(snap, [-2.0, 0.0])
        self.assertTrue(snap.safe[snap.model.cell(choice.goal)])
        self.assertGreater(choice.expected_new_area_m2, 0.12)
        self.assertLess(choice.goal[0], 0.0)
        self.assertTrue(
            all(
                segment_safe(snap.body_free, snap.model.cell(a), snap.model.cell(b))
                for a, b in zip(choice.path, choice.path[1:])
            )
        )

    def test_information_estimate_does_not_see_through_known_closed_walls(self):
        grid = np.full((30, 30), UNKNOWN, dtype=np.uint8)
        grid[10:20, 10:20] = FREE
        grid[9, 9:21] = grid[20, 9:21] = grid[9:21, 9] = grid[9:21, 20] = OCCUPIED
        self.assertEqual(visible_unknown(grid, (15, 15), 0.1), 0.0)

    def test_fully_known_room_finishes_without_inventing_a_goal(self):
        snap = snapshot()
        snap.model.grid[snap.model.grid == UNKNOWN] = OCCUPIED
        with self.assertRaisesRegex(ValueError, "No observed"):
            choose_frontier(snap, [-2.0, 0.0])

    def test_visited_viewpoints_are_not_repeated(self):
        snap = frontier_snapshot()
        first = choose_frontier(snap, [-2.0, 0.0])
        second = choose_frontier(snap, [-2.0, 0.0], [first.goal])
        self.assertGreaterEqual(math.dist(first.goal, second.goal), 0.5)

    def test_viewpoint_reachability_excludes_disconnected_room(self):
        snap = snapshot()
        snap.model.grid[15:105, 60] = OCCUPIED
        snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
        reachable = reachable_mask(snap, [-3.0, 0.0])
        self.assertTrue(reachable[snap.model.cell([-2.0, 0.0])])
        self.assertFalse(reachable[snap.model.cell([2.0, 0.0])])

    def test_stale_or_repeated_scan_does_not_launch_another_goal(self):
        snap = frontier_snapshot()
        session = ExplorationSession()
        session.start(snap)
        session.arrived(snap, [-0.5, 0.0])
        self.assertFalse(session.scanned(snap))
        self.assertEqual(session.phase, "SCANNING")
        snap.model.grid[30:40, 60:70] = FREE
        snap.stamp += 1
        self.assertTrue(session.scanned(snap))
        self.assertAlmostEqual(session.area_gain_m2, 1.0)

    def test_stalled_exploration_stops_after_three_viewpoints(self):
        snap = frontier_snapshot()
        session = ExplorationSession()
        session.start(snap)
        for i in range(3):
            session.arrived(snap, [i, 0.0])
            snap.stamp += 1
            session.scanned(snap)
        self.assertFalse(session.enabled)
        self.assertEqual(session.phase, "FINISHED")

    def test_hold_cancels_exploration_and_stale_results(self):
        node, modes, targets, _ = navigator_fixture()
        node.exploration.start(node.feed.get()[0])
        old_generation = node.plan_generation
        node.commands.put(dict(action="hold"))
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertFalse(node.exploration.enabled)
        self.assertIsNone(node.target_goal)
        self.assertGreater(node.plan_generation, old_generation)
        self.assertEqual(node.follower.phase, "HOLD")

    def test_map_loss_pauses_exploration_without_automatic_restart(self):
        node, modes, targets, _ = navigator_fixture()
        node.exploration.start(node.feed.get()[0])
        node.feed.get = lambda: (None, "Lost map")
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.exploration.phase, "PAUSED")
        self.assertTrue(node.exploration.enabled)
        self.assertEqual(modes[-1].timestamp, 0)

    def test_settings_enforce_sensor_stop_margin(self):
        self.assertEqual(load_settings().cruise_m_s, 0.5)
        with self.assertRaises(ValueError):
            NavigationSettings(stop_distance_m=0.5).validate()

    def test_controller_arrival_waits_for_new_scan_then_selects_next_viewpoint(self):
        node, modes, targets, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap)
        node.target_goal = None
        path = [[2.0, 3.0], [2.0, 3.0]]
        choice = ExplorationChoice([2.0, 3.0], path, 1.0, 0.0, 10)
        node.plan_results.put((node.plan_generation, path, None, choice))
        launches = []
        node.launch_frontier = lambda item: launches.append(item.stamp)
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
            self.assertEqual(node.follower.phase, "SETTLING")
            node.pacer.total_s += 1.1
            GoalNavigator.publish_target(node)
            self.assertEqual(node.exploration.phase, "SCANNING")
            self.assertEqual(node.exploration.reached, 1)
            GoalNavigator.publish_target(node)
            self.assertEqual(launches, [])
            snap.stamp += 1
            GoalNavigator.publish_target(node)
        self.assertEqual(launches, [snap.stamp])
        self.assertEqual(node.exploration.phase, "SELECTING")
        self.assertTrue(modes and targets)

    def test_no_reachable_viewpoint_finishes_while_retaining_hover_stream(self):
        node, modes, targets, _ = navigator_fixture()
        node.exploration.start(node.feed.get()[0])
        node.target_goal = None
        node.plan_results.put((node.plan_generation, None, "No useful viewpoint", "exploration"))
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertFalse(node.exploration.enabled)
        self.assertEqual(node.exploration.phase, "FINISHED")
        self.assertEqual(node.follower.phase, "HOLD")
        self.assertEqual(modes[-1].timestamp, 0)

    def test_hold_discards_completed_frontier_selection_result(self):
        node, modes, targets, _ = navigator_fixture()
        node.exploration.start(node.feed.get()[0])
        choice = ExplorationChoice([3.0, 3.0], [[2.0, 3.0], [3.0, 3.0]], 1.0, 1.0, 10)
        node.plan_results.put((node.plan_generation, choice.path, None, choice))
        node.commands.put(dict(action="hold"))
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertIsNone(node.target_goal)
        self.assertEqual(node.follower.phase, "HOLD")
        self.assertFalse(node.exploration.enabled)

    def test_faster_reference_waits_for_slow_tracking_instead_of_running_away(self):
        follower = RouteFollower()
        follower.begin([[0.0, 0.0], [3.0, 0.0]])
        actual = np.zeros(2)
        peak_lead = 0.0
        for _ in range(1600):
            follower.tick(0.02, actual, np.array([[6.0, 0.0]]))
            peak_lead = max(peak_lead, float(np.linalg.norm(follower.target - actual)))
            actual += (follower.target - actual) * 0.02 / 1.5
            self.assertIn(follower.phase, ("NAVIGATING", "SETTLING"))
            if follower.phase == "SETTLING" and np.linalg.norm(actual - [3.0, 0.0]) < 0.1:
                break
        self.assertLessEqual(peak_lead, 0.280001)
        self.assertEqual(follower.phase, "SETTLING")

    def test_open_space_reference_can_reach_half_metre_per_second(self):
        follower = RouteFollower()
        follower.begin([[0.0, 0.0], [3.0, 0.0]])
        actual = np.zeros(2)
        for _ in range(100):
            follower.tick(0.03, actual, np.array([[6.0, 0.0]]))
            actual = follower.target.copy()
        self.assertAlmostEqual(follower.speed, 0.5)


if __name__ == "__main__":
    unittest.main()
