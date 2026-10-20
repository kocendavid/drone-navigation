"""Committed near-term exploration routes; isolated workers and flight outputs."""

import math
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from navigation.curves import speed_profile
from navigation.exploration import ExplorationChoice
from navigation.motion import RouteFollower
from navigation.prepared_route import CommittedPrefix, PreparedRoute, route_safe
from navigation.settings import load_settings
from ros2_navigate import GoalNavigator
from test_navigation import navigator_fixture, snapshot


class RoutePrefixTests(unittest.TestCase):
    def follower(self, path, speed=0.4):
        follower = RouteFollower(load_settings())
        follower.begin(path)
        follower.velocity = np.array([speed, 0.0])
        follower.speed = speed
        return follower

    def test_prefix_follows_bends_and_cuts_long_segments_at_horizon(self):
        follower = self.follower([[0, 0], [0.3, 0], [0.3, 10]])
        prefix = CommittedPrefix(follower, 2.0)
        np.testing.assert_allclose(prefix.points, [[0, 0], [0.3, 0], [0.3, 0.5]])
        self.assertAlmostEqual(prefix.length, 0.8)
        self.assertEqual(prefix.join_index, 2)
        straight = CommittedPrefix(self.follower([[0, 0], [100, 0]], 1.3), 2.0)
        np.testing.assert_allclose(straight.anchor, [2.6, 0])

    def test_slow_prefix_has_minimum_distance_and_never_extends_past_goal(self):
        for goal, distance in (([10, 0], 0.5), ([0.2, 0], 0.2)):
            prefix = CommittedPrefix(self.follower([[0, 0], goal], 0), 2)
            self.assertAlmostEqual(prefix.length, distance)
            np.testing.assert_allclose(prefix.anchor, [distance, 0])

    def test_worker_result_trims_samples_flown_while_planning(self):
        follower = self.follower([[0, 0], [0.3, 0], [0.6, 0], [5, 0]])
        prefix = CommittedPrefix(follower, 2)
        snap = snapshot()
        path = PreparedRoute(prefix.points + [[2, 2]], snap.model, follower.settings)
        original = np.array(path)
        follower.index = 3
        follower.target = np.array([0.7, 0.0])
        rebased = prefix.rebase(path, follower)
        np.testing.assert_allclose(rebased, [[0.7, 0], [0.8, 0], [2, 2]])
        np.testing.assert_array_equal(path, original)
        np.testing.assert_allclose(
            rebased.limits,
            speed_profile([np.array(p) for p in rebased], 0.25, 3),
        )

    def test_worker_result_is_discarded_after_passing_anchor_or_its_segment(self):
        follower = self.follower([[0, 0], [0.3, 0], [0.6, 0], [5, 0], [5, 2]])
        prefix = CommittedPrefix(follower, 2)
        path = PreparedRoute(prefix.points + [[2, 2]], snapshot().model, follower.settings)
        for index, target in ((3, [0.81, 0]), (4, [5, 0.1])):
            follower.index, follower.target = index, np.array(target)
            with self.assertRaisesRegex(ValueError, "passed committed prefix"):
                prefix.rebase(path, follower)

    def test_prefix_and_new_tail_are_both_checked_against_latest_map(self):
        follower = self.follower([[0, 0], [5, 0]])
        prefix = CommittedPrefix(follower, 2)
        snap = snapshot()
        path = PreparedRoute(prefix.points + [[0.8, 2]], snap.model, follower.settings)
        self.assertTrue(route_safe(snap, path))
        for point in ([0.4, 0], [0.8, 1]):
            snap.body_free[:] = True
            snap.body_free[snap.model.cell(point)] = False
            self.assertFalse(route_safe(snap, path))

    def node(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True)
        node.exploration.phase = "TRAVELLING"
        node.follower.begin(
            PreparedRoute([[2, 3], [1.6, 3], [0, 3], [0, 1]], snap.model, node.settings)
        )
        node.follower.velocity = np.array([-0.4, 0.0])
        node.follower.speed = 0.4
        node.target_goal = np.array([0, 1])
        return node, snap

    def launch(self, node, snap):
        calls = []

        def choose(snap, start, *args, **kwargs):
            calls.append(np.array(start))
            return ExplorationChoice([-2, 2], [list(start), [-2, 2]], 3, 3, 20)

        with patch("ros2_navigate.choose_frontier", side_effect=choose):
            node.launch_reconsideration(snap)
            node.reconsider_thread.join(2)
        self.assertFalse(node.reconsider_thread.is_alive())
        return calls

    def test_background_planner_starts_at_anchor_and_preserves_motion_on_application(self):
        node, snap = self.node()
        calls = self.launch(node, snap)
        np.testing.assert_allclose(calls, [[1.2, 3]])
        target, velocity, speed = (
            node.follower.target.copy(),
            node.follower.velocity.copy(),
            node.follower.speed,
        )
        node.reconsideration_tick(snap)
        np.testing.assert_allclose(node.follower.path[:3], [[2, 3], [1.6, 3], [1.2, 3]])
        np.testing.assert_array_equal(node.follower.target, target)
        np.testing.assert_array_equal(node.follower.velocity, velocity)
        self.assertEqual(node.follower.speed, speed)
        np.testing.assert_array_equal(node.target_goal, [-2, 2])
        self.assertEqual(node.exploration.target_switches, 1)

    def test_reconsideration_applies_latest_progress_and_rejects_overtaken_result(self):
        for target, accepted in (([1.5, 3], True), ([1.1, 3], False)):
            node, snap = self.node()
            self.launch(node, snap)
            node.follower.index = 2
            node.follower.target = np.array(target)
            node.reconsideration_tick(snap)
            if accepted:
                np.testing.assert_allclose(node.follower.path[:2], [target, [1.2, 3]])
                np.testing.assert_array_equal(node.target_goal, [-2, 2])
            else:
                np.testing.assert_array_equal(node.target_goal, [0, 1])
                self.assertIn("passed committed prefix", node.exploration.reconsider_reason)

    def test_new_obstacle_overrides_commitment_and_stops_existing_route(self):
        node, snap = self.node()
        self.launch(node, snap)
        snap.body_free[snap.model.cell([1.8, 3])] = False
        snap.scans += 1
        node.reconsideration_tick(snap)
        np.testing.assert_array_equal(node.target_goal, [0, 1])
        self.assertIn("no longer observed", node.exploration.reconsider_reason)
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "BLOCKED")
        self.assertEqual(node.follower.reason, "Route changed; waiting to replan")

    def test_short_horizon_does_not_force_stopping_for_aligned_new_goal(self):
        follower = self.follower([[0, 0], [10, 0]])
        prefix = CommittedPrefix(follower, 2)
        route = PreparedRoute(prefix.points + [[20, 0]], snapshot().model, follower.settings)
        follower.retarget(prefix.rebase(route, follower))
        for _ in range(10):
            follower.tick(
                0.02,
                follower.target.copy(),
                np.empty((0, 2)),
                observed_corridor=True,
                heading_error=0,
                measured_velocity=follower.velocity.copy(),
                braking_safe=lambda _: True,
            )
            self.assertIsNone(follower.braking_envelope)
            self.assertGreaterEqual(follower.speed, 0.4)

    def test_commit_horizon_settings_reject_invalid_values(self):
        for value in (0, -1, math.nan, 5.1):
            with self.assertRaises(ValueError):
                replace(load_settings(), exploration_commit_horizon_s=value).validate()


if __name__ == "__main__":
    unittest.main()
