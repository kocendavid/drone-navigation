"""Motion-aware routing and turn limits; entirely offline, no ROS or flight."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from navigation.curves import corner_limit, flight_metrics, rounded_route, tangent_tail
from navigation.mapping import RayMap, FREE, OCCUPIED
from navigation.motion import RouteFollower, yaw_speed_limit
from navigation.exploration import ExplorationChoice
from navigation.prepared_route import CommittedPrefix, route_safe
from navigation.planning_jobs import PlanningJobs
from navigation.planning import curve_acceptance, motion_score, plan, route_cost, segment_safe
from navigation.settings import NavigationSettings


def room():
    model = RayMap(resolution=0.1, size=240, origin=(-12, -12), clearance=0.8)
    model.grid[:] = FREE
    model.grid[[0, -1], :] = OCCUPIED
    model.grid[:, [0, -1]] = OCCUPIED
    clearance, body_free, safe = model.geometry()
    return SimpleNamespace(model=model, clearance=clearance, body_free=body_free, safe=safe)


class TurnPlanningTests(unittest.TestCase):
    def test_yaw_cap_is_continuous_and_does_not_stop_translation(self):
        values = [
            yaw_speed_limit(math.radians(d), 1.2, 0.7)
            for d in (29.9, 30, 30.1, 40, 50, 60, 90, 180)
        ]
        self.assertAlmostEqual(values[0], 1.2)
        self.assertLess(values[1] - values[2], 0.0001)
        self.assertTrue(all(a >= b >= 0.7 for a, b in zip(values, values[1:])))
        self.assertAlmostEqual(values[-1], 0.7)
        self.assertAlmostEqual(yaw_speed_limit(math.pi, 0.5, 0.7), 0.5)

    def test_reconsider_worker_blends_tail_and_can_rebase_after_progress(self):
        snap = room()
        cfg = NavigationSettings(cruise_m_s=1.2)
        follower = RouteFollower(cfg)
        follower.begin([[-4, 0], [8, 0]])
        follower.speed = 1
        follower.velocity = np.array([1.0, 0.0])
        prefix = CommittedPrefix(follower, 2)
        choice = ExplorationChoice([4, 5], [prefix.anchor.tolist(), [4, 5]], 1, 8, 10)
        data = dict(
            token=1,
            snap=snap,
            settings=cfg,
            prefix=prefix,
            current=[8, 0],
            visited=[],
            trail=[],
            catalog=[],
            active_id=None,
            current_route=follower.path,
        )
        cost = np.ones_like(snap.safe, dtype=float)
        with (
            patch("navigation.planning_jobs.choose_frontier", return_value=choice),
            patch("navigation.planning_jobs.routing_geometry", return_value=(None, None, cost)),
        ):
            result = PlanningJobs()(("reconsider", data))
        self.assertIsNone(result[-1])
        path = result[3].path
        np.testing.assert_equal(path[: len(prefix.points)], prefix.points)
        self.assertGreater(len(path), 10)
        self.assertTrue(route_safe(snap, path))
        follower.target = np.array([-3.0, 0.0])
        rebased = prefix.rebase(path, follower)
        np.testing.assert_equal(rebased[0], follower.target)
        np.testing.assert_equal(rebased[1], prefix.anchor)
        self.assertTrue(route_safe(snap, rebased))

    def test_corner_limit_has_no_twelve_degree_cliff_and_sharp_turns_still_slow(self):
        speeds = []
        for degrees in (5, 11.9, 12, 12.1, 15, 45, 90, 179.9):
            theta = math.radians(degrees)
            path = np.array([[-10, 0], [0, 0], [10 * math.cos(theta), 10 * math.sin(theta)]])
            speeds.append(corner_limit(path, 1, 0.25, 1.2))
        self.assertTrue(all(a >= b > 0 for a, b in zip(speeds, speeds[1:])))
        self.assertLess(speeds[1] - speeds[3], 0.02)
        self.assertLess(speeds[-2], 0.15)
        self.assertLess(speeds[-1], 0.002)

    def test_total_turn_angle_does_not_limit_a_wide_sampled_arc(self):
        for spacing_deg in (0.5, 1, 2):
            theta = np.linspace(0, math.pi / 2, round(90 / spacing_deg) + 1)
            path = np.column_stack((10 * np.cos(theta), 10 * np.sin(theta)))
            speed = min(corner_limit(path, i, 0.25, 1.2) for i in range(1, len(path) - 1))
            self.assertAlmostEqual(speed, 1.2, places=6)

    def test_dense_geometry_keeps_speed_across_control_step_sizes(self):
        for dt in (0.02, 0.05, 0.084, 0.1, 0.13):
            with self.subTest(dt=dt):
                follower = RouteFollower(NavigationSettings(cruise_m_s=1.2))
                follower.begin([[i * 0.08, 0] for i in range(1001)])
                follower.speed = 1.2
                follower.velocity = np.array([1.2, 0.0])
                for _ in range(50):
                    before = follower.target.copy()
                    follower.tick(dt, before, np.empty((0, 2)), observed_corridor=True)
                    self.assertAlmostEqual(follower.speed, 1.2, places=8)
                    self.assertAlmostEqual(np.linalg.norm(follower.target - before), 1.2 * dt)
                    self.assertIsNone(follower.turn_stage)

    def test_rounded_bend_keeps_motion_and_vector_acceleration_across_step_sizes(self):
        path = rounded_route([[-10, 0], [0, 0], [0, 10]], lambda *_: True)
        for dt in (0.02, 0.084, 0.13):
            with self.subTest(dt=dt):
                follower = RouteFollower(NavigationSettings(cruise_m_s=1.2))
                follower.begin(path)
                bend_speeds = []
                for _ in range(math.ceil(80 / dt)):
                    before = follower.velocity.copy()
                    follower.tick(
                        dt, follower.target.copy(), np.empty((0, 2)), observed_corridor=True
                    )
                    if follower.phase == "SETTLING":
                        break
                    self.assertIsNone(follower.turn_stage)
                    self.assertLessEqual(
                        np.linalg.norm(follower.velocity - before), 0.25 * dt + 1e-8
                    )
                    if -2 < follower.target[0] < -0.2 and follower.target[1] > 0.2:
                        bend_speeds.append(follower.speed)
                self.assertEqual(follower.phase, "SETTLING")
                self.assertTrue(bend_speeds)
                self.assertGreater(min(bend_speeds), 0.45)

    def test_forward_tail_is_tangent_and_preserves_prefix_and_goal(self):
        prefix = [[-3, 0], [0, 0]]
        tail = [[0, 0], [3, 3], [7, 3]]
        saved = [p[:] for p in prefix]
        blend = tangent_tail(prefix, tail, lambda *_: True, minimum_radius=3)
        np.testing.assert_equal(prefix, saved)
        np.testing.assert_equal(blend[0], tail[0])
        np.testing.assert_equal(blend[-1], tail[-1])
        direction = np.asarray(blend[1]) - blend[0]
        self.assertGreater(direction[0], 0)
        self.assertLess(abs(direction[1] / direction[0]), 0.03)
        original = np.asarray(prefix + tail[1:])
        smoothed = np.asarray(prefix + blend[1:])
        self.assertGreater(
            corner_limit(smoothed, 1, 0.25, 1.2), corner_limit(original, 1, 0.25, 1.2) * 2
        )

    def test_tail_rejects_unobserved_or_unsafe_blends(self):
        snap = room()
        prefix, tail = [[-3, 0], [0, 0]], [[0, 0], [0, 4]]
        cost = np.ones_like(snap.safe, dtype=float)
        snap.safe[:, snap.model.cell([0.1, 0])[1] :] = False
        blend = tangent_tail(prefix, tail, curve_acceptance(snap, cost), 3)
        self.assertEqual(blend, tail)
        self.assertEqual(tangent_tail(prefix, tail, lambda *_: False, 3), tail)

    def test_route_score_prefers_wide_curve_over_stop_and_turn(self):
        snap = room()
        cfg = NavigationSettings(cruise_m_s=1.2)
        cost = np.ones_like(snap.safe, dtype=float)
        sharp = [[-8, 0], [0, 0], [0, 8]]
        rounded = rounded_route(sharp, lambda *_: True)
        self.assertLess(
            motion_score(snap, rounded, cost, cfg), motion_score(snap, sharp, cost, cfg)
        )

    def test_physical_cost_and_time_are_invariant_to_straight_sample_density(self):
        snap = room()
        cost = np.full_like(snap.safe, 2.0, dtype=float)
        for samples in (2, 21, 501):
            path = np.linspace([-10, 0], [10, 0], samples)
            self.assertAlmostEqual(route_cost(snap, path, cost), 40, places=6)
            # Rest-to-rest: 4.8 m acceleration + 4.8 m braking at 0.15 m/s².
            self.assertAlmostEqual(
                flight_metrics(path, 0.25, 1.2)["seconds"], 2 * 8 + (20 - 9.6) / 1.2, places=6
            )

    def test_faster_cruise_requests_wider_curves_without_cutting_obstacles(self):
        route = [[-10, 0], [0, 0], [0, 10]]
        narrow = rounded_route(route, lambda *_: True, minimum_radius=1)
        wide = rounded_route(route, lambda *_: True, minimum_radius=10)
        self.assertLess(wide[1][0], narrow[1][0])
        snap = room()
        snap.model.grid[110:130, 110:130] = OCCUPIED
        snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
        for cruise in (0.5, 1.2):
            path = plan(snap, [-8, 0], [8, 0], NavigationSettings(cruise_m_s=cruise))
            self.assertEqual(path[0], [-8, 0])
            self.assertEqual(path[-1], [8, 0])
            self.assertTrue(
                all(
                    segment_safe(snap.safe, snap.model.cell(a), snap.model.cell(b))
                    for a, b in zip(path, path[1:])
                )
            )


if __name__ == "__main__":
    unittest.main()
