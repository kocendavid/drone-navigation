"""Stopping-space and route-handover regressions; no flight-control traffic."""

import math
from types import SimpleNamespace
import unittest
import numpy as np

from navigation.braking import stopping_envelope, swept_space_safe
from navigation.curves import rounded_route
from navigation.mapping import RayMap
from navigation.motion import RouteFollower, FrameAlignment
from navigation.prepared_route import remaining_safe
from navigation.settings import load_settings


class BrakingHandoverTests(unittest.TestCase):
    def test_missing_or_partial_braking_feedback_pauses_instead_of_using_another_controller(self):
        for feedback in ({}, {"measured_velocity": [0.6, 0]}, {"braking_safe": lambda _: True}):
            with self.subTest(feedback=list(feedback)):
                follower = self.moving(0.6)
                follower.retarget([[0, 0], [-20, 0]])
                follower.tick(0.05, [0, 0], np.empty((0, 2)), **feedback)
                self.assertEqual(follower.phase, "PAUSED")
                self.assertIn("measured velocity", follower.reason)
                self.assertIsNone(follower.braking_envelope)
                np.testing.assert_array_equal(follower.velocity, [0, 0])

    def tick(self, follower, dt, pose, velocity, **kwargs):
        follower.tick(
            dt,
            pose,
            np.empty((0, 2)),
            measured_velocity=velocity,
            braking_safe=kwargs.pop("braking_safe", lambda points: True),
            observed_corridor=True,
            **kwargs,
        )

    def moving(self, speed=1.3267883394438813):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [100, 0]])
        follower.speed = speed
        follower.velocity = np.array([speed, 0.0])
        return follower

    def test_recorded_clamp_case_brakes_without_abrupt_velocity_cut(self):
        follower = self.moving()
        follower.retarget([[0, 0], [-20, 2]])
        before = follower.velocity.copy()
        # Recorded reference was at the 0.28 m lead boundary. Previously the
        # late clamp changed speed by ~0.597 m/s in just 0.084 simulation s.
        self.tick(follower, 0.084, [-0.28, 0], before)
        self.assertEqual(follower.phase, "NAVIGATING")
        self.assertEqual(follower.turn_stage, "BRAKING")
        self.assertAlmostEqual(np.linalg.norm(follower.velocity - before), 0.25 * 0.084)
        np.testing.assert_allclose(follower.target / 0.084, follower.velocity)
        self.assertGreater(follower.target[0] + 0.28, follower.settings.target_lead_m)

    def test_tracking_warning_before_turn_handling_is_not_a_pause(self):
        follower = self.moving()
        follower.retarget([[0, 0], [-20, 0]])
        self.tick(follower, 0.05, [0.5, 0], [1.3, 0])
        self.assertGreater(follower.tracking["error_m"], 0.35)
        self.assertTrue(follower.tracking["tracking_warning"])
        self.assertEqual(follower.phase, "NAVIGATING")
        self.assertEqual(follower.turn_stage, "BRAKING")

    def test_lead_limit_starts_braking_instead_of_cutting_feedforward(self):
        follower = self.moving()
        before = follower.velocity.copy()
        self.tick(follower, 0.084, [-0.28, 0], before)
        self.assertEqual(follower.turn_stage, "BRAKING")
        self.assertAlmostEqual(np.linalg.norm(follower.velocity - before), 0.25 * 0.084)

    def test_reached_and_nearby_waypoints_advance_before_live_turn_check(self):
        for target in ([1.0, 0.0], [1.0, 0.01]):
            follower = self.moving(0.4)
            follower.retarget([[0, 0], [1, 0], [1, 0], [20, 0]])
            follower.target = np.array(target)
            before = follower.velocity.copy()
            self.tick(follower, 0.02, target, before, heading_error=0)
            self.assertEqual(follower.index, 3)
            self.assertEqual(follower.phase, "NAVIGATING")
            self.assertIsNone(follower.braking_envelope)
            self.assertGreater(follower.speed, 0.4)
            self.assertLessEqual(np.linalg.norm(follower.velocity - before), 0.25 * 0.02 + 1e-9)

    def test_advancing_reached_waypoint_still_brakes_for_real_turn_or_unsafe_space(self):
        for goal, safe, phase in (([1, 20], True, "NAVIGATING"), ([20, 0], False, "BLOCKED")):
            follower = self.moving(0.4)
            follower.retarget([[0, 0], [1, 0], goal])
            follower.target = np.array([1.0, 0.0])
            self.tick(follower, 0.02, [1, 0], [0.4, 0], braking_safe=lambda _: safe)
            self.assertEqual(follower.index, 2)
            self.assertEqual(follower.phase, phase)
            if safe:
                self.assertEqual(follower.turn_stage, "BRAKING")
                np.testing.assert_allclose(follower.velocity, [0.395, 0])

    def test_live_follower_traverses_dense_shallow_curve_without_braking_handover(self):
        path = rounded_route([[0, 0], [12, 0], [24, 3]], lambda *_: True)
        follower = RouteFollower(load_settings())
        follower.begin(path)
        speeds = []
        for _ in range(5000):
            self.tick(
                follower, 0.02, follower.target.copy(), follower.velocity.copy(), heading_error=0
            )
            self.assertIsNone(follower.braking_envelope)
            if 10.6 < follower.target[0] < 13.4:
                speeds.append(follower.speed)
            if follower.phase == "SETTLING":
                break
        self.assertEqual(follower.phase, "SETTLING")
        self.assertTrue(speeds)
        self.assertGreater(min(speeds), 0.4)

    def test_slow_response_and_batched_clock_reach_cruise_without_stop_replan(self):
        for response_s in (1.2, 2.0):
            with self.subTest(response_s=response_s):
                follower = self.moving(0)
                actual, velocity = np.zeros(2), np.zeros(2)
                peak_error = 0.0
                for index in range(1300):
                    dt = (0.016, 0.084, 0.036, 0.1)[index % 4]
                    before, previous = follower.velocity.copy(), follower.target.copy()
                    self.tick(follower, dt, actual, velocity)
                    self.assertEqual(follower.phase, "NAVIGATING")
                    self.assertIsNone(follower.braking_envelope)
                    self.assertLessEqual(
                        np.linalg.norm(follower.velocity - before), 0.25 * dt + 1e-9
                    )
                    np.testing.assert_allclose(
                        (follower.target - previous) / dt, follower.velocity, atol=1e-10
                    )
                    peak_error = max(peak_error, follower.tracking["error_m"])
                    desired = follower.velocity + 0.95 * (follower.target - actual)
                    velocity += (desired - velocity) * dt / response_s
                    actual += velocity * dt
                self.assertLess(peak_error, 0.23)
                self.assertAlmostEqual(follower.speed, 1.2)
                self.assertGreater(actual[0], 75)

    def test_heading_error_caps_speed_without_stopping_aligned_translation(self):
        for angle, expected in (
            (0, 1.2),
            (30, 1.2),
            (45, 0.95),  # Midpoint of the continuous 30–60 degree transition.
            (60, 0.7),
            (90, 0.7),
            (180, 0.7),
            (-179, 0.7),
        ):
            with self.subTest(angle=angle):
                follower = self.moving(1.2)
                for _ in range(60):
                    before = follower.velocity.copy()
                    self.tick(
                        follower,
                        0.05,
                        follower.target.copy(),
                        before,
                        heading_error=math.radians(angle),
                    )
                    self.assertEqual(follower.phase, "NAVIGATING")
                    self.assertIsNone(follower.braking_envelope)
                    self.assertIsNone(follower.turn_stage)
                    self.assertGreater(follower.speed, 0)
                    self.assertLessEqual(
                        np.linalg.norm(follower.velocity - before), 0.25 * 0.05 + 1e-9
                    )
                self.assertAlmostEqual(follower.speed, expected)

    def test_yaw_cap_does_not_override_obstacle_stop_or_tracking_limit(self):
        follower = self.moving(0.1)
        follower.tick(
            0.05,
            [0, 0],
            np.array([[0.6, 0]]),
            observed_corridor=True,
            heading_error=math.pi,
            measured_velocity=[0.1, 0],
            braking_safe=lambda _: True,
        )
        self.assertEqual(follower.phase, "BLOCKED")
        self.assertEqual(follower.speed, 0)
        follower = self.moving(0.1)
        self.tick(follower, 0.05, [-0.3, 0], [0.1, 0], heading_error=math.pi)
        self.assertEqual(follower.turn_stage, "BRAKING")

    def test_new_unknown_stopping_space_prevents_further_acceleration(self):
        follower = self.moving(1)
        certificates = []

        def clear(points):
            certificates.append(points.copy())
            return points[1, 0] <= 2.50001

        self.tick(follower, 0.05, [0, 0], [1, 0], braking_safe=clear)
        self.assertGreater(certificates[0][1, 0], 2.5)
        self.assertAlmostEqual(certificates[1][1, 0], 2.5)
        self.assertEqual(follower.turn_stage, "BRAKING")
        self.assertLess(follower.speed, 1)

    def test_full_stop_is_certified_not_only_next_reference_step(self):
        follower = self.moving(3)
        follower.retarget([[0, 0], [0, 25]])
        checked = []

        def clear(points):
            checked.append(points)
            return points[1, 0] < 10  # First step is clear; stopping ray is not.

        self.tick(follower, 0.05, [0, 0], [3, 0], braking_safe=clear)
        self.assertEqual(follower.phase, "BLOCKED")
        self.assertAlmostEqual(checked[0][1, 0], 19.5)
        np.testing.assert_array_equal(follower.target, [0, 0])

    def test_fast_reversal_with_response_lag_settles_then_requests_fresh_route(self):
        for angle in (math.pi / 2, math.pi):
            for response_s in (0.15, 0.30):
                follower = self.moving(3)
                goal = np.array([30 * math.cos(angle), 30 * math.sin(angle)])
                follower.retarget([[0, 0], goal.tolist()])
                actual, velocity = np.array([0.4, 0.0]), np.array([3.0, 0.0])
                model = RayMap(resolution=0.2, origin=(-40, -40), size=400)
                snap = SimpleNamespace(model=model, body_free=np.ones((400, 400), dtype=bool))
                dt = 0.02
                peak_error = 0
                for _ in range(1800):
                    previous, before = follower.target.copy(), follower.velocity.copy()
                    self.tick(
                        follower,
                        dt,
                        actual,
                        velocity,
                        heading_error=0.0,
                        braking_safe=lambda points: swept_space_safe(snap, points),
                    )
                    self.assertIn(follower.phase, ("NAVIGATING", "PLANNING"))
                    self.assertLessEqual(
                        np.linalg.norm(follower.velocity - before), 0.25 * dt + 1e-8
                    )
                    np.testing.assert_allclose(
                        (follower.target - previous) / dt, follower.velocity, atol=1e-10
                    )
                    desired = follower.velocity + 0.95 * (follower.target - actual)
                    velocity += (desired - velocity) * dt / response_s
                    actual += velocity * dt
                    peak_error = max(peak_error, np.linalg.norm(follower.target - actual))
                    if follower.phase == "PLANNING":
                        break
                self.assertEqual(follower.phase, "PLANNING")
                self.assertGreater(peak_error, 0.35)  # Deliberately exercises former latch.
                self.assertLess(np.linalg.norm(velocity), 0.15)
                self.assertGreater(follower.target[0], 17)
                self.assertAlmostEqual(follower.target[1], 0)
                self.assertAlmostEqual(follower.braking_settled_s, dt)
                np.testing.assert_array_equal(follower.goal, goal)

    def test_recorded_near_hover_requests_plan_without_dwell_or_goal_yaw_alignment(self):
        follower = self.moving(0)
        follower.retarget([[0, 0], [0, 8]])
        follower.braking_envelope = np.zeros((4, 2))
        # 20:43:58: zero reference velocity, 0.097 m/s measured and 0.147 m
        # position error. Waiting 0.5 s let noise at 0.15054 m/s restart braking.
        self.tick(follower, 0.016, [0.051, 0.138], [0, -0.097], heading_error=math.pi)
        self.assertEqual(follower.phase, "PLANNING")
        np.testing.assert_array_equal(follower.target, [0, 0])
        np.testing.assert_array_equal(follower.velocity, [0, 0])
        np.testing.assert_array_equal(follower.goal, [0, 8])

    def test_stopped_reference_waits_for_measured_motion_and_position(self):
        for pose, velocity in (([0, 0], [0.15054, 0]), ([0.151, 0], [0.09, 0])):
            follower = self.moving(0)
            follower.braking_envelope = np.zeros((4, 2))
            for _ in range(20):
                self.tick(follower, 0.05, pose, velocity, heading_error=0)
            self.assertEqual(follower.phase, "NAVIGATING")
            self.assertEqual(follower.turn_stage, "SETTLING")
            self.assertEqual(
                follower.reason, "Holding stopping point while measured motion settles"
            )
            np.testing.assert_array_equal(follower.velocity, [0, 0])

    def test_unknown_or_new_wall_during_braking_blocks_immediately(self):
        follower = self.moving()
        follower.retarget([[0, 0], [-20, 0]])
        self.tick(follower, 0.05, [0, 0], [1.3, 0])
        self.tick(follower, 0.05, follower.target, [1.3, 0], braking_safe=lambda points: False)
        self.assertEqual(follower.phase, "BLOCKED")

    def test_bad_time_velocity_or_heading_still_pause(self):
        for dt, velocity, heading in (
            (0.8, [1, 0], 0),
            (0.05, [math.nan, 0], 0),
            (0.05, [1, 0], math.nan),
        ):
            follower = self.moving()
            follower.retarget([[0, 0], [-20, 0]])
            self.tick(follower, dt, [0, 0], velocity, heading_error=heading)
            self.assertEqual(follower.phase, "PAUSED")

    def test_velocity_conversion_round_trip_has_no_position_offset(self):
        frame = FrameAlignment([20, -30], 0.3, [4, 2], 0.5, position_yaw=0.6)
        np.testing.assert_allclose(
            frame.world_velocity(frame.ned_velocity([1.3, -0.4])), [1.3, -0.4]
        )

    def test_envelope_interior_unknown_diagonal_and_outside_map_are_rejected(self):
        model = RayMap(resolution=0.2, origin=(-10, -10), size=100)
        snap = SimpleNamespace(model=model, body_free=np.ones((100, 100), dtype=bool))
        points = np.array([[-2, -2], [2, -2], [-2, 2], [2, 2]])
        self.assertTrue(swept_space_safe(snap, points))
        snap.body_free[model.cell([0, 0])] = False
        self.assertFalse(swept_space_safe(snap, points))  # Perimeter-only test would miss this.
        snap.body_free[:] = True
        diagonal = np.array([[-2, -2], [2, 2], [-2, -2], [2, 2]])
        snap.body_free[model.cell([0.1, -0.1])] = False
        self.assertFalse(swept_space_safe(snap, diagonal))
        self.assertFalse(swept_space_safe(snap, points + [10, 0]))
        self.assertFalse(swept_space_safe(snap, points * 1e10))

    def test_remaining_route_during_braking_uses_stopping_space_not_stale_shortcut(self):
        model = RayMap(resolution=0.2, origin=(-10, -10), size=100)
        snap = SimpleNamespace(model=model, body_free=np.ones((100, 100), dtype=bool))
        follower = self.moving(1)
        follower.retarget([[0, 0], [-8, 8]])
        self.tick(follower, 0.05, [0, 0], [1, 0])
        snap.body_free[model.cell([-4, 4])] = False
        self.assertTrue(remaining_safe(follower, snap, [0, 0]))
        snap.body_free[model.cell([2, 0])] = False
        self.assertFalse(remaining_safe(follower, snap, [0, 0]))

    def test_stopping_geometry_includes_measured_velocity_and_reaction_time(self):
        points = stopping_envelope(
            [0, 0], np.array([1.0, 0]), [0.4, 0.1], np.array([2.0, 0]), 0.25, 0.5
        )
        np.testing.assert_allclose(points[1], [2.5, 0])
        np.testing.assert_allclose(points[3], [9.4, 0.1])


if __name__ == "__main__":
    unittest.main()
