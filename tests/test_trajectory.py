"""Timebase, rounded paths and tracking governor regressions; no flight."""

import math
import unittest
import numpy as np
from navigation.timing import TrajectoryClock
from navigation.curves import rounded_route, corner_limit
from navigation.motion import RouteFollower, smooth_yaw
from navigation.settings import load_settings
from navigation.frontier_ledger import boundary_geometry


class TrajectoryTests(unittest.TestCase):
    def test_duplicate_clock_callbacks_preserve_yaw_turn_progress(self):
        yaw = rate = duplicate_yaw = duplicate_rate = 0.0
        limit, acceleration = math.radians(60), math.radians(90)
        for _ in range(100):
            duplicate_yaw, duplicate_rate = smooth_yaw(
                duplicate_yaw, duplicate_rate, math.pi, 0.0, limit, acceleration
            )
            yaw, rate = smooth_yaw(yaw, rate, math.pi, 0.05, limit, acceleration)
            duplicate_yaw, duplicate_rate = smooth_yaw(
                duplicate_yaw, duplicate_rate, math.pi, 0.05, limit, acceleration
            )
            self.assertAlmostEqual(duplicate_yaw, yaw)
            self.assertAlmostEqual(duplicate_rate, rate)
        self.assertLess(abs(math.remainder(yaw - math.pi, 2 * math.pi)), math.radians(2))

    def test_major_retarget_waits_for_measured_near_hover_then_moves_while_yaw_aligns(self):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [20, 0]])
        follower.velocity = np.array([0.6, 0.0])
        follower.speed = 0.6
        follower.retarget([[0, 0], [0, 20]])
        yaw = rate = 0.0
        for _ in range(100):
            before = follower.velocity.copy()
            follower.tick(
                0.05,
                follower.target.copy(),
                np.empty((0, 2)),
                observed_corridor=True,
                heading_error=0.0,  # Yaw can align before physical braking finishes.
                measured_speed=0.6,
            )
            self.assertLessEqual(np.linalg.norm(follower.velocity - before), 0.0125 + 1e-8)
            self.assertEqual(follower.turn_stage, "BRAKING")
            yaw, rate = smooth_yaw(
                yaw,
                rate,
                math.pi / 2,
                0.05,
                math.radians(follower.settings.yaw_rate_deg_s),
                math.radians(follower.settings.yaw_acceleration_deg_s2),
            )
        self.assertGreater(yaw, math.radians(80))
        self.assertAlmostEqual(follower.speed, 0)
        np.testing.assert_allclose(follower.velocity, [0, 0], atol=1e-9)
        # A new frontier update must not bypass an unfinished braking stage.
        follower.retarget([follower.target.tolist(), [0, 25]])
        self.assertEqual(follower.turn_stage, "BRAKING")
        target = follower.target.copy()
        follower.tick(
            0.05,
            target,
            np.empty((0, 2)),
            observed_corridor=True,
            heading_error=math.radians(70),
            measured_speed=0.05,
        )
        self.assertIsNone(follower.turn_stage)
        self.assertGreater(np.linalg.norm(follower.target - target), 0)
        self.assertIn("yaw aligns", follower.reason)
        follower.tick(
            0.05,
            target,
            np.empty((0, 2)),
            observed_corridor=True,
            heading_error=math.radians(25),
            measured_speed=0.05,
        )
        self.assertIsNone(follower.turn_stage)
        self.assertGreater(follower.speed, 0)
        self.assertLessEqual(follower.speed, 0.025 + 1e-8)

    def test_shallow_retarget_keeps_moving_without_near_hover_stage(self):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [30, 0]])
        follower.velocity = np.array([0.6, 0.0])
        follower.speed = 0.6
        follower.retarget([[0, 0], [30, 10]])
        follower.tick(
            0.05,
            [0, 0],
            np.empty((0, 2)),
            observed_corridor=True,
            heading_error=math.radians(20),
            measured_speed=0.6,
        )
        self.assertIsNone(follower.turn_stage)
        self.assertGreater(follower.speed, 0.59)

    def clock(self):
        clock = TrajectoryClock()
        for i in range(5):
            clock.observe(i * 50_000_000, i * 0.05)
        clock.consume(0.2)
        return clock

    def test_missing_intermediate_clock_packets_preserve_elapsed_time(self):
        clock = self.clock()
        clock.observe(300_000_000, 0.3)
        self.assertAlmostEqual(clock.consume(0.3), 0.1)
        self.assertEqual(clock.consume(0.3), 0)

    def test_present_time_tracking_does_not_compare_motion_to_previous_time_target(self):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [100, 0]])
        follower.velocity = np.array([3.0, 0.0])
        follower.speed = 3
        # During a 0.13 s physics batch the vehicle follows the prior velocity.
        follower.tick(0.13, [0.39, 0], np.empty((0, 2)), observed_corridor=True)
        self.assertEqual(follower.phase, "NAVIGATING")
        self.assertAlmostEqual(follower.tracking["error_m"], 0)
        self.assertAlmostEqual(follower.tracking["previous_target_error_m"], 0.39)
        self.assertLess(np.linalg.norm(follower.target - [0.39, 0]), 0.05)
        follower.tick(0.05, follower.target + [0, 0.36], np.empty((0, 2)), observed_corridor=True)
        self.assertEqual(follower.phase, "PAUSED")

    def test_telemetry_before_clock_uses_imu_prediction_without_advancing_route_twice(self):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [100, 0]])
        follower.velocity = np.array([3.0, 0.0])
        follower.speed = 3
        follower.tick(0, [0.39, 0], np.empty((0, 2)), observed_corridor=True, tracking_dt=0.13)
        self.assertEqual(follower.phase, "NAVIGATING")
        np.testing.assert_array_equal(follower.target, [0, 0])
        follower.tick(0.13, [0.39, 0], np.empty((0, 2)), observed_corridor=True)
        self.assertEqual(follower.phase, "NAVIGATING")
        self.assertLess(np.linalg.norm(follower.target - [0.39, 0]), 0.05)

    def test_paused_clock_duplicates_reset_and_large_jump_cannot_advance(self):
        for stamp in (100_000_000, 900_000_000):
            clock = self.clock()
            clock.observe(stamp, 0.3)
            self.assertIsNone(clock.consume(0.3))
            for i in range(1, 4):
                clock.observe(stamp + i * 50_000_000, 0.3 + i * 0.05)
            self.assertEqual(clock.consume(0.45), 0)
        clock = self.clock()
        clock.observe(200_000_000, 0.6)
        self.assertIsNone(clock.consume(0.6))

    def test_rounding_is_tangent_and_falls_back_when_unknown_or_margin_blocks(self):
        route = [[0, 0], [6, 0], [6, 6]]
        curve = np.asarray(rounded_route(route, lambda points, original: True))
        self.assertGreater(len(curve), 20)
        self.assertLess(abs(curve[2, 1] - curve[1, 1]), 0.01)
        self.assertTrue(np.all(curve >= 0))
        self.assertTrue(np.all(curve <= 6))
        self.assertEqual(rounded_route(route, lambda points, original: False), route)

    def test_wider_corner_arcs_reduce_curvature_and_allow_more_turn_speed(self):
        route = [[0, 0], [10, 0], [10, 10]]
        wide = [np.asarray(p) for p in rounded_route(route, lambda *_: True)]
        narrow = [np.asarray(p) for p in rounded_route(route, lambda *_: True, maximum_cut=1.5)]
        np.testing.assert_allclose(wide[1], [7, 0])
        wide_speed = min(corner_limit(wide, i, 0.25, 1.2) for i in range(1, len(wide) - 1))
        narrow_speed = min(corner_limit(narrow, i, 0.25, 1.2) for i in range(1, len(narrow) - 1))
        self.assertGreater(wide_speed, narrow_speed * 1.35)

    def test_wider_corner_retries_smaller_curve_when_clearance_rejects_it(self):
        route = [[0, 0], [10, 0], [10, 10]]
        constrained = rounded_route(route, lambda candidate, _: candidate[1][0] >= 8.5)
        narrow = rounded_route(route, lambda *_: True, maximum_cut=1.5)
        np.testing.assert_allclose(constrained, narrow)

    def test_shallow_rounded_bend_does_not_stop_at_each_sample(self):
        shallow = rounded_route([[0, 0], [12, 0], [24, 3]], lambda points, original: True)
        follower = RouteFollower(load_settings())
        follower.begin(shallow)
        speeds = []
        for _ in range(5000):
            follower.tick(
                0.02,
                follower.target.copy(),
                np.empty((0, 2)),
                observed_corridor=True,
                heading_error=math.radians(20),
            )
            if 10.6 < follower.target[0] < 13.4:
                speeds.append(follower.speed)
            if follower.phase == "SETTLING":
                break
        self.assertEqual(follower.phase, "SETTLING")
        self.assertTrue(speeds)
        self.assertGreater(min(speeds), 0.4)

    def test_governor_slows_leading_reference_but_not_a_drone_already_ahead(self):
        scales = []
        for position in (-0.2, 0.2):
            follower = RouteFollower(load_settings())
            follower.begin([[0, 0], [30, 0]])
            for _ in range(15):
                follower.tick(
                    0.05, follower.target + [position, 0], np.empty((0, 2)), observed_corridor=True
                )
            scales.append(follower.tracking_scale)
            self.assertEqual(follower.phase, "NAVIGATING")
        self.assertLess(scales[0], 0.5)
        self.assertEqual(scales[1], 1)

    def test_boundary_span_is_extent_not_number_of_samples_or_doorway_clearance(self):
        self.assertAlmostEqual(boundary_geometry([(0, x) for x in range(8)], 0.2)[0], 1.6)
        self.assertAlmostEqual(boundary_geometry([(0, x) for x in range(7)], 0.2)[0], 1.4)
        # A compact noisy patch is not a wide frontier merely because it has 16 cells.
        self.assertLess(
            boundary_geometry([(y, x) for y in range(4) for x in range(4)], 0.2)[0], 1.5
        )

    def test_rounded_turn_tracks_with_control_lag_and_bounded_vector_acceleration(self):
        path = rounded_route([[0, 0], [8, 0], [8, 8], [16, 8]], lambda points, original: True)
        follower = RouteFollower(load_settings())
        follower.begin(path)
        actual, velocity = np.zeros(2), np.zeros(2)
        peak_error = 0
        for _ in range(6000):
            old_velocity = follower.velocity.copy()
            follower.tick(0.02, actual, np.empty((0, 2)), observed_corridor=True)
            if follower.phase == "NAVIGATING":
                self.assertLessEqual(
                    np.linalg.norm(follower.velocity - old_velocity), 0.25 * 0.02 + 1e-7
                )
            desired = follower.velocity + 0.95 * (follower.target - actual)
            velocity += (desired - velocity) * 0.02 / 0.15
            actual += velocity * 0.02
            peak_error = max(peak_error, np.linalg.norm(follower.target - actual))
            self.assertIn(follower.phase, ("NAVIGATING", "SETTLING"))
            if follower.phase == "SETTLING" and np.linalg.norm(actual - path[-1]) < 0.05:
                break
        self.assertEqual(follower.phase, "SETTLING")
        self.assertLess(peak_error, 0.25)

    def test_position_and_feedforward_need_the_same_physics_time(self):
        errors = []
        for reference_dt in (0.03, 0.05):
            follower = RouteFollower(load_settings())
            follower.begin([[0, 0], [100, 0]])
            actual, velocity = np.zeros(2), np.zeros(2)
            peak_error = 0
            for _ in range(800):
                follower.tick(reference_dt, actual, np.empty((0, 2)), observed_corridor=True)
                if follower.phase == "PAUSED":
                    peak_error = max(peak_error, follower.tracking["error_m"])
                    break
                desired = follower.velocity + 0.95 * (follower.target - actual)
                velocity += (desired - velocity) * 0.05 / 0.15
                actual += velocity * 0.05
                peak_error = max(peak_error, np.linalg.norm(follower.target - actual))
            errors.append(peak_error)
        self.assertGreater(
            errors[0], 0.35, "Discarded IMU periods reproduce the ahead-of-reference fault"
        )
        self.assertLess(errors[1], 0.2, "One physics timebase keeps the model tracking")
