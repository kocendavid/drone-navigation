"""Meaningful mapping, detour, persistent control and local-dashboard checks."""

import json
from collections import deque
from io import StringIO
import math
from pathlib import Path
from queue import Queue
import sys
import threading
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
from navigation.mapping import RayMap, distance_field, FREE, OCCUPIED, UNKNOWN, START_AREA
from navigation.planning import plan, segment_safe, astar, reachable_mask, route_distances
from navigation.centering import routing_geometry
from navigation.curves import speed_profile
from navigation.timing import TrajectoryClock
from navigation.source_time import ReferenceHistory
from navigation.motion import (
    RouteFollower,
    FrameAlignment,
    approach_speed,
    smooth_yaw,
    braking_speed,
)
from navigation.feed import MapSnapshot, MapFeed
from navigation.dashboard import Dashboard
from navigation.exploration import (
    frontiers,
    visible_unknown,
    choose_frontier,
    ExplorationSession,
    ExplorationChoice,
    wall_endings,
)
from navigation.settings import NavigationSettings, load_settings
from navigation.mission import ExploreMission, assess_completion
from navigation.ros_map import map_message
from px4_msgs.msg import VehicleStatus, VehicleCommand
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
    def test_center_distance_uses_only_real_ridge_sites_and_matches_reference(self):
        blocked = np.zeros((25, 40), dtype=bool)
        blocked[12, 20] = True
        y, x = np.indices(blocked.shape)
        expected = np.maximum(0, np.hypot(y - 12, x - 20) * 0.2 - 0.2 / math.sqrt(2))
        np.testing.assert_allclose(distance_field(blocked, 0.2, exterior_blocked=False), expected)
        with patch("navigation.mapping._edt", None):
            np.testing.assert_allclose(
                distance_field(blocked, 0.2, exterior_blocked=False), expected
            )
        self.assertTrue(
            np.isinf(distance_field(np.zeros((5, 5), bool), 0.2, exterior_blocked=False)).all()
        )

    def test_chunked_large_map_distance_field_matches_exact_linear_reference(self):
        rng = np.random.default_rng(42)
        blocked = rng.random((70, 130)) < 0.08
        np.testing.assert_allclose(
            distance_field(blocked, 0.2),
            distance_field(blocked, 0.2, vectorized=False),
            atol=1e-12,
        )

    def test_translated_rectangular_maze_observations_keep_world_coordinates(self):
        settings = load_settings()
        model = RayMap(**settings.map_options())
        body = np.array([-28.0, -17.0])
        z = settings.floor_z_m + 1.0
        theta = np.linspace(0, 2 * math.pi, 720, endpoint=False)
        points = np.column_stack((3 * np.cos(theta), 3 * np.sin(theta), np.zeros(720)))
        model.update(points, np.eye(3), np.array([*body, z]), body, z, 0.5)
        self.assertEqual(model.grid.shape, (265, 320))
        self.assertTrue(model.inside(model.cell(body)))
        self.assertFalse(model.inside(model.cell([31.0, -17.0])))
        cell = model.cell(body + [2.0, 0.0])
        self.assertEqual(model.grid[cell], FREE)
        self.assertLess(math.dist(model.world(cell), body + [2.0, 0.0]), 0.3)

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

    def circle(self, radius, phase=0.0):
        a = np.linspace(0, 2 * math.pi, 720, endpoint=False) + phase
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

    def test_distant_returns_clear_to_horizon_without_inventing_a_wall(self):
        model = RayMap(resolution=0.2, size=160, origin=(-16, -16))
        hits = model.update(
            self.circle(15, math.radians(0.25)), np.eye(3), np.zeros(3), [0, 0], 0.0
        )
        self.assertEqual(len(hits), 720)
        np.testing.assert_allclose(np.linalg.norm(hits, axis=1), 15)
        for angle in np.linspace(0, 2 * math.pi, 36, endpoint=False):
            xy = 10 * np.array([math.cos(angle), math.sin(angle)])
            self.assertEqual(model.grid[model.cell(xy)], FREE)
        self.assertEqual(model.grid[model.cell([12.5, 0])], UNKNOWN)
        self.assertEqual(model.grid[model.cell([15, 0])], UNKNOWN)
        self.assertFalse((model.grid == OCCUPIED).any())

    def test_nearest_return_blocks_distant_free_ray_in_same_angular_bin(self):
        model = RayMap(resolution=0.2, size=160, origin=(-16, -16))
        near = self.circle(3, math.radians(0.25))
        points = np.vstack((self.circle(15, math.radians(0.25)), near))
        hits = model.update(points, np.eye(3), np.zeros(3), [0, 0], 0.0)
        self.assertAlmostEqual(np.linalg.norm(hits, axis=1).max(), 3)
        self.assertEqual(model.grid[model.cell([2, 0])], FREE)
        self.assertEqual(model.grid[model.cell(near[0, :2])], OCCUPIED)
        self.assertEqual(model.grid[model.cell([5, 0])], UNKNOWN)

    def test_missing_sector_stays_unknown_despite_distant_returns_elsewhere(self):
        model = RayMap(resolution=0.2, size=160, origin=(-16, -16))
        points = self.circle(15)
        angles = np.arctan2(points[:, 1], points[:, 0])
        points = points[np.abs(angles) > math.radians(20)]
        model.update(points, np.eye(3), np.zeros(3), [0, 0], 0.0)
        self.assertEqual(model.grid[model.cell([8, 0])], UNKNOWN)
        self.assertEqual(model.grid[model.cell([-8, 0])], FREE)

    def test_beyond_sensor_far_range_is_not_evidence_of_free_space(self):
        model = RayMap()
        with self.assertRaises(ValueError):
            model.update(self.circle(201), np.eye(3), np.zeros(3), [0, 0], 0.0)
        self.assertTrue((model.grid == UNKNOWN).all())

    def test_new_valid_rays_can_clear_an_unconfirmed_single_scan_obstacle(self):
        model = RayMap()
        model.update(self.circle(2), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertEqual(model.grid[model.cell([2.0, 0])], OCCUPIED)
        model.update(self.circle(4), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertEqual(model.grid[model.cell([2.0, 0])], FREE)

    def test_confirmed_walls_survive_free_rays_and_a_new_map_clears_confirmation(self):
        model = RayMap()
        for _ in range(3):
            model.update(self.circle(2), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        cell = model.cell([2.0, 0])
        self.assertTrue(model.confirmed_walls[cell])
        before = model.confirmed_walls.copy()
        for _ in range(5):
            model.update(self.circle(4), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
            self.assertTrue((model.grid[before] == OCCUPIED).all())
        _, body_free, safe = model.geometry()
        self.assertFalse(body_free[cell])
        self.assertFalse(safe[cell])
        reset = RayMap()
        self.assertFalse(reset.confirmed_walls.any())
        self.assertFalse(reset.wall_hit_scans.any())

    def test_dense_hits_in_one_scan_block_immediately_but_are_not_confirmed(self):
        model = RayMap()
        model.update(self.circle(2), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertFalse(model.confirmed_walls.any())
        self.assertEqual(int(model.wall_hit_scans.max()), 1)
        self.assertGreater(model.wall_policy()["tentative_cells"], 0)
        self.assertGreater(int((model.grid == OCCUPIED).sum()), 100)

    def test_conflicting_free_observation_resets_tentative_confirmation(self):
        model = RayMap()
        for radius in (2, 2, 4, 2, 2):
            model.update(self.circle(radius), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        cell = model.cell([2.0, 0])
        self.assertFalse(model.confirmed_walls[cell])
        self.assertEqual(model.wall_hit_scans[cell], 2)
        model.update(self.circle(2), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        self.assertTrue(model.confirmed_walls[cell])

    def test_invalid_scan_does_not_add_confirmation_evidence(self):
        model = RayMap()
        model.update(self.circle(2), np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        before = model.wall_hit_scans.copy()
        with self.assertRaises(ValueError):
            model.update(self.circle(2)[:20], np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
        np.testing.assert_array_equal(model.wall_hit_scans, before)
        self.assertEqual(model.scans, 1)

    def test_sparse_or_wrong_height_returns_do_not_create_free_space(self):
        for points in (self.circle(4)[:20], self.circle(4) + [0, 0, 5.0]):
            model = RayMap()
            with self.assertRaises(ValueError):
                model.update(points, np.eye(3), np.zeros(3), [0, 0], 0.0, 0.5)
            self.assertTrue((model.grid == UNKNOWN).all())


class PlanningTests(unittest.TestCase):
    def test_center_guidance_reduces_route_distance_from_ridges_after_shortcuts(self):
        model = RayMap(resolution=0.2, size=(50, 130), origin=(0, 0), clearance=0.8)
        model.grid[:] = FREE
        model.grid[[0, -1], :] = OCCUPIED
        model.grid[:, [0, -1]] = OCCUPIED
        snap = snapshot()
        snap.model = model
        snap.clearance, snap.body_free, snap.safe = model.geometry()
        field, centers, _ = routing_geometry(snap)
        old_cost = (
            1
            + model.route_clearance_weight
            * (model.route_clearance_scale / (field + model.route_clearance_scale)) ** 2
        )
        with patch("navigation.planning.routing_geometry", return_value=(field, centers, old_cost)):
            previous = plan(snap, [2, 1.2], [23, 1.2])
        corrected = plan(snap, [2, 1.2], [23, 1.2])
        center_distance = distance_field(centers, model.resolution, exterior_blocked=False)

        def mean_center_distance(path):
            samples = []
            for a, b in zip(path, path[1:]):
                count = max(2, int(math.dist(a, b) / 0.1))
                samples.extend(center_distance[model.cell(xy)] for xy in np.linspace(a, b, count))
            return float(np.mean(samples))

        self.assertLess(mean_center_distance(corrected), 0.8 * mean_center_distance(previous))
        self.assertLess(np.count_nonzero(speed_profile(np.asarray(corrected), 0.25, 3) == 0), 8)
        self.assertEqual(corrected[0], [2, 1.2])
        self.assertEqual(corrected[-1], [23, 1.2])

    def test_corridor_route_moves_to_known_center_and_shortcuts_keep_it_there(self):
        model = RayMap(resolution=0.2, size=(50, 130), origin=(0, 0), clearance=0.8)
        model.grid[:] = FREE
        model.grid[[0, -1], :] = OCCUPIED
        model.grid[:, [0, -1]] = OCCUPIED
        snap = snapshot()
        snap.model = model
        snap.clearance, snap.body_free, snap.safe = model.geometry()
        before = model.grid.copy()
        path = plan(snap, [2, 1.2], [23, 1.2])
        self.assertGreater(max(p[1] for p in path), 4.3)
        self.assertLess(
            np.count_nonzero(speed_profile(np.asarray(path), 0.25, 3) == 0),
            8,
            "Curve samples must not become braking stops",
        )
        self.assertEqual(path[0], [2, 1.2])
        self.assertEqual(path[-1], [23, 1.2])
        _, centers, _ = routing_geometry(snap)
        self.assertEqual(np.flatnonzero(centers[:, 65]).tolist(), [24, 25])
        self.assertTrue(
            all(
                segment_safe(snap.safe, model.cell(a), model.cell(b))
                for a, b in zip(path, path[1:])
            )
        )
        np.testing.assert_array_equal(model.grid, before)

    def test_clearance_preference_accepts_longer_wider_corridor(self):
        model = RayMap(resolution=0.2, size=(85, 140), origin=(0, 0), clearance=0.8)
        model.grid[:] = OCCUPIED
        model.grid[20:70, 5:25] = FREE
        model.grid[20:70, 115:135] = FREE
        model.grid[48:60, 25:115] = FREE  # Short, 2.4 m wide corridor.
        model.grid[5:35, 15:125] = FREE  # Longer, 6 m wide alternative.
        snap = snapshot()
        snap.model = model
        snap.clearance, snap.body_free, snap.safe = model.geometry()
        start, goal = model.world((54, 15)), model.world((54, 125))
        shortest = astar(snap.safe, model.cell(start), model.cell(goal))
        path = plan(snap, start, goal)
        shortest_m = sum(math.dist(a, b) for a, b in zip(shortest, shortest[1:])) * model.resolution
        actual_m = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        self.assertGreater(actual_m, shortest_m + 5)
        self.assertLess(min(q[1] for q in path), 6)
        self.assertTrue(
            all(
                segment_safe(snap.safe, model.cell(a), model.cell(b))
                for a, b in zip(path, path[1:])
            )
        )

    def test_unknown_boundary_limits_center_and_does_not_create_obstacles(self):
        snap = frontier_snapshot()
        field, centers, costs = routing_geometry(snap)
        before = snap.model.grid.copy()
        self.assertTrue(np.isfinite(costs).all())
        self.assertTrue((costs >= 1).all())
        self.assertTrue((snap.safe[centers]).all())
        self.assertLess(field[snap.model.cell([-0.5, 0])], 0.6)
        np.testing.assert_array_equal(snap.model.grid, before)
        with self.assertRaises(ValueError):
            plan(snap, [-2, 0], [2, 0])

    def test_mapped_return_uses_shortcut_instead_of_retracing_exploration(self):
        snap = snapshot(True)
        travel = [[-3, 0], [-3, 5], [3, 5], [3, 0]]
        route = plan(snap, travel[-1], travel[0])
        length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
        self.assertLess(length, sum(math.dist(a, b) for a, b in zip(travel, travel[1:])))
        self.assertEqual(route[-1], travel[0])
        self.assertGreater(max(abs(p[1]) for p in route), 1.5)

    def test_frontier_distance_counts_the_detour_in_known_space(self):
        snap = snapshot(True)
        distances = route_distances(snap, [-3.0, 0.0])
        self.assertGreater(distances[snap.model.cell([3.0, 0.0])], 6.0)
        self.assertTrue(math.isinf(distances[snap.model.cell([0.0, 0.0])]))

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
    def test_yaw_alignment_reduces_speed_then_allows_acceleration(self):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [100, 0]])
        for _ in range(20):
            follower.tick(
                0.05, follower.target.copy(), np.array([[100, 0]]), heading_error=math.pi / 2
            )
        self.assertGreater(follower.target[0], 0)
        self.assertIn("yaw aligns", follower.reason)
        for _ in range(240):
            follower.tick(
                0.05, follower.target.copy(), np.array([[100, 0]]), heading_error=math.radians(60)
            )
        slow_speed = follower.speed
        self.assertAlmostEqual(slow_speed, 0.7)
        self.assertLess(slow_speed, follower.settings.cruise_m_s * 0.95)
        for _ in range(120):
            follower.tick(
                0.05, follower.target.copy(), np.array([[100, 0]]), heading_error=math.radians(5)
            )
        self.assertGreater(follower.speed, slow_speed + 0.15)

    def test_reversal_brakes_vector_without_jump_and_tracks_with_response_lag(self):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [100, 0]])
        actual, physical_velocity = np.zeros(2), np.zeros(2)
        switched = False
        replanned = False
        minimum_forward_speed = math.inf
        for i in range(300):
            if not switched and follower.speed >= 0.6:
                old_target = follower.target.copy()
                follower.retarget([old_target.tolist(), (old_target + [-20, 2]).tolist()])
                np.testing.assert_array_equal(follower.target, old_target)
                switched = True
            before = follower.target.copy()
            previous_velocity = follower.velocity.copy()
            follower.tick(
                0.05,
                actual,
                np.array([[100, 0], [-100, 0]]),
                observed_corridor=True,
                motion_safe=lambda a, b: True,
                measured_velocity=physical_velocity,
                braking_safe=lambda points: True,
            )
            self.assertIn(follower.phase, ("NAVIGATING", "PLANNING"))
            self.assertLessEqual(
                np.linalg.norm(follower.velocity - previous_velocity), 0.25 * 0.05 + 1e-8
            )
            feedforward = (follower.target - before) / 0.05
            desired = feedforward + 0.95 * (follower.target - actual)
            physical_velocity += np.clip((desired - physical_velocity) * 0.05 / 0.15, -0.15, 0.15)
            actual += physical_velocity * 0.05
            if follower.phase == "PLANNING":
                self.assertLessEqual(np.linalg.norm(physical_velocity), 0.15)
                follower.begin([actual.tolist(), (actual + [-20, 2]).tolist()])
                replanned = True
            if switched:
                minimum_forward_speed = min(minimum_forward_speed, follower.velocity[0])
        self.assertTrue(switched)
        self.assertTrue(replanned)
        self.assertLess(minimum_forward_speed, -0.5)

    def test_turn_braking_cannot_enter_unobserved_cells_or_ignore_close_old_direction(self):
        for close_obstacle in (False, True):
            follower = RouteFollower(load_settings())
            follower.begin([[0, 0], [10, 0]])
            follower.velocity = np.array([0.6, 0.0])
            follower.speed = 0.6
            follower.retarget([[0, 0], [-10, 0]])
            hits = np.array([[0.6 if close_obstacle else 10, 0], [-10, 0]])
            follower.tick(
                0.05,
                [0, 0],
                hits,
                motion_safe=lambda a, b: close_obstacle,
                measured_velocity=[0.6, 0],
                braking_safe=lambda points: close_obstacle,
            )
            self.assertEqual(follower.phase, "BLOCKED")
            np.testing.assert_array_equal(follower.target, [0, 0])
            np.testing.assert_array_equal(follower.velocity, [0, 0])

    def test_configured_feedforward_tracks_with_response_lag_and_preserves_lead_guard(self):
        # Simplified PX4-shaped position + velocity response, not a flight proof.
        for response_s in (0.05, 0.15, 0.30):
            settings = load_settings()
            follower = RouteFollower(settings)
            follower.begin([[0, 0], [100, 0]])
            actual, velocity = np.zeros(2), np.zeros(2)
            peak = 0.0
            for _ in range(3000):
                previous = follower.target.copy()
                follower.tick(0.05, actual, np.array([[102, 0]]))
                self.assertLessEqual(
                    np.linalg.norm(follower.target - actual), settings.target_lead_m + 1e-9
                )
                feedforward = (
                    (follower.target - previous) / 0.05
                    if follower.phase == "NAVIGATING"
                    else np.zeros(2)
                )
                desired = feedforward + 0.95 * (follower.target - actual)
                velocity += np.clip((desired - velocity) * 0.05 / response_s, -0.15, 0.15)
                actual += velocity * 0.05
                peak = max(peak, float(np.linalg.norm(velocity)))
                self.assertIn(follower.phase, ("NAVIGATING", "SETTLING"))
                self.assertGreater(102 - actual[0], settings.stop_distance_m)
                if (
                    follower.phase == "SETTLING"
                    and np.linalg.norm(actual - [100, 0]) < 0.05
                    and np.linalg.norm(velocity) < 0.05
                ):
                    break
            self.assertGreater(peak, settings.cruise_m_s - 0.1)
            self.assertLess(peak, settings.cruise_m_s + 0.1)
            self.assertEqual(follower.phase, "SETTLING")
            self.assertLess(np.linalg.norm(actual - [100, 0]), 0.05)

    def test_velocity_frame_conversion_rotates_without_applying_position_offset(self):
        alignment = FrameAlignment([-28, -17], 0.3, [3, 9], 0.1, position_yaw=0.6)
        velocity = np.array([4.0, 0.0])
        expected = alignment.ned(np.array([7.0, -4.0]) + velocity) - alignment.ned([7.0, -4.0])
        np.testing.assert_allclose(alignment.ned_velocity(velocity), expected)
        self.assertAlmostEqual(np.linalg.norm(alignment.ned_velocity(velocity)), 4.0)

    def test_braking_limit_reserves_reaction_travel_and_stopping_distance(self):
        for distance in (0.65, 0.7, 0.8, 1.0, 2.0, 3.15, 5.0, 12.0, 34.65):
            speed = braking_speed(distance, 0.65, 0.25, 0.5)
            self.assertGreaterEqual(speed, 0)
            self.assertLessEqual(speed * 0.5 + speed**2 / (2 * 0.25), distance - 0.65 + 1e-12)
        self.assertAlmostEqual(braking_speed(3.15, 0.65, 0.25, 0.5), 1.0)
        self.assertAlmostEqual(braking_speed(34.65, 0.65, 0.25, 0.5), 4.0)
        self.assertLess(braking_speed(1.0, 0.65, 0.25, 0.5), 0.5)
        self.assertEqual(braking_speed(None, 0.65, 0.25, 0.5), 0)

    def test_configured_cruise_tracks_a_goal_before_a_wall_with_response_lag(self):
        settings = load_settings()
        self.assertEqual(settings.cruise_m_s, 1.2)
        follower = RouteFollower(settings)
        follower.begin([[0.0, 0.0], [6.0, 0.0]])
        actual = np.zeros(2)
        peak = 0.0
        minimum_clearance = math.inf
        for _ in range(2000):
            follower.tick(0.05, actual, np.array([[7.0, 0.0]]))
            actual += (follower.target - actual) * 0.05 / 0.2
            peak = max(peak, follower.speed)
            minimum_clearance = min(minimum_clearance, 7.0 - actual[0])
            self.assertIn(follower.phase, ("NAVIGATING", "SETTLING"))
            if follower.phase == "SETTLING" and np.linalg.norm(actual - [6.0, 0.0]) < 0.05:
                break
        else:
            self.fail("Faster reference did not settle at its goal")
        self.assertGreater(peak, 0.75 * settings.cruise_m_s)
        self.assertLessEqual(peak, settings.cruise_m_s + 1e-12)
        self.assertGreater(minimum_clearance, settings.stop_distance_m)

    def test_four_metre_reference_reaches_cap_only_with_long_clear_route_and_good_tracking(self):
        settings = NavigationSettings(cruise_m_s=4.0).validate()
        follower = RouteFollower(settings)
        follower.begin([[0, 0], [100, 0]])
        peak = 0.0
        for _ in range(3000):
            actual = follower.target.copy()
            follower.tick(0.05, actual, np.empty((0, 2)), observed_corridor=True)
            peak = max(peak, follower.speed)
            self.assertLessEqual(
                np.linalg.norm(follower.target - actual), settings.target_lead_m + 1e-9
            )
            if follower.phase == "SETTLING":
                break
        self.assertAlmostEqual(peak, 4.0)
        self.assertEqual(follower.phase, "SETTLING")

    def test_fast_cruise_brakes_before_entering_the_bend_zone(self):
        follower = RouteFollower(load_settings())
        follower.begin([[0, 0], [6, 0], [6, 6]])
        peak, bend_speeds = 0.0, []
        for _ in range(2000):
            actual = follower.target.copy()
            follower.tick(0.05, actual, np.empty((0, 2)), observed_corridor=True)
            peak = max(peak, follower.speed)
            if follower.index == 1 and np.linalg.norm(follower.target - [6, 0]) < 0.4:
                bend_speeds.append(follower.speed)
            if follower.phase == "SETTLING":
                break
        self.assertGreater(peak, 0.95)
        self.assertTrue(bend_speeds)
        self.assertLessEqual(max(bend_speeds), follower.settings.turn_speed_m_s + 0.02)

    def test_yaw_turns_shortest_way_across_angle_wrap(self):
        yaw, rate = smooth_yaw(
            math.radians(179), 0.0, math.radians(-179), 0.1, math.radians(45), math.radians(90)
        )
        self.assertGreater(rate, 0)
        self.assertLess(abs(math.remainder(math.radians(-179) - yaw, 2 * math.pi)), math.radians(2))

    def test_yaw_rate_and_acceleration_stay_bounded_through_route_reversal(self):
        yaw, rate = 0.0, 0.0
        dt = 0.05
        limit, acceleration = math.radians(45), math.radians(90)
        for i in range(200):
            desired = math.pi / 2 if i < 40 else -math.pi / 2
            previous_yaw, previous_rate = yaw, rate
            yaw, rate = smooth_yaw(yaw, rate, desired, dt, limit, acceleration)
            self.assertLessEqual(abs(rate), limit + 1e-12)
            self.assertLessEqual(abs(rate - previous_rate), acceleration * dt + 1e-12)
            self.assertLessEqual(
                abs(math.remainder(yaw - previous_yaw, 2 * math.pi)), limit * dt + 1e-12
            )
        self.assertLess(abs(math.remainder(yaw + math.pi / 2, 2 * math.pi)), 0.001)

    def test_yaw_freezes_when_stopped_or_simulation_time_is_unusable(self):
        for desired, dt in ((None, 0.1), (None, 0), (1.0, -0.1), (1.0, 0.6), (1.0, math.nan)):
            yaw, rate = smooth_yaw(0.4, 0.2, desired, dt, 0.8, 1.5)
            self.assertEqual(yaw, 0.4)
            self.assertEqual(rate, 0)

    def test_route_heading_conversion_corrects_bias_without_rotating_positions(self):
        alignment = FrameAlignment([0, 0], 0.2, [0, 0], math.pi / 2)
        for vector, bearing in (
            ([1, 0], 0),
            ([0, 1], math.pi / 2),
            ([-1, 0], math.pi),
            ([0, -1], -math.pi / 2),
        ):
            heading = alignment.yaw_ned(vector)
            true_world = math.pi / 2 - heading + alignment.heading_bias
            self.assertAlmostEqual(math.remainder(true_world - bearing, 2 * math.pi), 0)
        np.testing.assert_allclose(alignment.world([1, 0]), [0, 1])
        with self.assertRaises(ValueError):
            alignment.yaw_ned([0, 0])

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
    node.status_sim_s = 100.0
    node.auto_recover = False
    node.recovery_stamp = None
    node.recovery_scans = 0
    node.recovery_started_s = None
    node.commands = Queue()
    node.land_pub = SimpleNamespace(get_subscription_count=lambda: 1, publish=lambda msg: None)
    node.status.pre_flight_checks_pass = True
    node.alignment = FrameAlignment([2.0, 3.0], math.pi / 2, [2.0, 3.0], 0.0)
    node.world_pose = np.array([2.0, 3.0])
    node.reference = np.array([2.0, 3.0])
    node.hold_yaw = 0.0
    node.yaw_rate = 0.0
    node.yaw_desired = None
    node.follower = RouteFollower()
    node.follower.begin([[2.0, 3.0], [2.0, 2.0]])
    node.target_goal = np.array([2.0, 2.0])
    node.home_world_xy = None
    node.returning_home = False
    node.pacer_mark = 100.0
    node.unclocked_imu_s = 0.0
    node.acceleration_reference_ned = np.zeros(2)
    node.native_control = None
    node.control_backend = "python"
    node.sample_timeline = None
    node.reference_history = ReferenceHistory()
    node.tracking_alignment = dict(mode="prediction_fallback")
    node.alignment_counters = None

    # Most fixtures isolate control logic; dedicated timing tests exercise the
    # independent Isaac clock, including dropped IMU messages.
    def clock_elapsed(now):
        elapsed = node.pacer.total_s - node.pacer_mark
        return elapsed if 0 <= elapsed <= 0.25 else None

    node.trajectory_clock = SimpleNamespace(
        consume=clock_elapsed, fresh=lambda now: True, total_s=100.0
    )
    node.last_callback = 99.9
    node.last_retry = 100.0
    node.last_map_scans = 3
    node.plan_results = Queue()
    node.plan_generation = 0
    node.plan_thread = None
    node.last_report = 100.0
    node.exploration = ExplorationSession()
    from navigation.planning_jobs import PlanningJobs

    node.planning = node.replanning = SimpleNamespace(call=PlanningJobs())
    node.reconsider_thread = None
    node.reconsider_results = Queue()
    node.reconsider_mark = None
    node.reconsider_stamp = None
    node.active_ui_command = None
    node.alignment_settled_since = None
    node.alignment_capture_stamp = None
    node.alignment_capture_scans = 0
    node.mission = ExploreMission()
    node.mission_resume_phase = None
    node.last_completion_stamp = None
    node.certified_grid = None
    node.landed = False
    node.landed_wall = None
    node.landed_sim_s = None
    node.settings = NavigationSettings()
    node.pose_history = deque([(100.0, [2.0, 3.0], 0.0)])
    for name in (
        "flight_active",
        "status_fresh",
        "stable",
        "stop",
        "commands_tick",
        "launch_plan",
        "launch_frontier",
        "exploration_tick",
        "complete_exploration",
        "map_pose_matches",
        "mission_tick",
        "recovery_tick",
        "launch_reconsideration",
        "reconsideration_tick",
        "capture_alignment",
        "retire_obsolete_goal",
        "command_feedback",
        "update_command_progress",
        "reject_pending_commands",
        "flight_readiness",
        "update_yaw",
        "launch_mission_check",
        "launch_map_save",
    ):
        setattr(node, name, MethodType(getattr(GoalNavigator, name), node))
    node.update_dashboard = lambda *args: None
    return node, modes, targets, notices


class RuntimeTests(unittest.TestCase):
    def test_braking_completion_replans_retained_goal_without_operator_resume(self):
        node, modes, targets, notices = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.follower.braking_envelope = np.tile([2.0, 3.0], (4, 1))
        node.follower.turn_stage = "SETTLING"
        node.position.heading = math.pi
        node.exploration.enabled = True
        node.exploration.phase = "TRAVELLING"
        node.exploration.continuous = False
        # Request a route without waiting for yaw or a dwell. Planner launch
        # still waits for measured speed <=0.10 m/s, even below the follower's
        # <=0.15 m/s handover threshold.
        node.position.vx = -0.11
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            node.pacer.total_s += 0.05
            GoalNavigator.publish_target(node)
            self.assertEqual(node.follower.phase, "PLANNING", node.follower.reason)
            node.pacer.total_s += 0.05
            GoalNavigator.publish_target(node)
            self.assertEqual(node.follower.phase, "PLANNING")
            self.assertIsNone(node.plan_thread)
            node.position.vx = -0.09
            with patch("navigation.planning_jobs.plan", return_value=[[2, 3], [2, 4]]):
                node.pacer.total_s += 0.05
                GoalNavigator.publish_target(node)
                node.plan_thread.join(timeout=2)
                self.assertFalse(node.plan_thread.is_alive())
            node.pacer.total_s += 0.05
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "NAVIGATING")
        self.assertIsNone(node.follower.braking_envelope)
        self.assertEqual(node.exploration.phase, "TRAVELLING")
        self.assertTrue(node.exploration.enabled)
        np.testing.assert_array_equal(node.target_goal, [2, 4])
        self.assertEqual(len(modes), 4)
        self.assertEqual(len(targets), 4)
        self.assertGreater(node.follower.speed, 0)  # Heading alone no longer holds translation.
        self.assertLessEqual(node.follower.speed, 0.25 * 0.10 + 1e-9)
        self.assertFalse(any("0.35" in text for text in notices))

    def test_empty_route_after_alignment_keeps_hold_stream_and_dashboard_alive(self):
        for phase in ("HOLD", "PAUSED", "PLANNING"):
            node, modes, targets, _ = navigator_fixture()
            node.follower.path = []
            node.follower.hold(node.world_pose, "No route selected", phase)
            node.target_goal = None
            node.pacer.total_s += 0.05
            with patch("ros2_navigate.time.monotonic", return_value=100.0):
                GoalNavigator.publish_target(node)
            self.assertEqual(node.follower.phase, phase)
            self.assertTrue(modes and targets)
            np.testing.assert_allclose(targets[-1].position[:2], node.reference)
            np.testing.assert_allclose(targets[-1].velocity[:2], [0, 0])

    def test_navigator_feedforward_uses_validated_reference_advance_in_ned(self):
        node, modes, targets, _ = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])  # Aligned with measured North heading.
        node.target_goal = np.array([2, 4])
        before = node.follower.target.copy()
        node.pacer.total_s += 0.05
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        expected = node.alignment.ned_velocity((node.follower.target - before) / 0.05)
        np.testing.assert_allclose(targets[-1].velocity[:2], expected, atol=1e-6)
        self.assertGreater(np.linalg.norm(expected), 0)
        self.assertTrue(modes[-1].position)
        self.assertFalse(modes[-1].velocity)
        self.assertTrue(math.isnan(targets[-1].velocity[2]))
        np.testing.assert_allclose(targets[-1].acceleration[:2], expected / 0.05, atol=1e-6)
        self.assertTrue(math.isnan(targets[-1].acceleration[2]))

    def test_navigator_dropped_imu_periods_do_not_change_reference_velocity_timebase(self):
        node, _, targets, _ = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])
        node.target_goal = np.array([2, 4])
        node.pacer.total_s += 0.02  # Partial delivery, not the physics elapsed time.
        node.trajectory_clock.consume = lambda now: 0.05
        before = node.follower.target.copy()
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        expected = node.alignment.ned_velocity((node.follower.target - before) / 0.05)
        np.testing.assert_allclose(targets[-1].velocity[:2], expected, atol=1e-6)
        self.assertGreater(np.linalg.norm(expected), 0)

    def test_missing_isaac_clock_pauses_exploration_without_losing_heartbeat(self):
        node, modes, targets, _ = navigator_fixture()
        node.exploration.start(node.feed.get()[0])
        node.trajectory_clock.consume = lambda now: None
        node.trajectory_clock.fresh = lambda now: False
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertEqual(node.exploration.phase, "PAUSED")
        self.assertIn("clock", node.follower.reason)
        self.assertTrue(modes and targets)
        np.testing.assert_allclose(targets[-1].velocity[:2], [0, 0])

    def test_sparse_clock_and_imu_samples_do_not_pulse_velocity_to_zero_while_fresh(self):
        node, _, targets, _ = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])
        node.follower.velocity = np.array([0, 0.3])
        node.follower.speed = 0.3
        node.trajectory_clock.consume = lambda now: 0
        node.pacer.total_s += 0.04
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
            np.testing.assert_allclose(targets[-1].velocity[:2], [0.3, 0], atol=1e-6)
            GoalNavigator.publish_target(node)  # No new clock or IMU progression.
        np.testing.assert_allclose(targets[-1].velocity[:2], [0.3, 0], atol=1e-6)

    def test_feedforward_between_batches_expires_with_real_clock_freshness(self):
        node, _, targets, _ = navigator_fixture()
        node.follower.begin([[2, 3], [2, 4]])
        node.follower.velocity = np.array([0, 0.3])
        node.follower.speed = 0.3
        clock = TrajectoryClock()
        for index in range(5):
            clock.observe(index * 50_000_000, 99.8 + index * 0.05)
        clock.consume(100.0)
        node.trajectory_clock = clock
        previous = node.follower.target.copy()
        for now in (100.0, 100.05, 100.1, 100.2):
            with patch("ros2_navigate.time.monotonic", return_value=now):
                GoalNavigator.publish_target(node)
            np.testing.assert_allclose(targets[-1].velocity[:2], [0.3, 0], atol=1e-6)
            np.testing.assert_array_equal(node.follower.target, previous)
        with patch("ros2_navigate.time.monotonic", return_value=100.26):
            GoalNavigator.publish_target(node)
        np.testing.assert_array_equal(targets[-1].velocity[:2], [0, 0])
        self.assertEqual(node.follower.phase, "PAUSED")

    def test_feedforward_clears_on_hold_map_loss_pacing_gap_and_stopped_simulation(self):
        for scenario in ("hold", "map_loss", "callback_gap", "stopped_sim"):
            node, _, targets, _ = navigator_fixture()
            node.follower.begin([[2, 3], [2, 4]])
            node.target_goal = np.array([2, 4])
            node.pacer.total_s += 0.05
            with patch("ros2_navigate.time.monotonic", return_value=100.0):
                GoalNavigator.publish_target(node)
                self.assertGreater(np.linalg.norm(targets[-1].velocity[:2]), 0)
                if scenario != "stopped_sim":
                    node.pacer.total_s += 0.05
                if scenario == "hold":
                    node.commands.put(dict(action="hold"))
                elif scenario == "map_loss":
                    node.feed.get = lambda: (None, "Map unavailable")
                elif scenario == "callback_gap":
                    node.last_callback = 99.0
                elif scenario == "stopped_sim":
                    node.trajectory_clock.consume = lambda now: None
                    node.trajectory_clock.fresh = lambda now: False
                GoalNavigator.publish_target(node)
            np.testing.assert_allclose(targets[-1].velocity[:2], [0, 0])

    def test_home_request_during_motion_brakes_and_cancels_exploration_mission(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.home_world_xy = np.array([1.0, 2.0])
        node.position.vx = 0.4
        node.exploration.start(snap)
        node.mission.start()
        previous_generation = node.plan_generation
        node.commands.put(dict(action="home", request_id="return"))
        node.commands_tick(100.0, snap, True)
        self.assertEqual(node.follower.phase, "PLANNING")
        np.testing.assert_allclose(node.follower.target, node.world_pose)
        np.testing.assert_allclose(node.target_goal, [1, 2])
        self.assertTrue(node.returning_home)
        self.assertFalse(node.exploration.enabled)
        self.assertFalse(node.mission.enabled)
        self.assertIsNone(node.plan_thread)
        self.assertGreater(node.plan_generation, previous_generation)
        node.commands.put(dict(action="hold"))
        node.commands_tick(100.0, snap, True)
        self.assertFalse(node.returning_home)
        self.assertIsNone(node.target_goal)
        np.testing.assert_allclose(node.home_world_xy, [1, 2])

    def test_home_requires_saved_home_fresh_map_and_navigation_height(self):
        for captured, fresh, height in ((False, True, -1), (True, False, -1), (True, True, -2)):
            node, _, _, _ = navigator_fixture()
            if captured:
                node.home_world_xy = np.array([1.0, 2.0])
            node.position.z = height
            replies = []
            node.dashboard.acknowledge_command = lambda req, status, reason: replies.append(status)
            node.commands.put(dict(action="home", request_id="return"))
            node.commands_tick(100.0, node.feed.get()[0], fresh)
            self.assertEqual(replies, ["rejected"])
            self.assertFalse(node.returning_home)

    def test_return_waits_for_stability_then_plans_to_home_and_blocks_unknown_route(self):
        for unreachable in (False, True):
            node, _, _, _ = navigator_fixture()
            snap = node.feed.get()[0]
            node.home_world_xy = np.array([-3.0, 3.0])
            if unreachable:
                snap.model.grid[:, 60] = UNKNOWN
                snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
            node.position.vx = 0.3
            node.commands.put(dict(action="home", request_id="return"))
            with patch("ros2_navigate.time.monotonic", return_value=100.0):
                GoalNavigator.publish_target(node)
                self.assertIsNone(node.plan_thread)
                node.position.vx = 0
                GoalNavigator.publish_target(node)
                node.plan_thread.join(5)
                self.assertFalse(node.plan_thread.is_alive())
                GoalNavigator.publish_target(node)
            self.assertEqual(node.follower.phase, "BLOCKED" if unreachable else "NAVIGATING")
            self.assertEqual(node.mission.phase, "IDLE")
            if not unreachable:
                np.testing.assert_allclose(node.follower.path[-1], node.home_world_xy)

    def test_home_refuses_invalid_velocity_and_manual_goal_replaces_return_intent(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.home_world_xy = np.array([1.0, 2.0])
        node.position.v_xy_valid = False
        replies = []
        node.dashboard.acknowledge_command = lambda req, status, reason: replies.append(status)
        node.commands.put(dict(action="home", request_id="invalid"))
        node.commands_tick(100.0, snap, True)
        self.assertEqual(replies, ["rejected"])
        node.position.v_xy_valid = True
        node.commands.put(dict(action="home", point=[999, 999], request_id="return"))
        node.commands_tick(100.0, snap, True)
        np.testing.assert_allclose(node.target_goal, node.home_world_xy)
        node.commands.put(dict(action="goal", point=[2.0, 2.0], request_id="manual"))
        node.commands_tick(100.0, snap, True)
        node.plan_thread.join(5)
        self.assertFalse(node.returning_home)
        np.testing.assert_allclose(node.target_goal, [2, 2])
        np.testing.assert_allclose(node.home_world_xy, [1, 2])

    def test_active_route_replans_around_new_obstacle_and_retries_if_no_route_exists(self):
        from navigation.prepared_route import route_safe

        for mode, unreachable in (
            ("goal", False),
            ("goal", True),
            ("home", False),
            ("home", True),
            ("explore", False),
        ):
            with self.subTest(mode=mode, unreachable=unreachable):
                node, _, targets, _ = navigator_fixture()
                snap = node.feed.get()[0]
                node.home_world_xy = np.array([-3.0, 3.0])
                node.target_goal = node.home_world_xy.copy()
                node.returning_home = mode == "home"
                if mode == "explore":
                    node.exploration.start(snap, continuous=False)
                    node.exploration.phase = "TRAVELLING"
                node.follower.begin([node.world_pose, node.home_world_xy])
                y, x = snap.model.cell([0, 3])
                if unreachable:
                    snap.model.grid[:, x] = OCCUPIED
                else:
                    snap.model.grid[y - 2 : y + 3, x - 2 : x + 3] = OCCUPIED
                snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
                snap.scans += 1
                self.assertFalse(route_safe(snap, node.follower.path))
                node.last_retry = 97.0
                with patch("ros2_navigate.time.monotonic", return_value=100.0):
                    GoalNavigator.publish_target(node)
                    self.assertEqual(node.follower.phase, "BLOCKED")
                    np.testing.assert_allclose(targets[-1].velocity[:2], [0, 0])
                    # Do not start a new route until measured motion settles.
                    node.position.vx = 0.2
                    GoalNavigator.publish_target(node)
                    self.assertIsNone(node.plan_thread)
                    node.position.vx = 0
                    GoalNavigator.publish_target(node)
                    node.plan_thread.join(5)
                    self.assertFalse(node.plan_thread.is_alive())
                    GoalNavigator.publish_target(node)
                    if unreachable:
                        self.assertEqual(node.follower.phase, "BLOCKED")
                        previous_thread = node.plan_thread
                        GoalNavigator.publish_target(node)
                        self.assertIs(node.plan_thread, previous_thread)
                        node.last_retry = 97.0
                        GoalNavigator.publish_target(node)
                        self.assertIsNot(node.plan_thread, previous_thread)
                        node.plan_thread.join(5)
                        GoalNavigator.publish_target(node)
                        self.assertEqual(node.follower.phase, "BLOCKED")
                        np.testing.assert_allclose(targets[-1].velocity[:2], [0, 0])
                    else:
                        self.assertEqual(node.follower.phase, "NAVIGATING")
                        self.assertTrue(route_safe(snap, node.follower.path))
                        self.assertGreater(len(node.follower.path), 2)
                np.testing.assert_allclose(node.target_goal, node.home_world_xy)
                self.assertEqual(node.returning_home, mode == "home")

    def test_active_route_recovers_input_gap_and_plans_from_latest_observed_map(self):
        from navigation.prepared_route import route_safe
        from itertools import product

        for mode, gap in product(("goal", "home", "explore"), ("map", "clock")):
            with self.subTest(mode=mode, gap=gap):
                node, _, targets, _ = navigator_fixture()
                snap = node.feed.get()[0]
                node.home_world_xy = np.array([-3.0, 3.0])
                node.target_goal = node.home_world_xy.copy()
                node.returning_home = mode == "home"
                if mode == "explore":
                    node.exploration.start(snap, continuous=False)
                    node.exploration.phase = "TRAVELLING"
                command = dict(action=mode, request_id="route")
                node.active_ui_command = command
                node.follower.begin([node.world_pose, node.home_world_xy])
                node.stop("New obstacle; waiting to replan", "BLOCKED")
                if gap == "map":
                    snap.cloud_wall = 98.9
                else:
                    node.trajectory_clock.consume = lambda now: None
                    node.trajectory_clock.fresh = lambda now: False
                with patch("ros2_navigate.time.monotonic", return_value=100.0):
                    GoalNavigator.publish_target(node)
                    self.assertEqual(node.follower.phase, "PAUSED")
                    self.assertTrue(node.auto_recover)
                    self.assertIsNone(node.plan_thread)
                    # Map updates while paused must affect the replacement route.
                    y, x = snap.model.cell([0, 3])
                    snap.model.grid[y - 2 : y + 3, x - 2 : x + 3] = OCCUPIED
                    snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
                    snap.cloud_wall = 100
                    node.trajectory_clock.consume = lambda now: 0.05
                    node.trajectory_clock.fresh = lambda now: True
                    for index in range(3):
                        snap.stamp += 1
                        snap.scans += 1
                        node.pacer.total_s = 100.0 + index * 0.5
                        GoalNavigator.publish_target(node)
                        if index < 2:
                            self.assertEqual(node.follower.phase, "PAUSED")
                            self.assertTrue(node.auto_recover)
                            self.assertIsNone(node.plan_thread)
                            np.testing.assert_allclose(targets[-1].velocity[:2], [0, 0])
                    node.plan_thread.join(5)
                    self.assertFalse(node.plan_thread.is_alive())
                    GoalNavigator.publish_target(node)
                self.assertEqual(node.follower.phase, "NAVIGATING")
                self.assertTrue(route_safe(snap, node.follower.path))
                self.assertGreater(len(node.follower.path), 2)
                if mode != "explore":
                    self.assertIs(node.active_ui_command, command)
                else:
                    self.assertEqual(node.exploration.phase, "TRAVELLING")
                self.assertFalse(node.auto_recover)
                self.assertTrue(node.commands.empty())
                np.testing.assert_allclose(node.target_goal, node.home_world_xy)

    def test_home_recovery_is_cancelled_by_hold_or_safety_fault(self):
        for fault in ("hold", "pose", "height", "tracking", "callback", "position", "offboard"):
            with self.subTest(fault=fault):
                node, _, _, _ = navigator_fixture()
                snap = node.feed.get()[0]
                node.home_world_xy = node.target_goal.copy()
                node.returning_home = True
                node.stop("Temporary input gap", auto_recover=True)
                if fault == "hold":
                    node.commands.put(dict(action="hold"))
                elif fault == "pose":
                    snap.pose[0] += 0.3
                elif fault == "height":
                    node.position.z = -1.3
                elif fault == "tracking":
                    node.stop("Trajectory tracking error exceeded 0.35 m")
                elif fault == "position":
                    node.state.last_progress = 90.0
                elif fault == "offboard":
                    node.status.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
                else:
                    node.last_callback = 99.0
                with patch("ros2_navigate.time.monotonic", return_value=100.0):
                    GoalNavigator.publish_target(node)
                    self.assertFalse(node.auto_recover)
                    self.assertIsNone(node.plan_thread)
                    # Restoring inputs alone must not resume an operator/safety pause.
                    snap.pose[0] = 2.0
                    node.position.z = -1.0
                    node.position.timestamp += 1
                    node.state.observe(node.position, 100.0)
                    node.status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    for index in range(3):
                        snap.stamp += 1
                        node.pacer.total_s = 100 + index * 0.5
                        GoalNavigator.publish_target(node)
                self.assertIsNone(node.plan_thread)
                self.assertTrue(node.commands.empty())
                self.assertEqual(
                    node.follower.phase, "HOLD" if fault in ("hold", "offboard") else "PAUSED"
                )

    def test_home_arrival_confirms_settling_and_finishes_hover_without_land(self):
        node, _, _, _ = navigator_fixture()
        node.trajectory_clock.consume = lambda now: 0.05
        node.home_world_xy = node.world_pose.copy()
        node.target_goal = node.home_world_xy.copy()
        node.returning_home = True
        node.active_ui_command = dict(action="home", request_id="return")
        node.follower.hold(node.world_pose, "Settling home", "SETTLING")
        land_commands = []
        node.land_pub.publish = land_commands.append
        replies = []
        node.dashboard.acknowledge_command = lambda req, status, reason: replies.append(status)
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
            self.assertEqual(node.follower.phase, "SETTLING")
            node.pacer.total_s += 1.0
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "REACHED")
        self.assertIn("Home reached", node.follower.reason)
        node.update_command_progress()
        self.assertEqual(replies, ["completed"])
        self.assertEqual(land_commands, [])

    def test_yaw_changes_with_route_without_changing_position_reference(self):
        node, modes, targets, _ = navigator_fixture()
        node.pacer.total_s += 0.1
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(len(modes), 1)
        self.assertGreater(targets[-1].yaw, 0)
        self.assertLess(targets[-1].yaw, math.radians(45) * 0.1)
        np.testing.assert_allclose(targets[-1].position[:2], node.reference, atol=1e-6)

    def test_yaw_freezes_on_hold_stale_map_or_callback_gap(self):
        for phase, map_ready, gap in (
            ("HOLD", True, 0.1),
            ("BLOCKED", True, 0.1),
            ("NAVIGATING", False, 0.1),
            ("NAVIGATING", True, 0.8),
        ):
            node, _, _, _ = navigator_fixture()
            node.follower.phase = phase
            node.hold_yaw, node.yaw_rate = 0.4, 0.2
            node.update_yaw(0.1, 100.0, map_ready, gap)
            self.assertEqual(node.hold_yaw, 0.4)
            self.assertEqual(node.yaw_rate, 0)

    def test_yaw_keeps_rate_between_healthy_clock_samples_but_clears_it_when_stale(self):
        node, _, _, _ = navigator_fixture()
        node.update_yaw(0.05, 100.0, True, 0.05)
        yaw, rate = node.hold_yaw, node.yaw_rate
        self.assertNotEqual(rate, 0)
        node.update_yaw(0, 100.0, True, 0.05)
        self.assertEqual(node.hold_yaw, yaw)
        self.assertEqual(node.yaw_rate, rate)
        node.trajectory_clock.fresh = lambda _: False
        node.update_yaw(0, 100.0, True, 0.05)
        self.assertEqual(node.hold_yaw, yaw)
        self.assertEqual(node.yaw_rate, 0)

    def test_yaw_reference_remains_continuous_when_frontier_target_changes(self):
        node, _, _, _ = navigator_fixture()
        node.update_yaw(0.1, 100.0, True, 0.1)
        before = node.hold_yaw
        node.follower.retarget([[2, 3], [4, 3]])
        node.update_yaw(0.1, 100.0, True, 0.1)
        self.assertLessEqual(
            abs(math.remainder(node.hold_yaw - before, 2 * math.pi)), math.radians(45) * 0.1
        )

    def test_ground_flight_setup_names_takeoff_and_requires_fresh_landed_state(self):
        node, _, _, _ = navigator_fixture()
        node.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        node.position.z = 0.0
        node.landed = True
        node.landed_wall = node.landed_sim_s = 100.0
        result = node.flight_readiness(100.0, False)
        self.assertTrue(result["can_takeoff"])
        self.assertFalse(result["can_enable_navigation"])
        self.assertIn("Take off to 1 m", result["next_step"])
        self.assertTrue(result["checks"]["telemetry_fresh"])
        self.assertTrue(result["checks"]["preflight_ok"])
        self.assertTrue(result["checks"]["landed"])
        self.assertTrue(result["checks"]["ground_origin"])
        node.landed_wall = 80.0
        stale = node.flight_readiness(100.0, False)
        self.assertFalse(stale["can_takeoff"])
        self.assertIsNone(stale["checks"]["landed"])

    def test_offboard_button_requires_map_alignment_stability_and_target_stream(self):
        node, _, _, _ = navigator_fixture()
        node.status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        self.assertTrue(node.flight_readiness(100.0, True)["can_enable_navigation"])
        self.assertIn("Enable navigation", node.flight_readiness(100.0, True)["next_step"])
        self.assertFalse(node.flight_readiness(100.0, False)["can_enable_navigation"])
        node.ready_samples = 0
        self.assertFalse(node.flight_readiness(100.0, True)["can_enable_navigation"])

    def test_hold_cancels_queued_movement_and_stale_planner_results(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        replies = []
        node.dashboard.acknowledge_command = lambda request, status, reason: replies.append(
            (request, status)
        )
        old_generation = node.plan_generation
        node.commands.put(dict(action="goal", point=[2.0, 2.0], request_id="before"))
        node.commands.put(dict(action="hold", request_id="hold"))
        node.commands.put(dict(action="explore", request_id="after"))
        node.commands_tick(100.0, snap, True)
        self.assertEqual(node.follower.phase, "HOLD")
        self.assertIsNone(node.target_goal)
        self.assertGreater(node.plan_generation, old_generation)
        self.assertIn(("before", "cancelled"), replies)
        self.assertIn(("after", "cancelled"), replies)
        self.assertIn(("hold", "accepted"), replies)

    def test_expired_click_does_not_execute_late(self):
        node, _, _, _ = navigator_fixture()
        replies = []
        node.dashboard.acknowledge_command = lambda request, status, reason: replies.append(status)
        node.commands.put(dict(action="hold", request_id="expired", _deadline=99.0))
        node.commands_tick(100.0, node.feed.get()[0], True)
        self.assertEqual(replies, ["expired"])
        self.assertEqual(node.follower.phase, "NAVIGATING")

    def test_rejected_click_has_a_correlated_reason_and_does_not_reset_exploration(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap)
        node.exploration.reached = 5
        replies = []
        node.dashboard.acknowledge_command = lambda request, status, reason: replies.append(
            (request, status, reason)
        )
        node.commands.put(dict(action="explore", request_id="duplicate-session"))
        node.commands_tick(100.0, snap, True)
        self.assertEqual(node.exploration.reached, 5)
        self.assertEqual(replies[-1][0:2], ("duplicate-session", "rejected"))

    def test_mission_verification_is_not_reported_finished_before_landing(self):
        node, _, _, _ = navigator_fixture()
        node.mission.start()
        node.mission.phase = "VERIFYING"
        node.active_ui_command = dict(action="mission", request_id="landing-mission")
        node.follower.hold(node.world_pose, "Verifying enclosure")
        replies = []
        node.dashboard.acknowledge_command = lambda request, status, reason: replies.append(status)
        node.update_command_progress()
        self.assertEqual(replies, ["running"])
        self.assertIsNotNone(node.active_ui_command)

    def test_alignment_waits_for_continuous_stable_hover_and_three_fresh_scans(self):
        node, _, _, _ = navigator_fixture()
        node.alignment = None
        snap = node.feed.get()[0]
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            for i in range(3):
                node.pacer.total_s = 100.0 + i
                snap.stamp += 1
                node.capture_alignment(snap, True)
                if i < 2:
                    self.assertIsNone(node.alignment)
        self.assertIsNotNone(node.alignment)
        np.testing.assert_allclose(node.home_world_xy, snap.pose[:2])
        saved_home = node.home_world_xy.copy()
        snap.pose[0] += 2
        node.capture_alignment(snap, True)
        np.testing.assert_allclose(node.home_world_xy, saved_home)
        self.assertIsNone(navigator_fixture()[0].home_world_xy)

    def test_brief_takeoff_pause_does_not_lock_in_an_alignment(self):
        node, _, _, _ = navigator_fixture()
        node.alignment = None
        snap = node.feed.get()[0]
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            node.capture_alignment(snap, True)
            node.pacer.total_s += 1.0
            node.position.vx = 0.2
            node.capture_alignment(snap, True)
        self.assertIsNone(node.alignment_settled_since)
        self.assertEqual(node.alignment_capture_scans, 0)

    def test_exploration_retires_obsolete_destination_but_does_not_ignore_pose_faults(self):
        node, _, _, _ = navigator_fixture()
        node.exploration.start(node.feed.get()[0], continuous=True)
        node.exploration.phase = "TRAVELLING"
        self.assertFalse(
            node.retire_obsolete_goal("Current vehicle footprint is not sufficiently observed")
        )
        self.assertIsNotNone(node.target_goal)
        self.assertTrue(
            node.retire_obsolete_goal("Choose an observed destination outside the obstacle margin")
        )
        self.assertIsNone(node.target_goal)
        self.assertEqual(node.exploration.phase, "SELECTING")
        self.assertEqual(node.follower.phase, "HOLD")

    def test_small_target_change_switches_without_reference_jump_or_switching_radius(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True, simulation_s=100.0)
        node.exploration.phase = "TRAVELLING"
        node.reconsider_mark = node.pacer.total_s
        previous = node.target_goal.copy()
        new_goal = [previous[0] + 0.2, previous[1]]
        choice = ExplorationChoice(new_goal, [list(node.world_pose), new_goal], 3.0, 1.0, 20)
        reference = node.follower.target.copy()
        node.reconsider_results.put((node.plan_generation, previous, 0.2, choice, None))
        node.reconsideration_tick(snap)
        np.testing.assert_array_equal(node.follower.target, reference)
        np.testing.assert_array_equal(node.target_goal, new_goal)
        self.assertEqual(node.exploration.target_switches, 1)
        self.assertEqual(node.exploration.current_target_gain_m2, 3.0)

    def test_cancelled_reconsideration_cannot_replace_a_newer_goal(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True)
        node.exploration.phase = "TRAVELLING"
        node.reconsider_mark = node.pacer.total_s
        previous = node.target_goal.copy()
        choice = ExplorationChoice([3.0, 2.0], [list(node.world_pose), [3.0, 2.0]], 3.0, 1.0, 20)
        node.reconsider_results.put((node.plan_generation - 1, previous, 0.2, choice, None))
        node.reconsideration_tick(snap)
        np.testing.assert_array_equal(node.target_goal, previous)
        self.assertEqual(node.exploration.target_switches, 0)

    def test_same_frontier_can_update_a_stale_route_without_counting_a_target_switch(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True)
        node.exploration.phase = "TRAVELLING"
        node.reconsider_mark = node.pacer.total_s
        node.follower.begin([[2, 3], [0, 3], [0, 2], [2, 2]])
        node.follower.velocity = np.array([-0.2, 0.0])
        node.follower.speed = 0.2
        reference = node.follower.target.copy()
        choice = ExplorationChoice([2, 2], [[2, 3], [2, 2]], 3.0, 1.0, 20)
        events = []
        node.dashboard.history = SimpleNamespace(
            event=lambda kind, reason, **kw: events.append((kind, kw))
        )
        node.reconsider_results.put(
            (node.plan_generation, node.target_goal.copy(), 0.2, choice, None)
        )
        node.reconsideration_tick(snap)
        self.assertEqual(len(node.follower.path), 2)
        np.testing.assert_array_equal(node.follower.target, reference)
        np.testing.assert_array_equal(node.follower.velocity, [-0.2, 0])
        self.assertEqual(node.exploration.target_switches, 0)
        self.assertEqual(events[0][0], "route_recentered")
        self.assertIn("destination unchanged", node.exploration.reconsider_reason)

    def test_same_frontier_keeps_equivalent_route_to_avoid_replanning_churn(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True)
        node.exploration.phase = "TRAVELLING"
        node.reconsider_mark = node.pacer.total_s
        generation = node.plan_generation
        choice = ExplorationChoice([2, 2], [[2, 3], [2, 2]], 3.0, 1.0, 20)
        node.reconsider_results.put((generation, node.target_goal.copy(), 0.2, choice, None))
        node.reconsideration_tick(snap)
        self.assertEqual(node.plan_generation, generation)
        self.assertIn("competitive", node.exploration.reconsider_reason)

    def test_new_map_obstacle_rejects_reconsidered_route(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True)
        node.exploration.phase = "TRAVELLING"
        node.reconsider_mark = node.pacer.total_s
        previous = node.target_goal.copy()
        choice = ExplorationChoice([3.0, 2.0], [list(node.world_pose), [3.0, 2.0]], 3.0, 1.0, 20)
        snap.body_free[snap.model.cell([2.5, 2.5])] = False
        node.reconsider_results.put((node.plan_generation, previous, 0.2, choice, None))
        node.reconsideration_tick(snap)
        np.testing.assert_array_equal(node.target_goal, previous)
        self.assertEqual(node.exploration.target_switches, 0)
        self.assertIn("no longer observed", node.exploration.reconsider_reason)

    def test_reconsideration_computes_in_background_without_holding_existing_motion(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True)
        node.exploration.phase = "TRAVELLING"
        reference = node.follower.target.copy()
        choice = ExplorationChoice([3.0, 2.0], [list(node.world_pose), [3.0, 2.0]], 3.0, 1.0, 20)
        with patch("navigation.planning_jobs.choose_frontier", return_value=choice):
            node.launch_reconsideration(snap)
            node.reconsider_thread.join(2)
        self.assertEqual(node.follower.phase, "NAVIGATING")
        np.testing.assert_array_equal(node.follower.target, reference)
        self.assertFalse(node.reconsider_results.empty())
        self.assertEqual(node.exploration.reconsiderations, 1)

    def test_endpoint_free_strip_requires_observed_route_and_still_stops_for_obstacle(self):
        follower = RouteFollower()
        path = [[0.0, 0.0], [2.0, 0.0]]
        follower.begin(path)
        follower.tick(0.1, [0.0, 0.0], np.empty((0, 2)))
        self.assertEqual(follower.phase, "BLOCKED")
        follower.begin(path)
        follower.tick(0.1, [0.0, 0.0], np.empty((0, 2)), observed_corridor=True)
        self.assertEqual(follower.phase, "NAVIGATING")
        self.assertGreater(follower.target[0], 0.0)
        follower.tick(0.1, [0.0, 0.0], np.array([[0.6, 0.0]]), observed_corridor=True)
        self.assertEqual(follower.phase, "BLOCKED")
        np.testing.assert_array_equal(follower.target, [0.0, 0.0])

    def test_transient_map_pause_requires_three_fresh_scans_before_replanning(self):
        node, _, _, _ = navigator_fixture()
        node.mission.start()
        snap = node.feed.get()[0]
        node.stop("Temporary map gap", auto_recover=True)
        launched = []
        node.launch_plan = lambda item: launched.append(item.stamp)
        for index in range(3):
            snap.stamp += 1
            node.pacer.total_s = 100.0 + index * 0.5
            node.recovery_tick(100.0, snap, True)
            if index < 2:
                self.assertTrue(node.commands.empty())
        self.assertFalse(node.commands.empty())
        node.commands_tick(100.0, snap, True)
        self.assertTrue(launched)
        self.assertEqual(node.mission.phase, "EXPLORING")
        node.stop("Operator hold", "HOLD", True)
        self.assertFalse(node.auto_recover)

    def test_plain_exploration_recovers_map_gaps_but_not_pose_faults(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.exploration.start(snap, continuous=True)
        node.exploration.phase = "TRAVELLING"
        snap.cloud_wall = 98.9
        with patch("ros2_navigate.time.monotonic", return_value=100):
            GoalNavigator.publish_target(node)
        self.assertTrue(node.auto_recover)
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertIn("publication/cloud/clock ages", node.follower.reason)
        snap.cloud_wall = 100
        for i in range(3):
            snap.stamp += 1
            node.pacer.total_s = 100 + i * 0.5
            node.recovery_tick(100, snap, True)
        self.assertFalse(node.commands.empty())
        node.commands.get_nowait()
        node.follower.begin([[2, 3], [2, 2]])
        node.exploration.phase = "TRAVELLING"
        snap.pose[0] += 0.3
        with patch("ros2_navigate.time.monotonic", return_value=100):
            GoalNavigator.publish_target(node)
        self.assertFalse(node.auto_recover)
        self.assertIn("position error", node.follower.reason)
        for _ in range(3):
            node.recovery_tick(100, snap, True)
        self.assertTrue(node.commands.empty())
        self.assertFalse(node.mission.enabled)

    def test_imu_clock_confirmation_freezes_reference_without_cancelling_mission(self):
        node, modes, targets, _ = navigator_fixture()
        node.mission.start()
        node.pacer.confirming = 3
        self.assertFalse(node.pacer.fresh(100.0))
        self.assertTrue(node.flight_active(100.0))
        before = node.follower.target.copy()
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.mission.phase, "EXPLORING")
        np.testing.assert_array_equal(node.follower.target, before)
        self.assertTrue(modes and targets)

    def test_status_uses_simulation_age_with_real_time_and_imu_watchdogs(self):
        node, _, _, _ = navigator_fixture()
        node.status_wall = 97.0
        node.status_sim_s = 99.5
        self.assertTrue(node.flight_active(100.0))
        node.status_sim_s = 97.0
        self.assertFalse(node.flight_active(100.0))
        node.status_sim_s = 99.5
        node.status_wall = 89.0
        self.assertFalse(node.flight_active(100.0))
        node.status_wall = 99.0
        node.pacer.last_progress = 98.0
        self.assertFalse(node.flight_active(100.0))

    def test_mission_lands_only_after_completion_save_and_confirmed_px4_mode(self):
        node, modes, targets, _ = navigator_fixture()
        commands = []
        node.land_pub = SimpleNamespace(publish=commands.append, get_subscription_count=lambda: 1)
        node.mission.start()
        node.target_goal = None
        node.follower.hold(node.world_pose)
        snap = node.feed.get()[0]
        evidence = assess_completion(snap, node.world_pose)
        self.assertTrue(evidence.complete)
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            for _ in range(3):
                snap.stamp += 1
                node.pacer.total_s += 0.6
                node.plan_results.put(
                    (node.plan_generation, evidence, None, ("mission_check", snap.stamp))
                )
                GoalNavigator.publish_target(node)
            self.assertEqual(node.mission.phase, "SAVING")
            self.assertEqual(node.follower.path, [])
            self.assertEqual(commands, [])
            node.plan_results.put((node.plan_generation, "saved-map", None, "mission_saved"))
            GoalNavigator.publish_target(node)
            GoalNavigator.publish_target(node)
            self.assertEqual(len(commands), 1)
            self.assertEqual(commands[0].command, VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.assertTrue(math.isnan(commands[0].param5))
            self.assertEqual(commands[0].timestamp, 0)
            count = len(targets)
            node.status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            GoalNavigator.publish_target(node)
            self.assertEqual(len(targets), count)
            self.assertEqual(node.mission.phase, "LANDING")
            node.status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
            node.landed = True
            node.landed_wall = 100.0
            node.landed_sim_s = node.pacer.total_s
            GoalNavigator.publish_target(node)
        self.assertEqual(node.mission.phase, "LANDED")
        self.assertFalse(node.mission.enabled)

    def test_mission_map_save_error_keeps_hover_without_landing(self):
        node, modes, targets, _ = navigator_fixture()
        node.mission.start()
        node.mission.phase = "SAVING"
        node.target_goal = None
        node.follower.hold(node.world_pose)
        node.plan_results.put((node.plan_generation, None, "Disk full", "mission_saved"))
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.mission.phase, "INCOMPLETE")
        self.assertTrue(modes and targets)

    def test_stale_position_pauses_completion_and_requires_new_proof_on_resume(self):
        node, _, _, _ = navigator_fixture()
        node.mission.start()
        node.mission.phase = "SAVING"
        node.follower.hold(node.world_pose)
        node.state.last_progress = 95.0
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.mission.phase, "PAUSED")
        self.assertEqual(node.mission_resume_phase, "VERIFYING")

    def test_rviz_map_uses_world_origin_rows_and_preserves_unknown_prior(self):
        snap = snapshot()
        snap.model.grid[30, 31] = START_AREA
        message = map_message(snap, -2.98)
        self.assertEqual(message.header.frame_id, "world")
        self.assertEqual(message.info.origin.position.x, -6.0)
        self.assertEqual(message.info.origin.position.z, -2.98)
        self.assertEqual(message.data[30 * 120 + 31], -1)
        self.assertEqual(message.data[50 * 120 + 50], 0)
        self.assertEqual(message.data[15 * 120 + 50], 100)

    def test_frame_check_uses_position_at_cloud_receipt_during_motion(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.world_pose = np.array([3.0, 3.0])
        self.assertTrue(node.map_pose_matches(snap))
        self.assertEqual(node.pose_check["reason"], "matched")
        self.assertAlmostEqual(node.pose_check["position_error_m"], 0.0)
        snap.pose[0] += 1.0
        self.assertFalse(node.map_pose_matches(snap))
        self.assertEqual(node.pose_check["reason"], "position_error")
        self.assertAlmostEqual(node.pose_check["position_error_m"], 1.0)

    def test_frame_check_rejects_missing_or_old_pose_history(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        node.pose_history = deque()
        self.assertFalse(node.map_pose_matches(snap))
        self.assertEqual(node.pose_check["reason"], "missing_history")
        node.pose_history.append((99.0, [2.0, 3.0], 0.0))
        self.assertFalse(node.map_pose_matches(snap))
        self.assertEqual(node.pose_check["reason"], "history_gap")
        self.assertAlmostEqual(node.pose_check["sample_offset_s"], -1.0)

    def test_frame_check_distinguishes_heading_error_from_position_error(self):
        node, _, _, _ = navigator_fixture()
        snap = node.feed.get()[0]
        snap.pose[2] += math.radians(20)
        self.assertFalse(node.map_pose_matches(snap))
        self.assertEqual(node.pose_check["reason"], "heading_error")
        self.assertAlmostEqual(node.pose_check["heading_error_deg"], 20.0)
        self.assertAlmostEqual(node.pose_check["position_error_m"], 0.0)

    def test_frame_check_stop_records_exact_failed_sample(self):
        node, modes, targets, _ = navigator_fixture()
        snap = node.feed.get()[0]
        snap.pose[0] += 0.23
        events = []
        node.dashboard.history = SimpleNamespace(event=lambda *args, **kw: events.append(kw))
        with patch("ros2_navigate.time.monotonic", return_value=100.0):
            GoalNavigator.publish_target(node)
        self.assertEqual(node.follower.phase, "PAUSED")
        self.assertIn("position error 0.230 m > 0.200 m", node.follower.reason)
        self.assertAlmostEqual(events[0]["pose_check"]["position_error_m"], 0.23)
        self.assertEqual(events[0]["pose_check"]["px4_world_xy"], [2.0, 3.0])
        self.assertTrue(modes and targets)
        # Later successful checks must not overwrite the failed event's sample.
        snap.pose[0] -= 0.23
        self.assertTrue(node.map_pose_matches(snap))
        self.assertEqual(events[0]["pose_check"]["reason"], "position_error")

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


class MapFeedTests(unittest.TestCase):
    def test_small_clock_records_bypass_map_geometry_and_pipe_closure_invalidates_clock(self):
        feed = self.make_feed()
        feed.trajectory_clock = TrajectoryClock()
        maps = feed.process.stdout.getvalue()
        clocks = "".join(
            json.dumps(
                dict(
                    type="clock",
                    session="test",
                    frame="world",
                    generated_monotonic_s=100.0,
                    clock_ns=i * 50_000_000,
                )
            )
            + "\n"
            for i in range(8)
        )
        feed.process.stdout = StringIO(clocks + maps)
        with patch("navigation.feed.time.monotonic", return_value=100.0):
            feed._read()
        self.assertTrue(feed.snapshot.valid)
        self.assertAlmostEqual(feed.trajectory_clock.total_s, 0.2)
        self.assertFalse(feed.trajectory_clock.fresh(100.0))

    def test_unchanged_geometry_is_reused_across_new_scans(self):
        feed = self.make_feed()
        rows = [json.loads(line) for line in feed.process.stdout.getvalue().splitlines()]
        for i, row in enumerate(rows):
            row["scans"] = i + 1
        feed.process.stdout = StringIO("\n".join(json.dumps(r) for r in rows) + "\n")
        with (
            patch("navigation.feed.time.monotonic", return_value=100),
            patch(
                "navigation.map_geometry.center_geometry",
                wraps=__import__(
                    "navigation.centering", fromlist=["center_geometry"]
                ).center_geometry,
            ) as centers,
        ):
            feed._read()
        self.assertEqual(centers.call_count, 1)
        self.assertEqual(feed.snapshot.scans, 3)
        self.assertTrue(feed.snapshot.fresh(100))

    def test_pipe_preserves_confirmed_wall_evidence_for_dashboard_and_exports(self):
        feed = self.make_feed()
        rows = [json.loads(line) for line in feed.process.stdout.getvalue().splitlines()]
        for row in rows:
            row["grid"] = "#" + row["grid"][1:]
            row["confirmed_walls"] = "1" + "0" * (len(row["grid"]) - 1)
        feed.process.stdout = StringIO("\n".join(json.dumps(row) for row in rows) + "\n")
        with patch("navigation.feed.time.monotonic", return_value=100.0):
            feed._read()
        self.assertTrue(feed.snapshot.model.confirmed_walls[0, 0])
        self.assertEqual(feed.snapshot.model.wall_policy()["confirmed_cells"], 1)

    def test_pipe_rejects_confirmation_of_a_free_cell(self):
        feed = self.make_feed()
        row = json.loads(feed.process.stdout.getvalue().splitlines()[0])
        row["confirmed_walls"] = "1" + "0" * (len(row["grid"]) - 1)
        feed.process.stdout = StringIO(json.dumps(row) + "\n")
        with patch("navigation.feed.time.monotonic", return_value=100.0):
            feed._read()
        self.assertIsNone(feed.snapshot)
        self.assertIn("Confirmed walls must remain occupied", feed.failure)

    def make_feed(self, width=320):
        settings = load_settings()
        feed = MapFeed.__new__(MapFeed)
        feed.settings = settings
        from navigation.map_geometry import MapGeometry

        feed.geometry = SimpleNamespace(call=MapGeometry(settings))
        feed.lock = threading.Lock()
        feed.map_condition = threading.Condition(feed.lock)
        feed.pending_map = None
        feed.stopping = False
        feed.maps_superseded = 0
        feed.trajectory_clock = TrajectoryClock()
        # Geometry/schema tests run synchronously; concurrency has separate tests.
        feed._enqueue_map = feed._process_map
        feed.session = "test"
        feed.snapshot = None
        feed.failure = None
        feed.confirmed = 0
        feed.stamp = None
        theta = np.linspace(0, 2 * math.pi, 120, endpoint=False)
        hits = np.column_stack((-28 + 3 * np.cos(theta), -17 + 3 * np.sin(theta))).tolist()
        lines = []
        for index in range(3):
            lines.append(
                json.dumps(
                    dict(
                        session="test",
                        frame="world",
                        generated_monotonic_s=100.0,
                        width=width,
                        height=settings.map_height_cells,
                        resolution=0.2,
                        origin=[settings.map_origin_x_m, settings.map_origin_y_m],
                        grid="." * (width * settings.map_height_cells),
                        scans=1,
                        fresh=True,
                        stamp_ns=1000000000 + index,
                        pose=[-28.0, -17.0, 0.0, -2.98],
                        hits=hits,
                        cloud_age_s=0.1,
                        clock_age_s=0.1,
                        reason="Live",
                    )
                )
                + "\n"
            )
        feed.process = SimpleNamespace(stdout=StringIO("".join(lines)), poll=lambda: None)
        return feed

    def test_rectangular_pipe_preserves_geometry_and_closure_revokes_readiness(self):
        feed = self.make_feed()
        with patch("navigation.feed.time.monotonic", return_value=100.0):
            feed._read()
        self.assertEqual(feed.snapshot.model.grid.shape, (265, 320))
        self.assertEqual(feed.snapshot.model.origin.tolist(), [-34.0, -56.0])
        self.assertTrue(feed.snapshot.valid)
        self.assertEqual(feed.get()[1], "Map worker output closed")

    def test_wrong_configured_pipe_dimensions_are_rejected(self):
        feed = self.make_feed(319)
        with patch("navigation.feed.time.monotonic", return_value=100.0):
            feed._read()
        self.assertIsNone(feed.snapshot)
        self.assertIn("Unexpected map geometry", feed.failure)


class DashboardTests(unittest.TestCase):
    def test_blocked_route_preview_does_not_block_state_or_command_acknowledgement(self):
        snap = snapshot()
        commands = Queue()
        hub = Dashboard(SimpleNamespace(get=lambda: (snap, None)), commands, port=0)
        self.addCleanup(hub.close)
        started, release = threading.Event(), threading.Event()
        original = hub.preview_planning.call
        errors = []

        def blocked(item):
            started.set()
            if not release.wait(5):
                raise RuntimeError("Test did not release preview")
            return original(item)

        def preview():
            try:
                hub.plan_destination([2.0, 2.0])
            except Exception as exc:
                errors.append(exc)

        with (
            patch.object(hub.preview_planning, "call", side_effect=blocked),
            patch("navigation.dashboard.time.monotonic", return_value=100.0),
        ):
            thread = threading.Thread(target=preview)
            thread.start()
            try:
                self.assertTrue(started.wait(2))
                with urlopen(hub.url + "/api/state", timeout=2) as response:
                    self.assertTrue(json.load(response)["map_fresh"])
                request = Request(
                    hub.url + "/api/command",
                    json.dumps(dict(action="hold")).encode(),
                    headers={"Content-Type": "application/json", "X-Navigation-Token": hub.token},
                )
                # Even a stalled presentation service cannot own this small reply.
                with patch.object(
                    hub.presentation,
                    "call",
                    side_effect=AssertionError("queued behind presentation"),
                ):
                    with urlopen(request, timeout=2) as response:
                        self.assertEqual(json.load(response)["command"]["status"], "queued")
                self.assertEqual(commands.get_nowait()["action"], "hold")
                self.assertTrue(thread.is_alive())
            finally:
                release.set()
                thread.join(5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])

    def test_map_display_reuses_centerline_without_constructing_routing_costs(self):
        from navigation.report_jobs import PresentationJobs

        snap = snapshot()
        centerline = np.zeros(snap.safe.shape, dtype=bool)
        centerline[40, 40] = True
        with patch(
            "navigation.centering.routing_geometry", side_effect=AssertionError("routing cost")
        ):
            snap.centerline = centerline
            with patch(
                "navigation.centering.center_geometry",
                side_effect=AssertionError("cached geometry"),
            ):
                self.assertEqual(PresentationJobs()(("map", snap))["centerline"].count("1"), 1)
            snap.centerline = None
            with patch(
                "navigation.centering.center_geometry", return_value=(snap.clearance, centerline)
            ) as geometry:
                self.assertEqual(PresentationJobs()(("map", snap))["centerline"].count("1"), 1)
                geometry.assert_called_once()

    def test_dashboard_reports_rectangular_dimensions(self):
        snap = snapshot()
        snap.model = RayMap(**load_settings().map_options())
        snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
        hub = Dashboard(SimpleNamespace(get=lambda: (snap, None)), port=0)
        try:
            with patch("navigation.dashboard.time.monotonic", return_value=100.0):
                state = hub.state()
            self.assertEqual((state["width"], state["height"]), (320, 265))
            self.assertEqual(len(state["grid"]), 320 * 265)
        finally:
            hub.close()

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
    def test_corner_clues_do_not_override_nearest_flight_route_policy(self):
        snap = snapshot()
        model = snap.model
        model.grid[:] = UNKNOWN
        model.grid[15:105, 15:60] = FREE
        model.grid[:80, 60] = OCCUPIED
        model.grid[80:100, 60:80] = FREE  # Observed doorway; room above/right is hidden.
        model.confirmed_walls[:] = model.grid == OCCUPIED
        snap.clearance, snap.body_free, snap.safe = model.geometry()
        start = model.world((80, 40))
        before = model.grid.copy()
        choice = choose_frontier(snap, start)
        self.assertTrue(wall_endings(model.grid, model.resolution, model.confirmed_walls))
        self.assertIn("Nearest reachable", choice.selection_reason)
        self.assertTrue(snap.safe[model.cell(choice.goal)])
        self.assertTrue(
            all(
                segment_safe(snap.safe, model.cell(a), model.cell(b))
                for a, b in zip(choice.path, choice.path[1:])
            )
        )
        np.testing.assert_array_equal(model.grid, before)
        model.confirmed_walls[:] = False
        fallback = choose_frontier(snap, start)
        self.assertIsNone(fallback.corner_xy)
        self.assertAlmostEqual(fallback.route_length_m, choice.route_length_m)

    def test_mapped_room_does_not_remain_a_hidden_corner_opportunity(self):
        snap = snapshot()
        model = snap.model
        model.grid[30:80, 60] = OCCUPIED
        model.confirmed_walls[:] = model.grid == OCCUPIED
        snap.clearance, snap.body_free, snap.safe = model.geometry()
        self.assertEqual(wall_endings(model.grid, model.resolution, model.confirmed_walls), [])
        with self.assertRaisesRegex(ValueError, "No observed free/unknown frontier remains"):
            choose_frontier(snap, model.world((80, 40)))

    def test_wall_ending_detection_is_rotation_independent_and_requires_confirmation(self):
        grid = np.full((80, 80), UNKNOWN, dtype=np.uint8)
        grid[10:70, 10:40] = FREE
        grid[20:40, 40] = OCCUPIED
        for turns in range(4):
            rotated = np.rot90(grid, turns)
            tips = wall_endings(rotated, 0.2, rotated == OCCUPIED)
            self.assertEqual(len(tips), 2)
            self.assertTrue(all(length >= 0.6 for _, _, length in tips))
            self.assertEqual(wall_endings(rotated, 0.2, np.zeros_like(rotated, bool)), [])

    def test_wall_middle_noise_and_fully_observed_end_are_not_hidden_passage_clues(self):
        grid = np.full((80, 80), UNKNOWN, dtype=np.uint8)
        grid[:, :40] = FREE
        grid[:, 40] = OCCUPIED
        self.assertEqual(wall_endings(grid, 0.2, grid == OCCUPIED), [])
        grid[40, 20] = OCCUPIED
        self.assertEqual(wall_endings(grid, 0.2, grid == OCCUPIED), [])
        grid[:, :] = FREE
        grid[20:40, 40] = OCCUPIED
        self.assertEqual(wall_endings(grid, 0.2, grid == OCCUPIED), [])

    def test_diagonal_wall_tip_is_detected_without_axis_assumption(self):
        grid = np.full((80, 80), UNKNOWN, dtype=np.uint8)
        yy, xx = np.indices(grid.shape)
        grid[xx < yy] = FREE
        for i in range(20, 41):
            grid[i, i] = OCCUPIED
        tips = wall_endings(grid, 0.1, grid == OCCUPIED)
        self.assertEqual(len(tips), 2)
        self.assertTrue(
            all(abs(direction[0]) > 0.6 and abs(direction[1]) > 0.6 for _, direction, _ in tips)
        )

    def test_prefers_nearest_frontier_by_planned_route_length(self):
        snap = frontier_snapshot()
        choice = choose_frontier(snap, [-2.0, 0.0])
        self.assertLess(abs(choice.goal[1]), 0.5)
        self.assertLess(choice.route_length_m, 2.0)
        self.assertIn("Nearest reachable", choice.selection_reason)
        self.assertAlmostEqual(
            choice.route_length_m,
            sum(math.dist(a, b) for a, b in zip(choice.path, choice.path[1:])),
        )

    def test_travel_history_does_not_override_nearest_unresolved_frontier(self):
        snap = frontier_snapshot()
        first = choose_frontier(snap, [-2.0, 0.0])
        trail = np.linspace([-2.0, 0.0], first.goal, 50).tolist()
        second = choose_frontier(snap, [-2.0, 0.0], trail=trail)
        np.testing.assert_allclose(first.goal, second.goal)
        self.assertLess(second.history_distance_m, 0.01)
        self.assertTrue(snap.safe[snap.model.cell(second.goal)])

    def test_trail_records_measured_progress_and_ignores_stationary_updates(self):
        session = ExplorationSession()
        session.start(frontier_snapshot())
        session.record_pose([-1.99, 0.0])
        self.assertEqual(len(session.trail), 1)
        session.record_pose([-1.7, 0.0])
        self.assertEqual(session.trail[-1], [-1.7, 0.0])
        session.cancel("Operator hold")
        session.record_pose([-1.0, 0.0])
        self.assertEqual(len(session.trail), 2)

    def test_oversized_or_nonfinite_map_settings_are_rejected(self):
        for settings in (
            NavigationSettings(map_width_cells=1024, map_height_cells=1024),
            NavigationSettings(map_origin_x_m=float("nan")),
            NavigationSettings(yaw_rate_deg_s=100),
            NavigationSettings(yaw_acceleration_deg_s2=200),
            NavigationSettings(route_clearance_weight=21),
            NavigationSettings(route_clearance_scale_m=11),
        ):
            with self.assertRaises(ValueError):
                settings.validate()

    def test_frontiers_only_use_observed_free_not_start_prior_or_obstacles(self):
        grid = np.array([[OCCUPIED, FREE, UNKNOWN], [FREE, START_AREA, UNKNOWN]], dtype=np.uint8)
        edge = frontiers(grid)
        self.assertTrue(edge[0, 1])
        self.assertFalse(edge[1, 1])
        self.assertFalse(edge[0, 0])

    def test_measures_actual_observations_and_travel_during_motion(self):
        snap = frontier_snapshot()
        session = ExplorationSession()
        session.start(snap, continuous=True, simulation_s=10.0)
        session.record_pose([-1.0, 0.0])
        session.record_pose([-1.0, 1.0])
        snap.model.grid[30:40, 60:70] = FREE
        session.observe(snap, 15.0)
        self.assertAlmostEqual(session.distance_flown_m, 2.0)
        self.assertAlmostEqual(session.area_gain_m2, 1.0)
        self.assertEqual(session.elapsed_s, 5.0)
        self.assertEqual(session.reached, 0)

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
        self.assertEqual(load_settings().cruise_m_s, 1.2)
        with self.assertRaises(ValueError):
            NavigationSettings(stop_distance_m=0.5).validate()
        with self.assertRaises(ValueError):
            NavigationSettings(cruise_m_s=4.01).validate()

    def test_controller_arrival_waits_for_new_scan_then_selects_next_viewpoint(self):
        node, modes, targets, _ = navigator_fixture()
        node.trajectory_clock.consume = lambda now: 0.05
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

    def test_plain_frontier_exhaustion_checks_enclosure_before_reporting_complete(self):
        for unresolved in (False, True):
            with self.subTest(unresolved=unresolved):
                node, modes, targets, _ = navigator_fixture()
                snap = node.feed.get()[0]
                if unresolved:
                    snap.model.grid[30, 30] = UNKNOWN
                node.exploration.start(snap)
                node.launch_frontier(snap)
                node.plan_thread.join(5)
                self.assertFalse(node.plan_thread.is_alive())
                with patch("ros2_navigate.time.monotonic", return_value=100.0):
                    GoalNavigator.publish_target(node)
                self.assertEqual(node.exploration.phase, "FINISHED" if unresolved else "COMPLETE")
                self.assertEqual(node.follower.phase, "HOLD" if unresolved else "COMPLETE")
                self.assertFalse(node.exploration.enabled)
                self.assertIsNone(node.target_goal)
                self.assertTrue(modes and targets)

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
