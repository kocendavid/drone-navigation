"""Exposure-aware wall confirmation; no live sensor or flight commands."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from lidar_tools.scan_motion import confirmation_quality, SCAN_PERIOD_S
from lidar_tools.sensor_config import (
    configure_sensor,
    SENSOR_VALUES,
    RENDERER_VALUES,
    SCAN_EXPOSURE_S,
)
from navigation.mapping import RayMap, FREE, OCCUPIED


def pose(degrees=0, x=0):
    angle = math.radians(degrees)
    return (
        np.array(
            [
                [math.cos(angle), -math.sin(angle), 0],
                [math.sin(angle), math.cos(angle), 0],
                [0, 0, 1],
            ]
        ),
        np.array([x, 0, 0.0]),
    )


class ScanMotionTests(unittest.TestCase):
    def quality(self, poses):
        return confirmation_quality(poses, resolution=0.2, mapping_range=12)

    def test_low_motion_allows_confirmation_and_translation_uses_the_same_budget(self):
        self.assertTrue(self.quality([pose()] * 5)["allowed"])
        self.assertTrue(self.quality([pose(x=x) for x in np.linspace(0, 0.09, 5)])["allowed"])
        translated = self.quality([pose(x=x) for x in np.linspace(0, 0.11, 5)])
        self.assertFalse(translated["allowed"])
        self.assertAlmostEqual(translated["motion_bound_m"], 0.11)

    def test_rotation_is_bounded_at_the_mapping_horizon_not_at_the_body(self):
        rotating = self.quality([pose(a) for a in np.linspace(0, 6, 5)])
        self.assertFalse(rotating["allowed"])
        self.assertAlmostEqual(rotating["motion_bound_m"], 12 * math.radians(6))
        self.assertTrue(self.quality([pose(a) for a in np.linspace(179.9, 180.1, 5)])["allowed"])

    def test_turn_and_return_cannot_hide_motion_with_matching_endpoints(self):
        self.assertFalse(self.quality([pose(a) for a in (0, 1, 2, 1, 0)])["allowed"])
        self.assertFalse(self.quality([pose(x=x) for x in (0, 0.04, 0.08, 0.04, 0)])["allowed"])

    def test_missing_invalid_and_reflected_transform_history_cannot_confirm(self):
        for poses in (
            [],
            [pose()] * 4,
            [(np.eye(3) * 2, np.zeros(3))] * 5,
            [(np.eye(3), np.full(3, np.nan))] * 5,
            [(np.diag([-1, 1, 1]), np.zeros(3))] * 5,
        ):
            self.assertFalse(self.quality(poses)["allowed"])

    def test_repeated_moving_hits_block_immediately_but_clear_after_good_free_rays(self):
        model = RayMap()
        angles = np.linspace(0, 2 * math.pi, 1440, endpoint=False)
        circle = lambda r: np.column_stack(
            [r * np.cos(angles), r * np.sin(angles), np.zeros(len(angles))]
        )
        cell = model.cell([2, 0])
        for _ in range(20):
            model.update(
                circle(2), np.eye(3), np.zeros(3), [0, 0], 0, allow_wall_confirmation=False
            )
            self.assertEqual(model.grid[cell], OCCUPIED)
            self.assertFalse(model.confirmed_walls.any())
        self.assertFalse(model.wall_hit_scans.any())
        model.update(circle(4), np.eye(3), np.zeros(3), [0, 0], 0)
        self.assertEqual(model.grid[cell], FREE)

    def test_uncertified_scan_breaks_pending_votes_but_preserves_existing_real_walls(self):
        model = RayMap()
        angles = np.linspace(0, 2 * math.pi, 1440, endpoint=False)
        points = np.column_stack([2 * np.cos(angles), 2 * np.sin(angles), np.zeros(len(angles))])
        for _ in range(2):
            model.update(points, np.eye(3), np.zeros(3), [0, 0], 0)
        model.update(points, np.eye(3), np.zeros(3), [0, 0], 0, allow_wall_confirmation=False)
        self.assertFalse(model.wall_hit_scans.any())
        for _ in range(2):
            model.update(points, np.eye(3), np.zeros(3), [0, 0], 0)
        self.assertFalse(model.confirmed_walls.any())
        model.update(points, np.eye(3), np.zeros(3), [0, 0], 0)
        confirmed = model.confirmed_walls.copy()
        self.assertTrue(confirmed.any())
        model.update(points * 2, np.eye(3), np.zeros(3), [0, 0], 0, allow_wall_confirmation=False)
        np.testing.assert_array_equal(model.confirmed_walls, confirmed)
        self.assertTrue((model.grid[confirmed] == OCCUPIED).all())

    def test_worker_requires_capture_time_history_and_withholds_votes_when_it_is_missing(self):
        from tools.navigation_map_worker import MapWorker
        from rclpy.time import Time

        calls = []

        def lookup(world, frame, stamp):
            calls.append((world, frame, stamp.nanoseconds))
            return pose()

        worker = SimpleNamespace(
            map=RayMap(resolution=0.2), buffer=SimpleNamespace(lookup_transform=lookup)
        )
        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=Time(nanoseconds=1_000_000_000).to_msg(), frame_id="point_cloud"
            )
        )
        with (
            patch("tools.navigation_map_worker.SCAN_EXPOSURE_S", SCAN_PERIOD_S),
            patch("tools.navigation_map_worker.transform_parts", side_effect=lambda value: value),
        ):
            quality = MapWorker.wall_confirmation_quality(worker, message, *pose())
        self.assertTrue(quality["allowed"])
        self.assertEqual(
            [entry[2] for entry in calls], [900_000_000, 925_000_000, 950_000_000, 975_000_000]
        )
        self.assertTrue(all(row[:2] == ("world", "point_cloud") for row in calls))
        worker.buffer.lookup_transform = lambda *args: (_ for _ in ()).throw(RuntimeError("No TF"))
        with patch("tools.navigation_map_worker.SCAN_EXPOSURE_S", SCAN_PERIOD_S):
            self.assertFalse(
                MapWorker.wall_confirmation_quality(worker, message, *pose())["allowed"]
            )

    def test_snapshot_worker_confirms_moving_wall_without_inter_scan_history(self):
        from tools.navigation_map_worker import MapWorker

        model = RayMap(resolution=0.2)
        lookup = lambda *args: (_ for _ in ()).throw(AssertionError("No historical TF needed"))
        worker = SimpleNamespace(map=model, buffer=SimpleNamespace(lookup_transform=lookup))
        angles = np.linspace(0, 2 * math.pi, 1440, endpoint=False)
        world_wall = np.column_stack(
            [3 * np.cos(angles), 3 * np.sin(angles), np.zeros(len(angles))]
        )
        self.assertEqual(SCAN_EXPOSURE_S, 0.0)
        self.assertIs(SENSOR_VALUES["omni:sensor:Core:instantLidar"], True)
        for i in range(3):
            rotation, translation = pose(degrees=6 * i, x=0.12 * i)
            quality = MapWorker.wall_confirmation_quality(worker, None, rotation, translation)
            self.assertTrue(quality["allowed"])
            points = (world_wall - translation) @ rotation
            model.update(
                points,
                rotation,
                translation,
                translation[:2],
                0,
                allow_wall_confirmation=quality["allowed"],
            )
        self.assertGreater(int(model.confirmed_walls.sum()), 50)
        self.assertTrue((model.grid[model.confirmed_walls] == OCCUPIED).all())

    def test_snapshot_requires_valid_capture_pose_and_unknown_exposure_cannot_confirm(self):
        from tools.navigation_map_worker import MapWorker

        worker = SimpleNamespace(map=RayMap(resolution=0.2))
        for rotation, translation in (
            (np.eye(3) * 2, np.zeros(3)),
            (np.eye(3), np.full(3, np.nan)),
        ):
            self.assertFalse(
                MapWorker.wall_confirmation_quality(worker, None, rotation, translation)["allowed"]
            )
        for exposure in (None, -0.1, float("nan")):
            self.assertFalse(
                confirmation_quality([pose()], 0.2, 12, exposure_s=exposure)["allowed"]
            )

    def test_grazing_ray_near_hit_preserves_tentative_cell_but_clear_through_ray_removes_it(self):
        for resolution in (0.1, 0.2):
            with self.subTest(resolution=resolution):
                model = RayMap(resolution=resolution, size=120, origin=(-6, -6))
                angles = np.linspace(0, 2 * math.pi, 1440, endpoint=False)
                circle = lambda r: np.column_stack(
                    [r * np.cos(angles), r * np.sin(angles), np.zeros(len(angles))]
                )
                model.update(
                    circle(2), np.eye(3), np.zeros(3), [0, 0], 0, allow_wall_confirmation=False
                )
                cell = model.cell([2, 0])
                old_obstacles = model.grid == OCCUPIED
                neighbour = np.array([[2 + resolution / 2, resolution * 1.5, 0]])
                model.update(
                    np.vstack([circle(4), neighbour]),
                    np.eye(3),
                    np.zeros(3),
                    [0, 0],
                    0,
                    allow_wall_confirmation=False,
                )
                self.assertEqual(model.grid[cell], OCCUPIED)
                self.assertFalse(model.confirmed_walls.any())
                self.assertFalse(model.wall_hit_scans.any())
                # The protection adds neither synthetic obstacles nor hit votes.
                new_obstacles = (model.grid == OCCUPIED) & ~old_obstacles
                expected_hits = np.zeros_like(new_obstacles)
                for point in np.vstack([circle(4), neighbour]):
                    expected_hits[model.cell(point[:2])] = True
                self.assertFalse((new_obstacles & ~expected_hits).any())
                model.update(
                    circle(4), np.eye(3), np.zeros(3), [0, 0], 0, allow_wall_confirmation=False
                )
                self.assertEqual(model.grid[cell], FREE)


class SensorConfigurationTests(unittest.TestCase):
    def fixtures(self):
        class Attribute:
            def __init__(self, value):
                self.value = value

            def IsValid(self):
                return True

            def Get(self):
                return self.value

            def Set(self, value):
                self.value = value
                return True

        attrs = {name: Attribute(value) for name, value in SENSOR_VALUES.items()}
        attrs["omni:sensor:Core:nearRangeM"] = Attribute(0.5)
        attrs["omni:sensor:Core:outputMotionCompensationState"].value = "NONCOMPENSATED"
        attrs["omni:sensor:Core:instantLidar"].value = False
        sensor = SimpleNamespace(GetAttribute=attrs.__getitem__)
        settings = {}
        renderer = SimpleNamespace(get=settings.get, set=settings.__setitem__)
        return sensor, renderer, attrs, settings

    def test_running_timeline_is_rejected_before_any_sensor_or_renderer_change(self):
        sensor, renderer, attrs, settings = self.fixtures()
        with self.assertRaisesRegex(RuntimeError, "stopped timeline"):
            configure_sensor(sensor, renderer, stopped=False, near_range=0.5)
        self.assertEqual(settings, {})
        self.assertEqual(
            attrs["omni:sensor:Core:outputMotionCompensationState"].Get(), "NONCOMPENSATED"
        )

    def test_compensated_output_retains_sensor_frame_and_enables_renderer_motion(self):
        sensor, renderer, attrs, settings = self.fixtures()
        report = configure_sensor(sensor, renderer, stopped=True, near_range=0.5)
        self.assertEqual(
            attrs["omni:sensor:Core:outputMotionCompensationState"].Get(), "COMPENSATED"
        )
        self.assertEqual(attrs["omni:sensor:Core:outputFrameOfReference"].Get(), "SENSOR")
        self.assertIs(attrs["omni:sensor:Core:instantLidar"].Get(), True)
        self.assertEqual(settings, RENDERER_VALUES)
        self.assertEqual(
            report["previous"]["sensor"]["omni:sensor:Core:outputMotionCompensationState"],
            "NONCOMPENSATED",
        )
        self.assertIn("required", report["validation"])

    def test_invalid_near_range_leaves_settings_untouched(self):
        sensor, renderer, _, settings = self.fixtures()
        with self.assertRaises(ValueError):
            configure_sensor(sensor, renderer, stopped=True, near_range=float("nan"))
        self.assertEqual(settings, {})


if __name__ == "__main__":
    unittest.main()
