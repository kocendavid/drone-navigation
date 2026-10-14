"""Meaningful wall/floor, coordinate, invalid-data and watchdog cases."""

import argparse
from dataclasses import replace
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from lidar_tools.clearance import (
    ClearanceLimits,
    estimate_flat_floor,
    forward_surfaces,
    health_issue,
)


class SurfaceTests(unittest.TestCase):
    def measure(self, points, limits=None, body_rotation=None):
        return forward_surfaces(
            points,
            np.eye(3) if body_rotation is None else body_rotation,
            [0, 0, 0.059],
            np.eye(3),
            [0, 0, 0.083],
            limits or ClearanceLimits(),
        )

    def test_floor_ceiling_side_and_rear_excluded_wall_retained(self):
        report, filtered = self.measure(
            [[1.5, 0, -0.083], [1.2, 0, 1.7], [1.5, 0.8, 0.2], [-2, 0, 0.2], [3, 0, 0.2]]
        )
        self.assertEqual(report["state"], "CLEAR")
        self.assertEqual(report["forward_distance_m"], 3)
        self.assertEqual(report["floor_excluded_points"], 1)
        np.testing.assert_allclose(filtered, [[3, 0, 0.259]])

    def test_hold_at_threshold_and_minimum_preserved_during_voxelization(self):
        limits = replace(ClearanceLimits(), voxel_m=0.3)
        report, reduced = self.measure([[2.09, 0, 0.2], [1.99, 0, 0.2]], limits)
        self.assertEqual(report["state"], "HOLD")
        self.assertAlmostEqual(report["forward_distance_m"], 1.99)
        self.assertEqual(len(reduced), 1)
        self.assertEqual(self.measure([[2, 0, 0.2]])[0]["state"], "HOLD")

    def test_empty_invalid_and_out_of_range_are_unknown(self):
        for points in (
            [],
            [[np.nan, 0, 0]],
            [[np.inf, 0, 0]],
            [[0.2, 0, 0.2]],
            [[31, 0, 0.2]],
            [[3, 2, 0.2]],
        ):
            report, _ = self.measure(points)
            self.assertEqual(report["state"], "UNKNOWN")
            self.assertIsNone(report["forward_distance_m"])

    def test_rotated_sensor_uses_body_forward_not_sensor_x(self):
        rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        report, _ = self.measure([[0, -1.5, 0.2]], body_rotation=rotation)
        self.assertEqual(report["state"], "HOLD")
        self.assertEqual(report["forward_distance_m"], 1.5)

    def test_invalid_configuration_rejected(self):
        for kwargs in ({"floor_z_world_m": np.nan}, {"hold_m": -1}, {"voxel_m": 0}):
            with self.assertRaises(ValueError):
                ClearanceLimits(**kwargs)

    def test_floor_calibration_below_world_origin_and_rejects_wall_or_slope(self):
        x, y = np.meshgrid(np.linspace(-2, 2, 50), np.linspace(-2, 2, 50))
        floor = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -0.78)))
        estimate = estimate_flat_floor(floor)
        self.assertAlmostEqual(estimate["height_world_m"], -0.78)
        wall = floor.copy()
        wall[:, 0] = 3
        with self.assertRaises(ValueError):
            estimate_flat_floor(wall)
        slope = floor.copy()
        slope[:, 2] += slope[:, 0] * 0.1
        with self.assertRaises(ValueError):
            estimate_flat_floor(slope)

    def test_wall_time_watchdog_and_clock_skew(self):
        healthy = dict(
            now_wall=10.0,
            arrival_wall=9.9,
            clock_progress_wall=9.9,
            clock_ns=2_100_000_000,
            cloud_ns=2_000_000_000,
        )
        self.assertIsNone(health_issue(**healthy))
        for changes in (
            {"arrival_wall": 8},
            {"clock_progress_wall": 8},
            {"cloud_ns": 0},
            {"clock_ns": 3_000_000_000},
        ):
            self.assertIsNotNone(health_issue(**{**healthy, **changes}))


class NodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import os
        import rclpy

        cls.rclpy = rclpy
        cls.prefix = f"/_clearance_test_{os.getpid()}"
        remaps = []
        for topic in (
            "/clock",
            "/tf",
            "/tf_static",
            "/navigation/forward_clearance",
            "/navigation/forward_points",
            "/navigation/clearance_markers",
        ):
            remaps += ["-r", f"{topic}:={cls.prefix}{topic}"]
        rclpy.init(args=["--ros-args", *remaps])

    @classmethod
    def tearDownClass(cls):
        cls.rclpy.shutdown()

    def setUp(self):
        from tools.forward_clearance import ForwardClearance

        self.node = ForwardClearance(
            argparse.Namespace(
                topic=self.prefix + "/points", sensor_frame="test_lidar", floor_z=0.0, output=None
            )
        )

    def tearDown(self):
        self.node.destroy_node()

    def send_wall(self, distance=3):
        from geometry_msgs.msg import TransformStamped
        from builtin_interfaces.msg import Time
        from rosgraph_msgs.msg import Clock
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Header

        stamp = Time(sec=10)
        self.node.on_clock(Clock(clock=stamp))
        for parent, child, height in (
            ("world", "chassis", 0.024),
            ("chassis", "test_lidar", 0.059),
        ):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.rotation.w = 1.0
            transform.transform.translation.z = height
            self.node.buffer.set_transform_static(transform, "isolated_test_mount")
        cloud = create_cloud_xyz32(Header(frame_id="test_lidar", stamp=stamp), [[distance, 0, 0.2]])
        self.node.on_cloud(cloud)
        self.node.process_pending()
        return cloud

    def test_wall_hold_and_missing_data(self):
        self.assertEqual(self.node.snapshot()["state"], "UNKNOWN")
        self.send_wall(1.5)
        self.assertEqual(self.node.snapshot()["state"], "HOLD")
        self.node.publish_status()

    def test_auto_floor_refuses_insufficient_evidence(self):
        self.node.args.floor_z = None
        self.node.floor_ready = False
        self.send_wall()
        report = self.node.snapshot()
        self.assertEqual(report["state"], "UNKNOWN")
        self.assertTrue(report["reason"].startswith("Floor calibration:"))
        self.assertEqual(self.node.counts["decode_errors"], 0)

    def test_stale_clock_reset_and_duplicate_invalidate_previous_clear(self):
        from builtin_interfaces.msg import Time
        from rosgraph_msgs.msg import Clock

        cloud = self.send_wall()
        self.assertEqual(self.node.snapshot()["state"], "CLEAR")
        self.node.clock_progress_wall = time.monotonic() - 2
        self.assertEqual(self.node.snapshot()["state"], "UNKNOWN")
        self.node.clock_progress_wall = time.monotonic()
        self.node.on_cloud(cloud)
        self.assertEqual(self.node.snapshot()["state"], "UNKNOWN")
        self.node.on_clock(Clock(clock=Time(sec=1)))
        self.assertEqual(self.node.counts["clock_resets"], 1)
        self.assertIsNone(self.node.last_seen_stamp)

    def test_missing_capture_time_tf_stays_unknown(self):
        from builtin_interfaces.msg import Time
        from rosgraph_msgs.msg import Clock
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Header

        stamp = Time(sec=10)
        self.node.on_clock(Clock(clock=stamp))
        cloud = create_cloud_xyz32(Header(frame_id="test_lidar", stamp=stamp), [[3, 0, 0.2]])
        self.node.on_cloud(cloud)
        self.node.pending = (cloud, time.monotonic() - 0.4)
        self.node.process_pending()
        self.assertEqual(self.node.snapshot()["state"], "UNKNOWN")
        self.assertEqual(self.node.counts["tf_timeouts"], 1)


if __name__ == "__main__":
    unittest.main()
