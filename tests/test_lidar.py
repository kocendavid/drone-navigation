"""Parser cases and isolated real ROS delivery; never publishes real /clock or /fmu."""

import argparse
import array
import json
import os
from pathlib import Path
import struct
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from lidar_tools.cloud import cloud_summary, xyz_array
from tools.lidar_probe import LidarProbe


def make_cloud(points, *, endian="<", row_padding=0):
    """Two points per row; XYZ deliberately reordered with an unused field."""
    cloud = PointCloud2()
    cloud.header = Header(frame_id="test_lidar")
    cloud.header.stamp.sec = 100
    cloud.height = len(points) // 2
    cloud.width = 2
    cloud.point_step = 16
    cloud.row_step = 32 + row_padding
    cloud.is_bigendian = endian == ">"
    cloud.is_dense = True  # Decoder must still detect invalid points.
    cloud.fields = [
        PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
        for n, o in (("x", 8), ("y", 0), ("z", 12), ("intensity", 4))
    ]
    data = bytearray()
    for index, (x, y, z) in enumerate(points):
        data.extend(struct.pack(endian + "ffff", y, 9.0, x, z))
        if index % 2 == 1:
            data.extend(b"\xaa" * row_padding)
    cloud.data = array.array("B", data)
    return cloud


class CloudTests(unittest.TestCase):
    def test_padded_rows_and_big_endian(self):
        expected = [(1, 2, 3), (4, 5, 6), (-1, -2, -3), (0, 0, 0)]
        for endian in ("<", ">"):
            cloud = make_cloud(expected, endian=endian, row_padding=8)
            before = bytes(cloud.data)
            np.testing.assert_array_equal(xyz_array(cloud), expected)
            self.assertEqual(bytes(cloud.data), before)

    def test_invalid_and_range_filter_even_when_dense(self):
        cloud = make_cloud([(1, 0, 0), (0, 0, 0), (float("nan"), 1, 2), (float("inf"), 0, 0)])
        result = cloud_summary(cloud, 0.2, 30)
        self.assertEqual(
            (result["finite_points"], result["invalid_points"], result["in_range_points"]),
            (2, 2, 1),
        )
        self.assertEqual(result["xyz_min_m"], [1, 0, 0])

    def test_malformed_layout_rejected(self):
        for mutation in ("short_data", "bad_row", "bad_offset", "missing_z"):
            cloud = make_cloud([(1, 0, 0), (2, 0, 0)])
            if mutation == "short_data":
                cloud.data.pop()
            elif mutation == "bad_row":
                cloud.row_step = 1
            elif mutation == "bad_offset":
                cloud.fields[0].offset = 16
            else:
                cloud.fields.pop(2)
            with self.assertRaises(ValueError, msg=mutation):
                xyz_array(cloud)

    def test_empty_cloud(self):
        cloud = PointCloud2()
        self.assertEqual(cloud_summary(cloud, 0.2, 30)["in_range_points"], 0)

    def test_float64_and_extreme_ranges_remain_json_serializable(self):
        cloud = make_cloud([(1, 2, 3), (4, 5, 6)])
        cloud.point_step = 32
        cloud.row_step = 64
        for field in cloud.fields:
            field.offset *= 2
            field.datatype = PointField.FLOAT64
        cloud.data = array.array("B", struct.pack("<dddddddd", 2, 9, 1, 3, 0, 9, 1e300, 0))
        result = cloud_summary(cloud, 0.2, 30)
        self.assertEqual(result["in_range_points"], 1)
        self.assertEqual(result["nonfinite_range_points"], 1)
        json.dumps(result, allow_nan=False)


class RosProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.prefix = f"/_isaac_lidar_test_{os.getpid()}"
        rclpy.init(
            args=[
                "--ros-args",
                "-r",
                f"/clock:={cls.prefix}/clock",
                "-r",
                f"/tf:={cls.prefix}/tf",
                "-r",
                f"/tf_static:={cls.prefix}/tf_static",
                "-r",
                f"/fmu/out/vehicle_local_position_v1:={cls.prefix}/position",
            ]
        )

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        args = argparse.Namespace(
            topic=self.prefix + "/points",
            min_range=0.2,
            max_range=30,
            stale_after=1.0,
            min_hz=2.0,
            max_clock_skew=0.5,
            target_frame="test_body",
        )
        self.probe = LidarProbe(args)
        self.source = Node("isolated_lidar_test_source")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.probe)
        self.executor.add_node(self.source)

    def tearDown(self):
        self.executor.shutdown()
        self.probe.destroy_node()
        self.source.destroy_node()

    def test_no_data_fails_without_sim_clock(self):
        report = self.probe.report()
        self.assertFalse(report["delivery_pass"])
        self.assertFalse(report["frame_time_pass"])
        self.assertEqual(report["cloud_messages"], 0)

    def test_capture_time_tf_can_arrive_after_cloud_and_missing_tf_expires(self):
        cloud = make_cloud([(2, 0, 0), (3, 0, 0)])
        self.probe.on_cloud(cloud)
        self.assertEqual(len(self.probe.tf_pending), 1)
        # A transform delivered later should satisfy the original acquisition stamp.
        transform = TransformStamped()
        transform.header.frame_id = "test_body"
        transform.child_frame_id = "test_lidar"
        transform.transform.translation.z = 0.06
        transform.transform.rotation.w = 1.0
        self.probe.buffer.set_transform_static(transform, "delayed_test_source")
        self.probe.resolve_pending_tf()
        self.assertEqual(
            (self.probe.tf_ok, self.probe.tf_failed, len(self.probe.tf_pending)), (1, 0, 0)
        )
        # Missing TF must fail after the wall-time grace, even if sim time is frozen.
        cloud.header.frame_id = "missing_test_sensor"
        self.probe.on_cloud(cloud)
        frame, stamp, arrived = self.probe.tf_pending[0]
        self.probe.tf_pending[0] = (frame, stamp, arrived - 1.0)
        self.probe.resolve_pending_tf()
        self.assertEqual(self.probe.tf_failed, 1)
        self.assertFalse(self.probe.tf_pending)

    def test_real_delivery_clock_tf_and_stale_handling(self):
        points = self.source.create_publisher(
            PointCloud2, self.prefix + "/points", qos_profile_sensor_data
        )
        clock = self.source.create_publisher(Clock, self.prefix + "/clock", qos_profile_sensor_data)
        tf_pub = self.source.create_publisher(
            TFMessage,
            self.prefix + "/tf_static",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        transform = TransformStamped()
        transform.header.frame_id = "test_body"
        transform.child_frame_id = "test_lidar"
        transform.transform.rotation.w = 1.0
        sequence = 0
        deadline = time.monotonic() + 6.0
        next_send = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_send:
                sequence += 1
                cloud = make_cloud([(2, 0, 0), (3, 1, 0)], row_padding=8)
                cloud.header.stamp.sec = 100 + sequence // 10
                cloud.header.stamp.nanosec = (sequence % 10) * 100_000_000
                tick = Clock(clock=cloud.header.stamp)
                tf_pub.publish(TFMessage(transforms=[transform]))
                clock.publish(tick)
                # Allow static TF and clock to arrive before the first cloud.
                if sequence > 5:
                    points.publish(cloud)
                next_send = time.monotonic() + 0.1
            self.executor.spin_once(timeout_sec=0.01)
            if self.probe.tf_ok >= 8:
                break
        report = self.probe.report()
        self.assertTrue(report["delivery_pass"], report)
        self.assertTrue(report["frame_time_pass"], report)
        self.assertEqual(report["capture_time_tf_failures"], 0)
        self.assertGreaterEqual(report["arrival_hz"], 2)
        # A paused simulation must become stale using wall time.
        self.probe.arrivals = [t - 2.0 for t in self.probe.arrivals]
        self.assertFalse(self.probe.report()["delivery_pass"])
        # Repeated acquisition stamps are not a healthy stream.
        now = time.monotonic()
        self.probe.arrivals = [now - 0.2, now - 0.1, now]
        self.probe.stamps = [100_000_000_000] * 3
        self.probe.usable_clouds = 3
        self.assertFalse(self.probe.report()["delivery_pass"])
        # A time reset is visible and prevents frame/time readiness.
        self.probe.on_clock(Clock())
        self.assertGreater(self.probe.clock_backwards, 0)
        self.assertFalse(self.probe.report()["frame_time_pass"])


if __name__ == "__main__":
    unittest.main()
