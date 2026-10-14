"""Observable clearance recommendations only; no PX4 or flight publishers."""

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from lidar_tools.cloud import xyz_array
from lidar_tools.clearance import (
    ClearanceLimits,
    estimate_flat_floor,
    forward_surfaces,
    health_issue,
)
from px4_isaac.model import rotation_wxyz


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def transform_parts(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    translation = np.array([t.x, t.y, t.z])
    if not np.isfinite(translation).all():
        raise ValueError("Invalid TF translation")
    return rotation_wxyz([q.w, q.x, q.y, q.z]), translation


class ForwardClearance(Node):
    def __init__(self, args):
        super().__init__(
            "isaac_forward_clearance", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.args = args
        self.limits = ClearanceLimits(
            floor_z_world_m=args.floor_z if args.floor_z is not None else 0.0
        )
        self.floor_ready = args.floor_z is not None
        self.floor_samples = []
        self.floor_calibration = {
            "source": (
                "Configured floor height"
                if self.floor_ready
                else "Waiting for five stationary scans"
            )
        }
        self.clock_ns = self.clock_progress_wall = None
        self.last_seen_stamp = None
        self.pending = None
        self.result = None
        self.reason = "Waiting for cloud, clock and transforms"
        self.counts = {
            "clouds_received": 0,
            "clouds_processed": 0,
            "tf_timeouts": 0,
            "decode_errors": 0,
            "clock_resets": 0,
        }
        self.state_history = []
        self.last_state = self.last_log_wall = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.cloud_sub = self.create_subscription(
            PointCloud2, args.topic, self.on_cloud, qos_profile_sensor_data
        )
        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(String, "/navigation/forward_clearance", 10)
        self.points_pub = self.create_publisher(PointCloud2, "/navigation/forward_points", 5)
        self.marker_pub = self.create_publisher(MarkerArray, "/navigation/clearance_markers", 5)
        # This watchdog continues when simulation time is paused.
        self.timer = self.create_timer(
            0.2, self.publish_status, clock=RosClock(clock_type=ClockType.STEADY_TIME)
        )

    def invalidate(self, reason):
        self.result = None
        self.reason = reason

    def on_clock(self, msg):
        current = stamp_ns(msg.clock)
        if self.clock_ns is not None and current < self.clock_ns:
            self.counts["clock_resets"] += 1
            self.pending = None
            self.last_seen_stamp = None
            if self.args.floor_z is None:
                self.floor_ready = False
                self.floor_samples = []
                self.floor_calibration = {"source": "Recalibrating after clock reset"}
            self.invalidate("Simulation clock reset; waiting for new cloud")
        if self.clock_ns is None or current != self.clock_ns:
            self.clock_progress_wall = time.monotonic()
        self.clock_ns = current

    def on_cloud(self, msg):
        self.counts["clouds_received"] += 1
        stamp = stamp_ns(msg.header.stamp)
        self.invalidate("Waiting for capture-time transforms")
        self.pending = None
        if msg.header.frame_id != self.args.sensor_frame:
            self.reason = "Unexpected sensor frame: " + msg.header.frame_id
        elif stamp <= 0 or (self.last_seen_stamp is not None and stamp <= self.last_seen_stamp):
            self.reason = "Zero, repeated or backwards cloud timestamp"
        else:
            self.pending = (msg, time.monotonic())
        self.last_seen_stamp = stamp

    def process_pending(self):
        if self.pending is None:
            return
        msg, arrived = self.pending
        now = time.monotonic()
        problem = health_issue(
            now_wall=now,
            arrival_wall=arrived,
            clock_progress_wall=self.clock_progress_wall,
            clock_ns=self.clock_ns,
            cloud_ns=stamp_ns(msg.header.stamp),
        )
        if problem:
            self.reason = problem
            if now - arrived > 0.3:
                self.pending = None
            return
        try:
            capture = Time.from_msg(msg.header.stamp)
            body = self.buffer.lookup_transform("chassis", msg.header.frame_id, capture)
            world = self.buffer.lookup_transform("world", msg.header.frame_id, capture)
        except Exception:
            if now - arrived > 0.3:
                self.counts["tf_timeouts"] += 1
                self.pending = None
                self.reason = "Capture-time TF missing for more than 0.3 s"
            return
        try:
            body_rotation, body_translation = transform_parts(body)
            world_rotation, world_translation = transform_parts(world)
            sensor_xyz = xyz_array(msg)
            if not self.floor_ready:
                finite = sensor_xyz[np.isfinite(sensor_xyz).all(axis=1)]
                try:
                    floor = estimate_flat_floor(finite @ world_rotation.T + world_translation)
                except ValueError as exc:
                    self.floor_samples = []
                    self.reason = "Floor calibration: " + str(exc)
                    self.pending = None
                    return
                self.floor_samples.append(floor["height_world_m"])
                self.floor_samples = self.floor_samples[-5:]
                self.floor_calibration = {
                    "source": "Stationary horizontal-plane fit",
                    **floor,
                    "scans": len(self.floor_samples),
                }
                if len(self.floor_samples) < 5 or np.ptp(self.floor_samples) > 0.02:
                    self.reason = "Calibrating a stable floor from five scans"
                    self.pending = None
                    return
                self.limits = replace(
                    self.limits, floor_z_world_m=float(np.median(self.floor_samples))
                )
                self.floor_ready = True
                print(
                    f"[Clearance] Calibrated floor world Z = {self.limits.floor_z_world_m:.3f} m",
                    file=sys.stderr if getattr(self.args, "stream_session", None) else sys.stdout,
                    flush=True,
                )
            metrics, points = forward_surfaces(
                sensor_xyz,
                body_rotation,
                body_translation,
                world_rotation,
                world_translation,
                self.limits,
            )
        except (ValueError, TypeError, BufferError) as exc:
            self.counts["decode_errors"] += 1
            self.reason = "Invalid cloud or transform: " + str(exc)
            self.pending = None
            return
        self.counts["clouds_processed"] += 1
        self.result = (metrics, points, msg.header.stamp, arrived)
        self.reason = metrics["reason"]
        self.pending = None
        self.points_pub.publish(
            create_cloud_xyz32(
                Header(frame_id="chassis", stamp=msg.header.stamp), points.astype(np.float32)
            )
        )

    def snapshot(self):
        now = time.monotonic()
        metrics = {"state": "UNKNOWN", "reason": self.reason, "forward_distance_m": None}
        stamp = None
        if self.result is not None:
            measured, _, stamp, arrived = self.result
            problem = health_issue(
                now_wall=now,
                arrival_wall=arrived,
                clock_progress_wall=self.clock_progress_wall,
                clock_ns=self.clock_ns,
                cloud_ns=stamp_ns(stamp),
            )
            metrics = dict(measured)
            if problem:
                metrics.update(
                    state="UNKNOWN",
                    reason=problem,
                    forward_distance_m=None,
                    nearest_body_xyz_m=None,
                )
        return {
            **metrics,
            "stamp_ns": stamp_ns(stamp) if stamp else None,
            "floor_ready": self.floor_ready,
            "cloud_age_wall_s": now - self.result[3] if self.result is not None else None,
            "clock_age_wall_s": (
                now - self.clock_progress_wall if self.clock_progress_wall is not None else None
            ),
            "body_frame": "chassis",
            "world_frame": "world",
            "limits": asdict(self.limits),
            "counts": dict(self.counts),
            "floor_calibration": self.floor_calibration,
            "meaning": "Observed forward surfaces only; recommendation, no flight command",
        }

    def publish_status(self):
        report = self.snapshot()
        state = report["state"]
        now = time.monotonic()
        if state != self.last_state:
            self.state_history.append(
                {
                    "state": state,
                    "reason": report["reason"],
                    "forward_distance_m": report["forward_distance_m"],
                }
            )
            self.state_history = self.state_history[-100:]
        report["state_history"] = list(self.state_history)
        self.status_pub.publish(String(data=json.dumps(report, allow_nan=False)))
        self.marker_pub.publish(self.markers(report))
        if state == "UNKNOWN":
            self.points_pub.publish(
                create_cloud_xyz32(
                    Header(frame_id="chassis", stamp=self.get_clock().now().to_msg()), []
                )
            )
        if (
            state != self.last_state
            or self.last_log_wall is None
            or now - self.last_log_wall >= 1.0
        ):
            distance = report["forward_distance_m"]
            label = f"{distance:.2f} m" if distance is not None else "no measurement"
            print(
                f"[Clearance] {state}: {label}; {report['reason']}",
                file=sys.stderr if getattr(self.args, "stream_session", None) else sys.stdout,
                flush=True,
            )
            self.last_log_wall = now
        self.last_state = state
        if getattr(self.args, "stream_session", None):
            # Dedicated local pipe to the owning controller. Same-host monotonic
            # time lets the receiver reject buffered old records. No saved file
            # is consumed by flight code, and the worker never publishes PX4 targets.
            envelope = dict(
                session=self.args.stream_session,
                generated_monotonic_s=time.monotonic(),
                report=report,
            )
            print(json.dumps(envelope, allow_nan=False), flush=True)
        if self.args.output:
            self.args.output.parent.mkdir(parents=True, exist_ok=True)
            self.args.output.write_text(
                json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
            )

    def markers(self, report):
        state = report["state"]
        rgb = {"CLEAR": (0.1, 0.8, 0.2), "HOLD": (1.0, 0.1, 0.1), "UNKNOWN": (0.6, 0.6, 0.6)}[state]
        markers = []
        for ident, kind in ((0, Marker.CUBE), (1, Marker.TEXT_VIEW_FACING)):
            marker = Marker()
            marker.header = Header(frame_id="chassis", stamp=self.get_clock().now().to_msg())
            marker.ns, marker.id, marker.type = "forward_clearance", ident, kind
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.frame_locked = True
            marker.lifetime = Duration(sec=1)
            marker.color.r, marker.color.g, marker.color.b = rgb
            marker.color.a = 0.15 if ident == 0 else 1.0
            if ident == 0:
                marker.pose.position.x = (self.limits.hold_m + self.limits.forward_min_m) / 2
                marker.pose.position.z = (self.limits.body_z_min_m + self.limits.body_z_max_m) / 2
                marker.scale.x = self.limits.hold_m - self.limits.forward_min_m
                marker.scale.y = 2 * self.limits.half_width_m
                marker.scale.z = self.limits.body_z_max_m - self.limits.body_z_min_m
            else:
                marker.pose.position.z = 0.9
                marker.scale.z = 0.18
                distance = report["forward_distance_m"]
                marker.text = state + (
                    f" {distance:.2f} m" if distance is not None else ": " + report["reason"]
                )
            markers.append(marker)
        return MarkerArray(markers=markers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/lidar/points")
    parser.add_argument("--sensor-frame", default="point_cloud")
    parser.add_argument(
        "--floor-z",
        type=float,
        help="Measured world floor height; omitted calibrates five stationary scans",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="Wall seconds; zero runs until Ctrl+C"
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--stream-session", help=argparse.SUPPRESS)
    args, ros_args = parser.parse_known_args()
    if (
        not math.isfinite(args.duration)
        or args.duration < 0
        or (args.floor_z is not None and not math.isfinite(args.floor_z))
    ):
        parser.error("Require finite floor height and nonnegative finite duration")
    rclpy.init(args=ros_args)
    if rclpy.get_rmw_implementation_identifier() != "rmw_zenoh_cpp":
        rclpy.shutdown()
        parser.error("Use ROS 2 Jazzy Zenoh or tools/forward_clearance.cmd")
    node = ForwardClearance(args)
    print(
        "Forward clearance: +X forward, +/-0.5 m sideways; 2 m HOLD threshold. No flight outputs.",
        file=sys.stderr if args.stream_session else sys.stdout,
        flush=True,
    )
    end = time.monotonic() + args.duration if args.duration else float("inf")
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            node.process_pending()
        node.publish_status()
    except KeyboardInterrupt:
        node.invalidate("Processor stopped")
        node.publish_status()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
