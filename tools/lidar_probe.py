"""Bounded subscriptions only: clouds, clock, TF and optional PX4 position."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lidar_tools.cloud import cloud_summary
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class LidarProbe(Node):
    def __init__(self, args):
        super().__init__(
            "isaac_lidar_probe", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.args = args
        self.started = time.monotonic()
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.arrivals = []
        self.stamps = []
        self.frames = set()
        self.usable_clouds = 0
        self.errors = []
        self.latest = None
        self.clock_count = 0
        self.clock_first = None
        self.clock_last = None
        self.clock_wall = None
        self.clock_backwards = 0
        self.position_count = 0
        self.position = None
        self.position_wall = None
        self.tf_ok = 0
        self.tf_failed = 0
        self.tf_error = None
        self.tf_pending = []
        self.sub = self.create_subscription(
            PointCloud2, args.topic, self.on_cloud, qos_profile_sensor_data
        )
        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.on_clock, qos_profile_sensor_data
        )
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        try:
            from px4_msgs.msg import VehicleLocalPosition
        except ImportError:
            self.position_sub = None
        else:
            self.position_sub = self.create_subscription(
                VehicleLocalPosition,
                "/fmu/out/vehicle_local_position_v1",
                self.on_position,
                qos_profile_sensor_data,
            )

    def on_clock(self, msg):
        value = stamp_ns(msg.clock)
        if self.clock_first is None:
            self.clock_first = value
        if self.clock_last is not None and value < self.clock_last:
            self.clock_backwards += 1
        self.clock_count += 1
        self.clock_last = value
        self.clock_wall = time.monotonic()

    def on_position(self, msg):
        self.position_count += 1
        self.position_wall = time.monotonic()
        values = (msg.x, msg.y, msg.z, msg.vz, msg.heading)
        self.position = {
            "xy_valid": bool(msg.xy_valid),
            "z_valid": bool(msg.z_valid),
            "finite": all(math.isfinite(v) for v in values),
            "x_ned_m": float(msg.x) if math.isfinite(msg.x) else None,
            "y_ned_m": float(msg.y) if math.isfinite(msg.y) else None,
            "z_ned_m": float(msg.z) if math.isfinite(msg.z) else None,
            "vz_ned_m_s": float(msg.vz) if math.isfinite(msg.vz) else None,
        }

    def on_cloud(self, msg):
        now = time.monotonic()
        self.arrivals.append(now)
        self.stamps.append(stamp_ns(msg.header.stamp))
        self.frames.add(msg.header.frame_id)
        try:
            summary = cloud_summary(msg, self.args.min_range, self.args.max_range)
        except (ValueError, TypeError, BufferError) as exc:
            if len(self.errors) < 10:
                self.errors.append(str(exc))
            return
        self.latest = {
            "frame_id": msg.header.frame_id,
            "stamp_ns": self.stamps[-1],
            "width": msg.width,
            "height": msg.height,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
            "fields": [
                {"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
                for f in msg.fields
            ],
            **summary,
        }
        if summary["in_range_points"]:
            self.usable_clouds += 1
        if self.args.target_frame:
            try:
                if self.stamps[-1] == 0:
                    raise ValueError(
                        "Zero cloud stamp would request latest TF, not capture-time TF"
                    )
                self.buffer.lookup_transform(
                    self.args.target_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)
                )
            except Exception as exc:
                self.tf_error = str(exc)
                self.tf_pending.append((msg.header.frame_id, self.stamps[-1], now))
            else:
                self.tf_ok += 1
        if len(self.arrivals) == 1:
            print("FIRST_CLOUD " + json.dumps(self.latest, allow_nan=False), flush=True)

    def resolve_pending_tf(self):
        """Allow independently delivered TF to catch up; never substitute latest TF."""
        now = time.monotonic()
        remaining = []
        for frame, stamp, arrived in self.tf_pending:
            try:
                if stamp == 0:
                    raise ValueError("Zero cloud stamp cannot use capture-time TF")
                self.buffer.lookup_transform(self.args.target_frame, frame, Time(nanoseconds=stamp))
            except Exception as exc:
                self.tf_error = str(exc)
                if now - arrived >= getattr(self.args, "tf_wait", 0.3):
                    self.tf_failed += 1
                else:
                    remaining.append((frame, stamp, arrived))
            else:
                self.tf_ok += 1
        self.tf_pending = remaining

    def report(self):
        self.resolve_pending_tf()
        now = time.monotonic()
        elapsed = now - self.started
        n = len(self.arrivals)
        gaps = [b - a for a, b in zip(self.arrivals, self.arrivals[1:])]
        ds = [b - a for a, b in zip(self.stamps, self.stamps[1:])]
        age = now - self.arrivals[-1] if n else None
        hz = (n - 1) / (self.arrivals[-1] - self.arrivals[0]) if n > 1 else None
        issues = []
        if n < 3:
            issues.append("Fewer than three clouds received")
        if self.usable_clouds != n or not n:
            issues.append("Some/all clouds contain no usable XYZ returns or failed decoding")
        if len(self.frames) != 1 or "" in self.frames:
            issues.append("frame_id is missing or changes during the window")
        if self.stamps and (min(self.stamps) <= 0 or any(v <= 0 for v in ds)):
            issues.append("Cloud timestamps are zero, repeated or go backwards")
        if (
            age is None
            or age > self.args.stale_after
            or any(g > self.args.stale_after for g in gaps)
        ):
            issues.append("Cloud stream is missing, stale or has excessive wall-time gaps")
        if hz is not None and hz < self.args.min_hz:
            issues.append("Arrival frequency is below configured minimum")
        delivery = not issues
        clock_issues = []
        if self.clock_count < 2 or self.clock_last <= self.clock_first:
            clock_issues.append("No advancing /clock received")
        if self.clock_backwards:
            clock_issues.append("Simulation clock reset during observation")
        if self.clock_wall is None or now - self.clock_wall > self.args.stale_after:
            clock_issues.append("/clock is missing or stale in wall time")
        offset = (
            (self.clock_last - self.stamps[-1]) / 1e9 if self.clock_last is not None and n else None
        )
        if offset is None or abs(offset) > self.args.max_clock_skew:
            clock_issues.append(
                "Cloud timestamp and /clock are not within configured skew tolerance"
            )
        if self.args.target_frame and (not self.tf_ok or self.tf_failed or self.tf_pending):
            issues.append("Capture-time transform unavailable for some/all clouds")
        return {
            "started_utc": self.started_utc,
            "rmw": rclpy.get_rmw_implementation_identifier(),
            "domain_id": os.environ.get("ROS_DOMAIN_ID", "0 (default)"),
            "use_sim_time": self.get_parameter("use_sim_time").value,
            "topic": self.args.topic,
            "topic_types": dict(self.get_topic_names_and_types()).get(self.args.topic, []),
            "publishers": [
                {
                    "node": p.node_name,
                    "namespace": p.node_namespace,
                    "type": p.topic_type,
                    "reliability": p.qos_profile.reliability.name,
                    "durability": p.qos_profile.durability.name,
                }
                for p in self.get_publishers_info_by_topic(self.args.topic)
            ],
            "observed_wall_s": elapsed,
            "cloud_messages": n,
            "usable_clouds": self.usable_clouds,
            "frames": sorted(self.frames),
            "arrival_hz": hz,
            "stamp_hz": (
                (n - 1) * 1e9 / (self.stamps[-1] - self.stamps[0])
                if n > 1 and self.stamps[-1] > self.stamps[0]
                else None
            ),
            "max_arrival_gap_s": max(gaps) if gaps else None,
            "last_arrival_age_s": age,
            "decode_errors": self.errors,
            "latest_cloud": self.latest,
            "clock_messages": self.clock_count,
            "cloud_to_clock_offset_s": offset,
            "clock_issues": clock_issues,
            "target_frame": self.args.target_frame,
            "capture_time_tf_successes": self.tf_ok,
            "capture_time_tf_failures": self.tf_failed,
            "capture_time_tf_pending": len(self.tf_pending),
            "last_tf_error": self.tf_error,
            "tf_frames_yaml": self.buffer.all_frames_as_yaml(),
            "px4_position_messages": self.position_count,
            "latest_px4_position": self.position,
            "px4_position_age_s": (
                now - self.position_wall if self.position_wall is not None else None
            ),
            "delivery_pass": delivery,
            "frame_time_pass": delivery
            and not clock_issues
            and bool(self.args.target_frame)
            and not self.tf_failed
            and not self.tf_pending
            and bool(self.tf_ok),
            "issues": issues,
            "observation_incomplete": self.args.target_frame is None,
            "limits": {
                "min_range_m": self.args.min_range,
                "max_range_m": self.args.max_range,
                "min_arrival_hz": self.args.min_hz,
                "stale_after_wall_s": self.args.stale_after,
                "max_clock_skew_s": self.args.max_clock_skew,
                "tf_warmup_wall_s": (
                    getattr(self.args, "tf_warmup", 1.0) if self.args.target_frame else 0.0
                ),
                "tf_wait_wall_s": getattr(self.args, "tf_wait", 0.3),
            },
            "note": "Subscription evidence only. TF existence does not validate physical mounting, coverage or navigation safety.",
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "middleware",
        choices=("dds", "zenoh"),
        help="Explicit middleware selected by ros_observe.cmd",
    )
    parser.add_argument("--topic", default="/lidar/points")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--min-range", type=float, default=0.2)
    parser.add_argument("--max-range", type=float, default=30.0)
    parser.add_argument("--min-hz", type=float, default=2.0)
    parser.add_argument("--stale-after", type=float, default=1.0)
    parser.add_argument("--max-clock-skew", type=float, default=0.5)
    parser.add_argument(
        "--target-frame", help="Actual ROS body frame; checks TF at every cloud stamp"
    )
    parser.add_argument(
        "--tf-wait",
        type=float,
        default=0.3,
        help="Bounded wall time for late capture-time TF (nonblocking)",
    )
    parser.add_argument(
        "--tf-warmup",
        type=float,
        default=1.0,
        help="Collect TF history before starting the cloud observation window",
    )
    parser.add_argument("--output", type=Path)
    args, ros_args = parser.parse_known_args()
    for key in (
        "duration",
        "max_range",
        "min_hz",
        "stale_after",
        "max_clock_skew",
        "tf_wait",
        "tf_warmup",
    ):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(f"{key} must be finite and positive")
    if not math.isfinite(args.min_range) or not 0 <= args.min_range < args.max_range:
        parser.error("Require 0 <= min_range < max_range")
    rclpy.init(args=ros_args)
    expected_rmw = "rmw_fastrtps_cpp" if args.middleware == "dds" else "rmw_zenoh_cpp"
    if rclpy.get_rmw_implementation_identifier() != expected_rmw:
        rclpy.shutdown()
        parser.error("Middleware differs from request; launch through tools/ros_observe.cmd")
    node = LidarProbe(args)
    if args.target_frame:
        # Dynamic TF cannot resolve clouds captured before this observer's TF history.
        # Establish history first; missing TF still fails during the observation window.
        node.destroy_subscription(node.sub)
        print(
            f"Collecting TF history for {args.tf_warmup:g} wall seconds before observing clouds.",
            flush=True,
        )
        warmup_deadline = time.monotonic() + args.tf_warmup
        while rclpy.ok() and time.monotonic() < warmup_deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        node.sub = node.create_subscription(
            PointCloud2, args.topic, node.on_cloud, qos_profile_sensor_data
        )
        node.started = time.monotonic()
        node.started_utc = datetime.now(timezone.utc).isoformat()
    print(
        f"Observing {args.topic} for {args.duration:g} wall seconds; no flight publishers.",
        flush=True,
    )
    interrupted = False
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            node.resolve_pending_tf()
        # Drain only already received clouds' TF requests, within the same bounded grace.
        node.destroy_subscription(node.sub)
        drain_deadline = time.monotonic() + args.tf_wait
        while node.tf_pending and rclpy.ok() and time.monotonic() < drain_deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            node.resolve_pending_tf()
    except KeyboardInterrupt:
        interrupted = True
    finally:
        result = node.report()
        result["interrupted"] = interrupted
        if interrupted:
            result["delivery_pass"] = result["frame_time_pass"] = False
            result["issues"].append("Observation interrupted")
        text = json.dumps(result, indent=2, allow_nan=False)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text + "\n", encoding="utf-8")
        print(text, flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result["delivery_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
