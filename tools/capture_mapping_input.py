"""Bounded read-only ROS capture of raw clouds and their capture-time transforms.

Creates an NPZ for offline replay; never publishes flight or simulator commands.
"""

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from lidar_tools.cloud import xyz_array
from tools.forward_clearance import transform_parts, stamp_ns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--output", type=Path, default=Path("logs/mapping_input.npz"))
    args = parser.parse_args()
    if not 1 <= args.duration <= 60 or not 1 <= args.samples <= 40:
        parser.error("Capture must be bounded to 1..60 seconds and 1..40 samples")
    rclpy.init()
    node = Node("mapping_input_capture")
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    pending = []
    arrays, records = {}, []
    received = 0

    def on_cloud(msg):
        nonlocal received
        received += 1
        if not pending:
            pending.append((msg, time.monotonic()))

    subscription = node.create_subscription(
        PointCloud2, "/lidar/points", on_cloud, qos_profile_sensor_data
    )
    deadline, last_capture = time.monotonic() + args.duration, 0.0
    errors = []
    try:
        while time.monotonic() < deadline and len(records) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.02)
            if not pending:
                continue
            msg, arrived = pending[0]
            if time.monotonic() - arrived > 0.5:
                pending.clear()
                continue
            try:
                capture = Time.from_msg(msg.header.stamp)
                world = buffer.lookup_transform("world", msg.header.frame_id, capture)
                body = buffer.lookup_transform("chassis", msg.header.frame_id, capture)
            except Exception:
                continue
            pending.clear()
            if time.monotonic() - last_capture < 0.5:
                continue
            try:
                xyz = xyz_array(msg)
                wr, wt = transform_parts(world)
                br, bt = transform_parts(body)
                index = len(records)
                for name, value in dict(xyz=xyz, wr=wr, wt=wt, br=br, bt=bt).items():
                    arrays[f"{name}_{index}"] = value
                records.append(
                    dict(
                        index=index,
                        stamp_ns=stamp_ns(msg.header.stamp),
                        frame=msg.header.frame_id,
                        points=len(xyz),
                        finite=int(np.isfinite(xyz).all(axis=1).sum()),
                    )
                )
                last_capture = time.monotonic()
            except (ValueError, TypeError) as exc:
                errors.append(str(exc))
    finally:
        node.destroy_subscription(subscription)
        del listener
        node.destroy_node()
        rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **arrays)
    report = dict(
        received=received,
        samples=records,
        errors=errors,
        output=str(args.output),
        note="Raw input and capture-time TF; read-only, no flight commands",
    )
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0 if records else 1


if __name__ == "__main__":
    raise SystemExit(main())
