"""Publish a hover target; the operator selects Offboard in the PX4 console."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.clock import ClockType
from rclpy.utilities import get_rmw_implementation_identifier
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition

# The installed PX4 XRCE deserializers replace zero with hrt_absolute_time().
# Command receipt time must not reuse a sensor timestamp with an old XRCE offset.
PX4_RECEIPT_TIMESTAMP = 0


def hover_messages(timestamp_us, x, y, height, yaw):
    mode = OffboardControlMode()
    mode.timestamp = timestamp_us
    mode.position = True
    target = TrajectorySetpoint()
    target.timestamp = timestamp_us
    # PX4 local coordinates are NED: negative Z means above the local origin.
    target.position = [float(x), float(y), -float(height)]
    target.velocity = [math.nan] * 3
    target.acceleration = [math.nan] * 3
    target.jerk = [math.nan] * 3
    target.yaw = float(yaw)
    target.yawspeed = math.nan
    return mode, target


class HoverState:
    """Keep a fixed target only while its PX4 coordinate origin remains valid."""

    def __init__(self):
        self.anchor = None
        self.timestamp_us = None
        self.last_progress = None
        self.resets = None
        self.fault = None
        self.clock_rebases = 0
        self.recovery_remaining = 0

    def observe(self, position, now):
        if self.fault:
            return False
        timestamp = int(position.timestamp)
        resets = (
            position.xy_reset_counter,
            position.z_reset_counter,
            position.heading_reset_counter,
        )
        if self.anchor is not None and resets != self.resets:
            self.fault = "PX4 position/height/heading origin reset; leave Offboard and restart this publisher."
            self.last_progress = None
            return False
        if not (
            timestamp > 0
            and position.xy_valid
            and position.z_valid
            and all(
                math.isfinite(v) for v in (position.x, position.y, position.z, position.heading)
            )
        ):
            self.last_progress = None
            if self.recovery_remaining:
                self.recovery_remaining = 3
            return False
        if self.timestamp_us is not None and timestamp < self.timestamp_us:
            # XRCE adjusts outgoing timestamps by a changing clock offset.
            # A backward mapped timestamp alone is not an aircraft-origin reset.
            self.timestamp_us = timestamp
            # Keep the last trusted sample's bounded freshness while confirming
            # the new timestamp domain. Corrections do not refresh that age.
            self.clock_rebases += 1
            self.recovery_remaining = 3
            return False
        if timestamp == self.timestamp_us:
            return False  # Repeated packets cannot refresh the watchdog.
        self.timestamp_us = timestamp
        if self.recovery_remaining:
            self.recovery_remaining -= 1
            if self.recovery_remaining:
                return False
        self.last_progress = now
        if self.anchor is None:
            self.anchor = (position.x, position.y, position.heading)
            self.resets = resets
        return True

    def fresh(self, now):
        return (
            self.fault is None
            and self.last_progress is not None
            and 0 <= now - self.last_progress <= 1.0
        )

    def motion_ready(self, now):
        return self.fresh(now) and self.recovery_remaining == 0


class HoverPublisher(Node):
    def __init__(
        self,
        height,
        *,
        node_name="isaac_hover_target",
        status_filename="ros2_hover_status.json",
        mode_notice="This script never arms or changes mode.",
        publish_period_s=0.1,
        status_sink=None,
    ):
        super().__init__(node_name)
        self.height = height
        self.publish_period_s = publish_period_s
        self.status_sink = status_sink
        self.state = HoverState()
        self.ready_samples = 0
        self.notice = None
        self.output_path = Path(__file__).parent / "logs" / status_filename
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
        )
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.target_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        version = getattr(VehicleLocalPosition, "MESSAGE_VERSION", 0)
        position_topic = "/fmu/out/vehicle_local_position" + (f"_v{version}" if version else "")
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, position_topic, self.on_position, qos
        )
        self.timer = self.create_timer(
            publish_period_s, self.publish_target, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.say("Waiting for valid PX4 position. " + mode_notice)

    def say(self, message):
        if message != self.notice:
            self.notice = message
            report = dict(
                recorded_utc=datetime.now(timezone.utc).isoformat(),
                message=message,
                anchor_ned_xy_yaw=self.state.anchor,
                reset_counters=self.state.resets,
                received_timestamp_us=self.state.timestamp_us,
                clock_rebases=self.state.clock_rebases,
                recovery_remaining=self.state.recovery_remaining,
                command_timestamp_policy="PX4 receipt time (timestamp=0)",
                fault=self.state.fault,
                note="Diagnostic snapshot at a message change, not a control heartbeat.",
            )
            if getattr(self, "status_sink", None) is not None:
                self.status_sink(message, report, self.output_path)
                return
            self.get_logger().info(message)
            try:
                self.output_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.output_path.with_suffix(".tmp")
                temporary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
                temporary.replace(self.output_path)
            except OSError as exc:
                self.get_logger().warning(f"Could not save hover diagnostic: {exc}")

    def on_position(self, position):
        first = self.state.anchor is None
        was_recovering = bool(self.state.recovery_remaining)
        progressed = self.state.observe(position, time.monotonic())
        if self.state.fault:
            self.say(self.state.fault)
        elif progressed and first:
            self.say(
                f"Holding current X/Y and yaw, targeting {self.height:.2f} m "
                f"above PX4's local origin. Streaming targets at {1 / self.publish_period_s:g} Hz."
            )
        elif progressed and was_recovering:
            self.say("Timestamp correction confirmed; fixed hover stream remains active.")

    def publish_target(self):
        if not self.state.fresh(time.monotonic()):
            self.ready_samples = 0
            self.say(
                self.state.fault
                or (
                    "Received timestamp moved backward; waiting for three advancing valid samples."
                    if self.state.recovery_remaining
                    else "Waiting for fresh valid PX4 position; targets are paused."
                )
            )
            return
        x, y, yaw = self.state.anchor
        # PX4 stamps receipt with its own clock. The position watchdog above is
        # independent; receipt stamping must never hide stale/invalid state.
        timestamp = PX4_RECEIPT_TIMESTAMP
        mode, target = hover_messages(timestamp, x, y, self.height, yaw)
        self.mode_pub.publish(mode)
        self.target_pub.publish(target)
        if (
            self.mode_pub.get_subscription_count() == 0
            or self.target_pub.get_subscription_count() == 0
        ):
            self.ready_samples = 0
            self.say("Streaming; waiting for PX4 to subscribe to both input topics.")
            return
        self.ready_samples += 1
        if self.ready_samples == 50:
            self.say("READY: in PX4 - Isaac SITL, enter: commander mode offboard")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--height",
        type=float,
        default=1.0,
        help="Height above PX4 local origin in metres (default: 1.0)",
    )
    parser.add_argument(
        "--check", action="store_true", help="Check message construction without connecting to ROS"
    )
    args = parser.parse_args()
    if not math.isfinite(args.height) or args.height <= 0:
        parser.error("height must be a finite positive number")
    if args.check:
        mode, target = hover_messages(PX4_RECEIPT_TIMESTAMP, 2.0, -3.0, 1.0, 0.25)
        assert mode.position and mode.timestamp == target.timestamp == 0
        assert not any(
            (
                mode.velocity,
                mode.acceleration,
                mode.attitude,
                mode.body_rate,
                mode.thrust_and_torque,
                mode.direct_actuator,
            )
        )
        assert list(target.position) == [2.0, -3.0, -1.0]
        assert target.yaw == 0.25 and math.isnan(target.yawspeed)
        assert all(
            math.isnan(v)
            for values in (target.velocity, target.acceleration, target.jerk)
            for v in values
        )
        print(
            "PASS: installed PX4 messages, receipt-time stamping, NED target and inactive fields. No ROS connection."
        )
        return
    rclpy.init()
    if get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
        rclpy.shutdown()
        parser.error("Use tools/ros2_hover.cmd in the Jazzy/Pixi terminal to select Fast DDS")
    node = HoverPublisher(args.height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
