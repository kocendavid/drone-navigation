"""One operator-triggered forward step. Never arms or selects a flight mode."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from queue import Empty, Queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from px4_msgs.msg import SensorCombined, VehicleLocalPosition, VehicleStatus

from ros2_hover import HoverPublisher, HoverState, hover_messages, PX4_RECEIPT_TIMESTAMP


def step_duration(distance, speed=0.2, acceleration=0.2):
    """Quintic reference peaks: speed=1.875*d/T, acceleration=5.774*d/T^2."""
    if not all(math.isfinite(v) and v > 0 for v in (distance, speed, acceleration)):
        raise ValueError("Distance, speed and acceleration must be finite and positive")
    return max(1.875 * distance / speed, math.sqrt(5.774 * distance / acceleration))


class SimulationPacer:
    """Count delivered IMU integration periods, independent of XRCE offsets.

    gyro_integral_dt is a duration, not a mapped timestamp. Lost messages or
    rejected clock corrections slow the reference; no wall-time catch-up.
    This counter is conservative progress, not an absolute simulation clock.
    """

    def __init__(self):
        self.total_s = 0.0
        self.timestamp = None
        self.last_progress = None
        self.confirming = 0

    def observe(self, sensor, now):
        stamp, period = int(sensor.timestamp), int(sensor.gyro_integral_dt)
        if stamp <= 0 or not 0 < period <= 50000:
            self.last_progress = None
            return
        if self.timestamp is None:
            self.timestamp = stamp
            return  # Establish the baseline; no interval before subscription.
        if stamp == self.timestamp:
            return
        if stamp < self.timestamp:
            self.timestamp = stamp
            self.confirming = 3
            return
        self.timestamp = stamp
        if self.confirming:
            self.confirming -= 1
            return  # Discard uncertain periods without advancing or catching up.
        self.total_s += period / 1e6
        self.last_progress = now

    def fresh(self, now):
        return (
            self.last_progress is not None
            and 0 <= now - self.last_progress <= 1.0
            and self.confirming == 0
        )


class StepPlan:
    """A single bounded reference in the heading captured at publisher startup."""

    def __init__(self, anchor, distance):
        if not math.isfinite(distance) or not 0 < distance <= 1:
            raise ValueError("This experiment supports a forward step of at most 1 m")
        self.anchor = tuple(anchor)
        self.distance = distance
        self.duration = step_duration(distance)
        self.target = self.anchor[:2]
        self.endpoint = (
            anchor[0] + distance * math.cos(anchor[2]),
            anchor[1] + distance * math.sin(anchor[2]),
        )
        self.phase = "HOVER"
        self.used = False
        self.elapsed = 0.0
        self.last_tick = None

    def start(self, now):
        if self.used:
            return False
        self.used = True
        self.phase = "MOVING"
        self.last_tick = now
        return True

    def hold(self, xy=None):
        if xy is not None:
            self.target = tuple(xy)
        self.used = True
        self.phase = "HOLD"
        self.last_tick = None

    def advance(self, now):
        """Advance using delivered simulation integration time, not wall time."""
        if self.phase != "MOVING":
            return self.target
        dt = now - self.last_tick
        self.last_tick = now
        if not 0 <= dt <= 0.5:
            self.hold()  # Never skip ahead after a delayed callback or clock jump.
            return self.target
        self.elapsed = min(self.duration, self.elapsed + dt)
        u = self.elapsed / self.duration
        fraction = 10 * u**3 - 15 * u**4 + 6 * u**5
        self.target = tuple(a + fraction * (b - a) for a, b in zip(self.anchor[:2], self.endpoint))
        if self.elapsed >= self.duration:
            self.phase = "TARGET_COMPLETE"
        return self.target

    def displacement(self, position):
        dx, dy = position.x - self.anchor[0], position.y - self.anchor[1]
        c, s = math.cos(self.anchor[2]), math.sin(self.anchor[2])
        return dx * c + dy * s, -dx * s + dy * c


def topic_for(message, base):
    version = getattr(message, "MESSAGE_VERSION", 0)
    return base + (f"_v{version}" if version else "")


class StepPublisher(HoverPublisher):
    def __init__(self, height, distance, clearance_feed=None):
        self.clearance_feed = clearance_feed
        self.clearance_decision = None
        self.distance = distance
        self.plan = None
        self.position = None
        self.status = None
        self.status_received = None
        self.commands = Queue(maxsize=8)
        self.last_report = 0.0
        self.settled_since = None
        self.pacer = SimulationPacer()
        self.last_callback_wall = None
        self.report_path = Path(__file__).parent / "logs" / "ros2_step_state.json"
        super().__init__(
            height, node_name="isaac_step_target", status_filename="ros2_step_status.json"
        )
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
        )
        self.status_sub = self.create_subscription(
            VehicleStatus, topic_for(VehicleStatus, "/fmu/out/vehicle_status"), self.on_status, qos
        )
        # A short bounded queue keeps sensor delivery current. Dropped periods
        # make movement slower, rather than stretching a stale trajectory.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sensor_sub = self.create_subscription(
            SensorCombined,
            topic_for(SensorCombined, "/fmu/out/sensor_combined"),
            lambda m: self.pacer.observe(m, time.monotonic()),
            sensor_qos,
        )
        threading.Thread(target=self.read_commands, daemon=True).start()

    def read_commands(self):
        while rclpy.ok():
            try:
                command = input().strip().lower()
            except (EOFError, OSError):
                return  # Losing stdin must not stop a live heartbeat.
            if command and not self.commands.full():
                self.commands.put_nowait(command)

    def on_status(self, status):
        self.status, self.status_received = status, time.monotonic()

    def on_position(self, position):
        self.position = position
        super().on_position(position)
        if self.plan is None and self.state.anchor is not None:
            self.plan = StepPlan(self.state.anchor, self.distance)
            x, y = self.plan.endpoint
            self.say(
                f"PREVIEW: forward {self.distance:.2f} m along captured heading; "
                f"endpoint local N/E=({x:.3f}, {y:.3f}), height={self.height:.2f} m. "
                f"Reference duration={self.plan.duration:.2f} s of delivered simulated IMU time. "
                "Waiting for your 'move' command."
            )

    def flight_active(self, now):
        return (
            self.status is not None
            and self.status_received is not None
            and 0 <= now - self.status_received <= 2.0
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and not self.status.failsafe
        )

    def stable_hover(self):
        p = self.position
        return (
            p is not None
            and p.v_xy_valid
            and p.v_z_valid
            and all(math.isfinite(v) for v in (p.vx, p.vy, p.vz))
            and abs(p.z + self.height) <= 0.15
            and abs(p.vz) <= 0.15
            and math.hypot(p.vx, p.vy) <= 0.15
            and math.hypot(p.x - self.plan.anchor[0], p.y - self.plan.anchor[1]) <= 0.15
            and abs(math.remainder(p.heading - self.plan.anchor[2], 2 * math.pi))
            <= math.radians(10)
        )

    def cancel_step(self, reason, *, use_current=False):
        if self.plan is not None and self.plan.phase in ("MOVING", "TARGET_COMPLETE", "REACHED"):
            xy = (self.position.x, self.position.y) if use_current else None
            self.plan.hold(xy)
            self.settled_since = None
            self.say("STEP STOPPED: " + reason + "; no further step in this run.")

    def clearance_check(self, now):
        self.clearance_decision = (
            self.clearance_feed.decision(now) if self.clearance_feed is not None else None
        )
        return self.clearance_decision

    def handle_commands(self, now):
        while True:
            try:
                command = self.commands.get_nowait()
            except Empty:
                return
            if command == "hold":
                self.plan.hold((self.position.x, self.position.y))
                self.settled_since = None
                self.say(
                    "HOLD: holding current measured X/Y at the same requested height and heading."
                )
            elif command == "move":
                gate = self.clearance_check(now)
                if self.plan.used:
                    self.say(
                        "This run permits one step only. Keep holding; switch PX4 to native Hold before restarting, or land if finished."
                    )
                elif gate is not None and not gate.allowed:
                    self.say(
                        "MOVE refused by LiDAR: " + gate.reason + ". This step is still unused."
                    )
                elif (
                    self.ready_samples < 50
                    or not self.flight_active(now)
                    or not self.stable_hover()
                    or not self.state.motion_ready(now)
                    or not self.pacer.fresh(now)
                ):
                    reasons = []
                    if self.ready_samples < 50:
                        reasons.append(f"READY warmup {self.ready_samples}/50 updates")
                    if not self.state.motion_ready(now):
                        reasons.append(
                            f"position/clock not ready (confirming samples: {self.state.recovery_remaining})"
                        )
                    if not self.pacer.fresh(now):
                        reasons.append("fresh progressing simulated IMU periods required")
                    if not self.flight_active(now):
                        reasons.append("fresh armed Offboard status required")
                    if not self.stable_hover():
                        p = self.position
                        reasons.append(
                            f"stable hover required: height={-p.z:.3f} m, "
                            f"horizontal speed={math.hypot(p.vx,p.vy):.3f} m/s, "
                            f"vertical speed={abs(p.vz):.3f} m/s, "
                            f"XY offset={math.hypot(p.x-self.plan.anchor[0],p.y-self.plan.anchor[1]):.3f} m, "
                            f"heading error={math.degrees(math.remainder(p.heading-self.plan.anchor[2],2*math.pi)):.2f} deg, "
                            f"velocity valid={bool(p.v_xy_valid and p.v_z_valid)}"
                        )
                    self.say("MOVE refused: " + "; ".join(reasons))
                elif self.plan.start(self.pacer.total_s):
                    self.say(
                        f"MOVING: reference advances {self.distance:.2f} m over {self.plan.duration:.2f} simulated s "
                        "(longer in real time when Isaac runs slowly); "
                        "type 'hold' to stop this step. PX4 controls the motors."
                    )
            else:
                self.say(
                    "Commands here: move (one forward step), hold (hold current X/Y). Land in the PX4 console."
                )

    def publish_target(self):
        now = time.monotonic()
        callback_gap = (
            (now - self.last_callback_wall) if self.last_callback_wall is not None else 0.0
        )
        self.last_callback_wall = now
        if not self.state.fresh(now) or self.plan is None:
            self.ready_samples = 0
            self.cancel_step("position became unavailable")
            self.say(self.state.fault or "Waiting for fresh valid position; targets paused.")
            return
        if any(
            self.count_publishers(topic) > 1
            for topic in ("/fmu/in/offboard_control_mode", "/fmu/in/trajectory_setpoint")
        ):
            self.ready_samples = 0
            self.cancel_step("another flight-target publisher is present")
            self.say(
                "Targets paused: another flight-target publisher is present. In native Hold or while disarmed, stop the extra copies; keep ONE hover/step publisher."
            )
            return
        self.handle_commands(now)
        gate = self.clearance_check(now)
        if self.plan.phase in ("MOVING", "TARGET_COMPLETE", "REACHED"):
            if callback_gap > 0.5 or callback_gap < 0:
                self.cancel_step("movement callback delayed; holding last reference")
            elif not self.flight_active(now):
                self.cancel_step("armed Offboard state unavailable", use_current=True)
            elif gate is not None and self.plan.phase != "REACHED" and not gate.allowed:
                self.cancel_step("LiDAR: " + gate.reason, use_current=True)
            elif gate is not None and abs(
                math.remainder(self.position.heading - self.plan.anchor[2], 2 * math.pi)
            ) > math.radians(10):
                self.cancel_step(
                    "heading changed beyond the forward-sensing experiment's limit",
                    use_current=True,
                )
            elif (
                abs(self.position.z + self.height) > 0.25
                or math.hypot(
                    self.position.x - self.plan.target[0], self.position.y - self.plan.target[1]
                )
                > 0.35
            ):
                horizontal_error = math.hypot(
                    self.position.x - self.plan.target[0], self.position.y - self.plan.target[1]
                )
                self.cancel_step(
                    f"tracking error exceeded limit: XY={horizontal_error:.3f} m "
                    f"(limit 0.350), height={abs(self.position.z+self.height):.3f} m "
                    "(limit 0.250)",
                    use_current=True,
                )
        if (
            self.state.recovery_remaining or not self.pacer.fresh(now)
        ) and self.plan.phase == "MOVING":
            # Keep the heartbeat and last reference steady while confirming the
            # changed clock mapping. Recovery time must not advance the ramp.
            self.plan.last_tick = self.pacer.total_s
            x, y = self.plan.target
        else:
            x, y = self.plan.advance(self.pacer.total_s)
        mode, target = hover_messages(PX4_RECEIPT_TIMESTAMP, x, y, self.height, self.plan.anchor[2])
        self.mode_pub.publish(mode)
        self.target_pub.publish(target)
        if (
            not self.mode_pub.get_subscription_count()
            or not self.target_pub.get_subscription_count()
        ):
            self.ready_samples = 0
            self.cancel_step("PX4 input reader unavailable", use_current=True)
            self.say("Streaming hover target; waiting for PX4 input readers.")
            return
        self.ready_samples += 1
        if self.ready_samples == 50:
            self.say(
                "READY: in PX4 enter 'commander mode offboard'. Once stable, type 'move' in THIS terminal."
            )
        if self.plan.phase == "TARGET_COMPLETE":
            forward, lateral = self.plan.displacement(self.position)
            settled = (
                abs(forward - self.distance) <= 0.1
                and abs(lateral) <= 0.1
                and abs(self.position.z + self.height) <= 0.15
                and self.position.v_xy_valid
                and self.position.v_z_valid
                and all(
                    math.isfinite(v) for v in (self.position.vx, self.position.vy, self.position.vz)
                )
                and math.hypot(self.position.vx, self.position.vy) <= 0.1
                and abs(self.position.vz) <= 0.1
            )
            self.settled_since = (
                (self.settled_since if self.settled_since is not None else now) if settled else None
            )
            if self.settled_since is not None and now - self.settled_since >= 1:
                self.plan.phase = "REACHED"
                self.say(
                    f"REACHED: measured forward={forward:.2f} m, sideways={lateral:.2f} m. Holding endpoint; land when ready."
                )
        if now - self.last_report >= 1:
            self.last_report = now
            forward, lateral = self.plan.displacement(self.position)
            print(
                f"[step] {self.plan.phase}: measured forward={forward:.2f} m, sideways={lateral:.2f} m, "
                f"height={-self.position.z:.2f} m, "
                f"reference={self.plan.elapsed:.2f}/{self.plan.duration:.2f} simulated s",
                flush=True,
            )
            if gate is not None:
                print(f"[LiDAR gate] {gate.state}: {gate.reason}", flush=True)
            report = dict(
                recorded_utc=datetime.now(timezone.utc).isoformat(),
                phase=self.plan.phase,
                anchor_ned_xy_yaw=self.plan.anchor,
                endpoint_ned_xy=self.plan.endpoint,
                target_ned_xyz=[x, y, -self.height],
                actual_ned_xyz=[self.position.x, self.position.y, self.position.z],
                forward_m=forward,
                lateral_m=lateral,
                reference_elapsed_s=self.plan.elapsed,
                reference_duration_s=self.plan.duration,
                armed_offboard=self.flight_active(now),
                reference_time_policy="delivered simulated IMU integration periods",
                delivered_simulation_s=self.pacer.total_s,
                simulation_pacer_fresh=self.pacer.fresh(now),
                command_timestamp_policy="PX4 receipt time (timestamp=0)",
                clock_rebases=self.state.clock_rebases,
                confirming_samples=self.state.recovery_remaining,
                position_age_wall_s=now - self.state.last_progress,
                lidar_gate=gate.report() if gate is not None else None,
                note="Reference limits are not measured aircraft speed/acceleration guarantees. "
                + (
                    "Observed-surface LiDAR stop gate enabled; no free-volume proof."
                    if gate is not None
                    else "No obstacle connection."
                ),
            )
            try:
                temporary = self.report_path.with_suffix(".tmp")
                temporary.write_text(
                    json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
                )
                temporary.replace(self.report_path)
            except OSError as exc:
                self.get_logger().warning(f"Could not save step report: {exc}")


def preview(height, distance):
    """Observe current pose and print a prospective endpoint; create no publishers."""
    node = Node("isaac_step_preview")
    state = HoverState()
    qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
    )
    node.create_subscription(
        VehicleLocalPosition,
        topic_for(VehicleLocalPosition, "/fmu/out/vehicle_local_position"),
        lambda p: state.observe(p, time.monotonic()),
        qos,
    )
    deadline = time.monotonic() + 10
    try:
        while rclpy.ok() and time.monotonic() < deadline and state.anchor is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        if state.anchor is None:
            raise RuntimeError("No valid position received within 10 seconds")
        plan = StepPlan(state.anchor, distance)
        dx = plan.endpoint[0] - plan.anchor[0]
        dy = plan.endpoint[1] - plan.anchor[1]
        print(
            json.dumps(
                dict(
                    mode="SUBSCRIPTION-ONLY PREVIEW; no flight targets",
                    heading_deg=math.degrees(plan.anchor[2]),
                    distance_m=distance,
                    delta_north_east_m=[dx, dy],
                    delta_isaac_world_east_north_m=[dy, dx],
                    endpoint_local_ned_xyz=[*plan.endpoint, -height],
                    duration_s=plan.duration,
                    note="A flight run captures a new anchor after native takeoff.",
                ),
                indent=2,
            )
        )
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height", type=float, default=1.0)
    parser.add_argument("--distance", type=float, default=0.5)
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Read-only pose and direction preview; no flight publishers",
    )
    parser.add_argument(
        "--require-clearance",
        action="store_true",
        help="Require live LiDAR and stop on close/unknown observations",
    )
    parser.add_argument(
        "--clearance-floor-z",
        type=float,
        help="Independently measured world floor Z for the flat-room experiment",
    )
    parser.add_argument(
        "--clearance-check",
        action="store_true",
        help="Run perception and the decision gate only; NO flight publishers",
    )
    args = parser.parse_args()
    if not math.isfinite(args.height) or not 0 < args.height <= 2:
        parser.error("height must be finite and between 0 and 2 m")
    if not math.isfinite(args.distance) or not 0 < args.distance <= 1:
        parser.error("distance must be finite and between 0 and 1 m")
    if args.clearance_floor_z is not None and not math.isfinite(args.clearance_floor_z):
        parser.error("clearance floor height must be finite")
    if args.preview and (args.require_clearance or args.clearance_check):
        parser.error("Use pose preview or clearance check separately")
    if args.clearance_check:
        from lidar_tools.clearance_feed import ClearanceFeed

        feed = ClearanceFeed(args.clearance_floor_z)
        observations = []
        print(
            "PERCEPTION CHECK ONLY: no PX4 targets or flight commands. Waiting for fresh scans.",
            flush=True,
        )
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                decision = feed.decision(time.monotonic())
                observations.append(
                    dict(recorded_utc=datetime.now(timezone.utc).isoformat(), **decision.report())
                )
                print(f"[LiDAR gate] {decision.state}: {decision.reason}", flush=True)
                time.sleep(0.5)
        finally:
            feed.close()
        path = Path(__file__).parent / "logs" / "clearance_gate_probe.json"
        path.parent.mkdir(exist_ok=True)
        path.write_text(
            json.dumps(dict(flight_publishers=False, observations=observations), indent=2) + "\n",
            encoding="utf-8",
        )
        return
    feed = None
    rclpy.init()
    try:
        if get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
            parser.error("Use tools/ros2_step.cmd in the Jazzy/Pixi terminal")
        if args.preview:
            preview(args.height, args.distance)
        else:
            if args.require_clearance:
                from lidar_tools.clearance_feed import ClearanceFeed

                feed = ClearanceFeed(args.clearance_floor_z)
                print(
                    "LiDAR guard enabled: observed surface stop at 2.20 m; UNKNOWN also stops. "
                    "Owns a perception worker; no extra clearance terminal required.",
                    flush=True,
                )
            node = StepPublisher(args.height, args.distance, feed)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
    finally:
        if feed is not None:
            feed.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
