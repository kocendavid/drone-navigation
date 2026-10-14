"""Persistent mapped-goal controller. Never arms or selects PX4 modes."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from queue import Empty, Queue
import threading
import time
import numpy as np
import rclpy
from px4_msgs.msg import SensorCombined, VehicleStatus
from rclpy.qos import qos_profile_sensor_data
from ros2_hover import HoverPublisher, hover_messages, PX4_RECEIPT_TIMESTAMP
from ros2_step import SimulationPacer, topic_for
from navigation.feed import MapFeed
from navigation.dashboard import Dashboard
from navigation.motion import FrameAlignment, RouteFollower, clearance_ahead
from navigation.planning import plan, segment_safe
from navigation.exploration import ExplorationSession, choose_frontier
from navigation.settings import load_settings


class GoalNavigator(HoverPublisher):
    def __init__(self, feed, dashboard, commands, settings=None):
        self.feed = feed
        self.dashboard = dashboard
        self.commands = commands
        self.settings = settings or load_settings()
        self.exploration = ExplorationSession(self.settings.exploration_max_goals)
        self.position = self.status = self.status_wall = self.alignment = None
        self.follower = RouteFollower(self.settings)
        self.pacer = SimulationPacer()
        self.pacer_mark = 0.0
        self.last_callback = None
        self.last_report = 0.0
        self.last_retry = 0.0
        self.last_map_scans = None
        self.target_goal = None
        self.plan_thread = None
        self.plan_results = Queue()
        self.plan_generation = 0
        self.settled_since = None
        self.hold_yaw = None
        self.reference = None
        self.world_pose = None
        super().__init__(
            1.0, node_name="isaac_goal_navigator", status_filename="ros2_navigation_status.json"
        )
        self.create_subscription(
            VehicleStatus,
            topic_for(VehicleStatus, "/fmu/out/vehicle_status"),
            self.on_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SensorCombined,
            topic_for(SensorCombined, "/fmu/out/sensor_combined"),
            lambda m: self.pacer.observe(m, time.monotonic()),
            qos_profile_sensor_data,
        )

    def on_status(self, status):
        self.status = status
        self.status_wall = time.monotonic()

    def on_position(self, p):
        self.position = p
        super().on_position(p)

    def flight_active(self, now):
        return (
            self.status is not None
            and self.status_wall is not None
            and 0 <= now - self.status_wall <= 2.0
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and not self.status.failsafe
        )

    def stable(self):
        p = self.position
        return (
            p is not None
            and p.v_xy_valid
            and p.v_z_valid
            and all(math.isfinite(v) for v in (p.vx, p.vy, p.vz))
            and abs(p.z + 1.0) <= 0.15
            and math.hypot(p.vx, p.vy) <= 0.10
            and abs(p.vz) <= 0.10
        )

    def stop(self, reason, phase="PAUSED", clear_goal=False):
        if self.world_pose is not None:
            self.follower.hold(self.world_pose, reason, phase)
        self.plan_generation += 1
        self.settled_since = None
        if clear_goal:
            self.target_goal = None
            self.exploration.cancel(reason)
        elif phase == "PAUSED":
            self.exploration.pause(reason)
        self.say(reason)

    def launch_plan(self, snap):
        if self.plan_thread is not None and self.plan_thread.is_alive():
            return
        self.last_retry = time.monotonic()
        self.plan_generation += 1
        token = self.plan_generation
        start = self.world_pose.copy()
        goal = self.target_goal.copy()
        self.follower.hold(start, "Planning route", "PLANNING")

        def work():
            try:
                self.plan_results.put((token, plan(snap, start, goal), None, None))
            except ValueError as exc:
                self.plan_results.put((token, None, str(exc), None))

        self.plan_thread = threading.Thread(target=work, daemon=True)
        self.plan_thread.start()

    def launch_frontier(self, snap):
        if self.plan_thread is not None and self.plan_thread.is_alive():
            return
        self.plan_generation += 1
        token = self.plan_generation
        start = self.world_pose.copy()
        visited = list(self.exploration.visited)
        self.target_goal = None
        self.follower.hold(start, "Selecting frontier viewpoint", "PLANNING")

        def work():
            try:
                choice = choose_frontier(snap, start, visited)
                self.plan_results.put((token, choice.path, None, choice))
            except ValueError as exc:
                self.plan_results.put((token, None, str(exc), "exploration"))

        self.plan_thread = threading.Thread(target=work, daemon=True)
        self.plan_thread.start()

    def exploration_tick(self, now, snap, map_ready):
        session = self.exploration
        if not session.enabled or session.phase == "PAUSED":
            return
        if not map_ready:
            self.stop("Exploration needs a fresh map; Resume when ready")
            return
        if session.phase == "SCANNING":
            session.scanned(snap)
            if not session.enabled:
                self.target_goal = None
                self.stop(session.reason, "HOLD")
                return
        if (
            session.phase == "SELECTING"
            and self.stable()
            and self.state.motion_ready(now)
            and self.pacer.fresh(now)
            and self.plan_results.empty()
            and (self.plan_thread is None or not self.plan_thread.is_alive())
        ):
            self.launch_frontier(snap)

    def commands_tick(self, now, snap, map_ready):
        while True:
            try:
                item = self.commands.get_nowait()
            except Empty:
                break
            action = item["action"]
            if action == "hold":
                self.stop("Operator hold", "HOLD", True)
                continue
            if not (
                self.ready_samples >= 50
                and self.flight_active(now)
                and map_ready
                and self.alignment is not None
                and self.state.motion_ready(now)
                and self.pacer.fresh(now)
                and self.stable()
            ):
                self.dashboard.note = (
                    "Goal refused: need fresh map, stable armed Offboard hover, and READY"
                )
                continue
            if action == "goal":
                self.exploration.cancel("Manual destination selected")
                self.target_goal = np.array(item["point"], dtype=float)
                self.stop("Goal selected", "PLANNING")
                self.launch_plan(snap)
            elif action == "explore":
                self.exploration.start(snap)
                self.target_goal = None
                self.stop("Exploration started", "PLANNING")
                self.launch_frontier(snap)
            elif action == "resume":
                if self.exploration.enabled and self.exploration.phase == "PAUSED":
                    self.exploration.phase = (
                        "TRAVELLING" if self.target_goal is not None else "SELECTING"
                    )
                if self.target_goal is not None:
                    self.stop("Resuming goal", "PLANNING")
                    self.launch_plan(snap)

    def publish_target(self):
        now = time.monotonic()
        gap = now - self.last_callback if self.last_callback is not None else 0.0
        self.last_callback = now
        dt = self.pacer.total_s - self.pacer_mark
        self.pacer_mark = self.pacer.total_s
        if not self.state.fresh(now):
            self.ready_samples = 0
            self.exploration.pause("Position unavailable; targets paused")
            if self.follower.phase in ("NAVIGATING", "SETTLING", "PLANNING", "BLOCKED"):
                self.stop("Position unavailable; targets paused")
            self.say(self.state.fault or "Waiting for fresh valid PX4 position; targets paused")
            self.update_dashboard(now, None, False)
            return
        if any(
            self.count_publishers(topic) > 1
            for topic in ("/fmu/in/offboard_control_mode", "/fmu/in/trajectory_setpoint")
        ):
            self.ready_samples = 0
            self.stop("Another flight-target publisher is present")
            self.update_dashboard(now, None, False)
            return
        p = self.position
        snap, failure = self.feed.get()
        map_ready = snap is not None and not failure and snap.fresh(now)
        if self.hold_yaw is None:
            self.hold_yaw = p.heading
        if self.alignment is None and map_ready and self.stable():
            self.alignment = FrameAlignment(snap.pose[:2], snap.pose[2], [p.x, p.y], p.heading)
            self.world_pose = self.alignment.world([p.x, p.y])
            self.follower.hold(self.world_pose)
            self.say("World/PX4 alignment captured at hover. Choose a destination on the map.")
        if self.alignment is not None:
            self.world_pose = self.alignment.world([p.x, p.y])
            if map_ready and (
                np.linalg.norm(self.world_pose - np.array(snap.pose[:2])) > 0.2
                or abs(
                    math.remainder(
                        snap.pose[2] - (math.pi / 2 - p.heading + self.alignment.heading_bias),
                        2 * math.pi,
                    )
                )
                > math.radians(15)
            ):
                map_ready = False
                failure = "World/PX4 pose disagreement; holding"
        if self.reference is None:
            self.reference = np.array(self.state.anchor[:2])
        active = self.follower.phase in ("NAVIGATING", "SETTLING", "PLANNING", "BLOCKED")
        if self.alignment is not None and not self.flight_active(now):
            self.hold_yaw = p.heading
            if self.exploration.enabled:
                self.exploration.cancel("PX4 left armed Offboard")
            if active:
                self.stop("PX4 left armed Offboard", "HOLD", True)
            else:
                self.follower.hold(self.world_pose, "Native PX4 mode; ready for Offboard", "HOLD")
        elif active and (not map_ready or gap > 0.5 or gap < 0):
            self.stop(
                failure
                or (
                    "Map data unavailable; hold until Resume"
                    if not map_ready
                    else "Control callback delayed"
                )
            )
        elif active and (
            abs(p.z + 1.0) > 0.25
            or not p.v_xy_valid
            or not p.v_z_valid
            or not all(math.isfinite(v) for v in (p.vx, p.vy, p.vz))
        ):
            self.stop("Height tracking or velocity validity failed")
        self.commands_tick(now, snap, map_ready)
        if self.flight_active(now):
            self.exploration_tick(now, snap, map_ready)
        if map_ready and self.flight_active(now):
            if (
                self.target_goal is not None
                and self.follower.phase == "BLOCKED"
                and self.stable()
                and now - self.last_retry >= 2.0
            ):
                self.launch_plan(snap)
            elif (
                self.target_goal is not None
                and self.follower.phase == "PLANNING"
                and self.stable()
                and self.plan_results.empty()
                and (self.plan_thread is None or not self.plan_thread.is_alive())
            ):
                self.launch_plan(snap)
            try:
                while True:
                    token, path, error, choice = self.plan_results.get_nowait()
                    if token != self.plan_generation:
                        continue
                    if error and choice == "exploration":
                        self.exploration.enabled = False
                        self.exploration.phase = "FINISHED"
                        self.exploration.reason = error
                        self.target_goal = None
                        self.stop(error, "HOLD")
                    elif error:
                        self.follower.hold(self.world_pose, error, "BLOCKED")
                        if self.exploration.enabled:
                            self.stop(error)
                    elif all(
                        segment_safe(snap.body_free, snap.model.cell(a), snap.model.cell(b))
                        for a, b in zip(path, path[1:])
                    ):
                        if choice is not None:
                            self.target_goal = np.array(choice.goal)
                            self.exploration.phase = "TRAVELLING"
                            self.exploration.expected_gain_m2 = choice.expected_new_area_m2
                            self.exploration.frontier_count = choice.frontier_cells
                            self.exploration.reason = "Travelling to selected frontier viewpoint"
                        self.follower.begin(path)
                        self.say(
                            f"Navigating to ({self.target_goal[0]:.1f}, {self.target_goal[1]:.1f}) m"
                        )
                    else:
                        if choice is not None:
                            self.target_goal = np.array(choice.goal)
                        self.follower.hold(
                            self.world_pose, "Map changed during planning", "BLOCKED"
                        )
            except Empty:
                pass
            if self.follower.phase == "NAVIGATING" and snap.scans != self.last_map_scans:
                route = self.follower.remaining_path(self.world_pose)
                if not all(
                    segment_safe(snap.body_free, snap.model.cell(a), snap.model.cell(b))
                    for a, b in zip(route, route[1:])
                ):
                    self.stop("Route changed; waiting to replan", "BLOCKED")
                self.last_map_scans = snap.scans
            if self.state.motion_ready(now) and self.pacer.fresh(now):
                self.follower.tick(dt, self.world_pose, snap.hits)
            if self.follower.phase == "SETTLING":
                settled = (
                    np.linalg.norm(self.world_pose - self.target_goal) <= 0.15 and self.stable()
                )
                self.settled_since = (
                    (self.settled_since if self.settled_since is not None else self.pacer.total_s)
                    if settled
                    else None
                )
                if (
                    self.settled_since is not None
                    and self.pacer.total_s - self.settled_since >= 1.0
                ):
                    self.follower.phase = "REACHED"
                    self.follower.reason = "Destination reached. Choose another goal."
                    if self.exploration.enabled:
                        self.exploration.arrived(snap, self.target_goal)
                        self.follower.reason = (
                            "Exploration viewpoint reached; scanning before choosing again"
                        )
                    self.say(self.follower.reason)
        if self.alignment is not None and self.follower.target is not None:
            self.reference = self.alignment.ned(self.follower.target)
        mode, target = hover_messages(PX4_RECEIPT_TIMESTAMP, *self.reference, 1.0, self.hold_yaw)
        self.mode_pub.publish(mode)
        self.target_pub.publish(target)
        if self.mode_pub.get_subscription_count() and self.target_pub.get_subscription_count():
            self.ready_samples += 1
            if self.ready_samples == 50:
                self.say(
                    "READY: select Offboard in PX4, then Navigate from the map. No repeated move commands."
                )
        else:
            self.ready_samples = 0
            self.stop("Waiting for PX4 target readers")
        self.update_dashboard(now, snap, map_ready)

    def update_dashboard(self, now, snap, map_ready):
        if now - self.last_report < 0.5:
            return
        self.last_report = now
        p = self.position
        direction = None
        if self.follower.path and self.world_pose is not None:
            direction = self.follower.path[self.follower.index] - self.world_pose
        ahead = (
            clearance_ahead(self.world_pose, direction, snap.hits)
            if snap is not None and direction is not None
            else None
        )
        report = dict(
            recorded_utc=datetime.now(timezone.utc).isoformat(),
            phase=self.follower.phase,
            reason=self.follower.reason,
            ready=bool(
                self.ready_samples >= 50
                and map_ready
                and self.alignment is not None
                and self.flight_active(now)
                and self.stable()
                and self.state.motion_ready(now)
                and self.pacer.fresh(now)
            ),
            path=[q.tolist() for q in self.follower.path],
            goal=self.target_goal.tolist() if self.target_goal is not None else None,
            remaining_m=(
                float(np.linalg.norm(self.world_pose - self.target_goal))
                if self.world_pose is not None and self.target_goal is not None
                else None
            ),
            clearance_m=ahead,
            speed_m_s=self.follower.speed,
            height_m=-float(p.z) if p is not None and math.isfinite(p.z) else None,
            armed_offboard=self.flight_active(now),
            clock_rebases=self.state.clock_rebases,
            reference_ned_xy=self.reference.tolist() if self.reference is not None else None,
            measured_world_xy=self.world_pose.tolist() if self.world_pose is not None else None,
            measured_speed_m_s=(
                math.hypot(p.vx, p.vy)
                if p is not None and p.v_xy_valid and all(math.isfinite(v) for v in (p.vx, p.vy))
                else None
            ),
            delivered_simulation_s=self.pacer.total_s,
            settings=asdict(self.settings),
            exploration=dict(
                enabled=self.exploration.enabled,
                phase=self.exploration.phase,
                reason=self.exploration.reason,
                viewpoints_reached=self.exploration.reached,
                new_observed_area_m2=self.exploration.area_gain_m2,
                expected_new_area_m2=self.exploration.expected_gain_m2,
                frontier_cells=self.exploration.frontier_count,
            ),
        )
        self.dashboard.report = report
        path = Path(__file__).parent / "logs" / "navigation_state.json"
        try:
            temp = path.with_suffix(".tmp")
            temp.write_text(json.dumps(report, allow_nan=False, indent=2) + "\n", encoding="utf-8")
            temp.replace(path)
            with path.with_name("navigation_trace.jsonl").open("a", encoding="utf-8") as trace:
                trace.write(json.dumps(report, allow_nan=False, separators=(",", ":")) + "\n")
        except OSError as exc:
            self.get_logger().warning(f"Could not save navigation snapshot: {exc}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--observe", action="store_true", help="Map and route preview only; NO flight publishers"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="Optional bounded observer duration"
    )
    parser.add_argument("--floor-z", type=float, default=-0.76965)
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--settings", type=Path, help="Launch-time navigation settings JSON")
    args = parser.parse_args()
    if not math.isfinite(args.floor_z) or not math.isfinite(args.duration) or args.duration < 0:
        parser.error("Invalid floor/duration")
    if args.duration and not args.observe:
        parser.error("Bounded duration is available only for observation")
    try:
        settings = load_settings(args.settings)
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    feed = MapFeed(
        args.floor_z, settings.near_range_m, settings.obstacle_margin_m, settings.body_radius_m
    )
    dashboard = None
    node = None
    try:
        commands = None if args.observe else Queue(maxsize=8)
        dashboard = Dashboard(feed, commands, args.port)
        dashboard.report["settings"] = asdict(settings)
        print("Navigator:", dashboard.url, flush=True)
        if args.observe:
            print("MAP PREVIEW ONLY: no PX4 targets, arming, or mode commands.", flush=True)
            end = time.monotonic() + args.duration if args.duration else math.inf
            while time.monotonic() < end:
                time.sleep(0.25)
        else:
            rclpy.init()
            if rclpy.get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
                raise RuntimeError("Use tools/ros2_navigate.cmd")
            node = GoalNavigator(feed, dashboard, commands, settings)
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if dashboard is not None:
            dashboard.close()
        feed.close()


if __name__ == "__main__":
    main()
