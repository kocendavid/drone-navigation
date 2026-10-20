"""Mapped navigation with explicit operator flight buttons and explore/land mission."""

import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import sys
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time
import numpy as np
import rclpy
from px4_msgs.msg import (
    SensorCombined,
    VehicleStatus,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    TimesyncStatus,
)
from rclpy.qos import qos_profile_sensor_data
from ros2_hover import HoverPublisher, hover_messages, PX4_RECEIPT_TIMESTAMP
from ros2_step import SimulationPacer, topic_for
from navigation.feed import MapFeed
from navigation.dashboard import Dashboard
from navigation.motion import (
    FrameAlignment,
    RouteFollower,
    clearance_ahead,
    smooth_yaw,
    acceleration_feedforward,
)
from navigation.source_time import SampleTimeline, ReferenceHistory
from navigation.planning import plan, segment_safe, segment_cost
from navigation.centering import routing_geometry
from navigation.exploration import ExplorationSession, choose_frontier, visible_unknown
from navigation.settings import load_settings
from navigation.mission import ExploreMission, assess_completion, save_map
from navigation.ros_map import RosMapPublisher
from navigation.reset import ExperimentReset, EmptyFeed
from navigation.history import RunHistory
from navigation.flight_controls import FlightControls, NativeFlightBackend
from navigation.runtime import RuntimeVersion, prepare_reload, restore_reload, RELOAD_EXIT_CODE
from navigation.diagnostic_wire import DiagnosticEmitter
from navigation.diagnostics import finite
from navigation.background import LatestWorker
from navigation.async_frontiers import AsyncFrontiers
from navigation.prepared_route import CommittedPrefix, PreparedRoute, route_safe, remaining_safe
from navigation.braking import swept_space_safe

RUNTIME = RuntimeVersion(Path(__file__).parent)


class GoalNavigator(HoverPublisher):
    def __init__(self, feed, dashboard, commands, settings=None):
        self.feed = feed
        self.diagnostics = DiagnosticEmitter("navigator")
        self.dashboard = dashboard
        self.commands = commands
        self.settings = settings or load_settings()
        self.exploration = ExplorationSession(self.settings.exploration_max_goals)
        self.exploration.frontier_ledger = AsyncFrontiers()
        self.mission = ExploreMission()
        self.mission_resume_phase = None
        self.last_completion_stamp = None
        self.certified_grid = None
        self.landed = False
        self.landed_wall = None
        self.landed_sim_s = None
        self.landed_stamp = None
        self.status_sim_s = None
        self.auto_recover = False
        self.recovery_stamp = None
        self.recovery_scans = 0
        self.recovery_started_s = None
        self.position = self.status = self.status_wall = self.alignment = None
        self.follower = RouteFollower(self.settings)
        self.pacer = SimulationPacer()
        self.trajectory_clock = feed.trajectory_clock
        self.pacer_mark = 0.0
        self.unclocked_imu_s = 0.0
        self.last_callback = None
        self.last_report = 0.0
        self.last_retry = 0.0
        self.last_map_scans = None
        self.target_goal = None
        self.home_world_xy = None
        self.returning_home = False
        self.plan_thread = None
        self.plan_results = Queue()
        self.plan_generation = 0
        self.settled_since = None
        self.hold_yaw = None
        self.yaw_rate = 0.0
        self.yaw_desired = None
        self.velocity_reference_ned = np.zeros(2)
        self.acceleration_reference_ned = np.zeros(2)
        self.sample_timeline = SampleTimeline()
        self.reference_history = ReferenceHistory()
        self.tracking_alignment = dict(mode="prediction_fallback", reason="awaiting_clock_mapping")
        self.reference = None
        self.world_pose = None
        self.pose_history = deque(maxlen=100)
        self.pose_check = None
        self.alignment_counters = None
        self.alignment_settled_since = None
        self.alignment_capture_stamp = None
        self.alignment_capture_scans = 0
        self.reconsider_thread = None
        self.reconsider_results = Queue()
        self.reconsider_mark = None
        self.reconsider_stamp = None
        self.active_ui_command = None
        self.notices = LatestWorker("navigation_notices", self.publish_notice)
        super().__init__(
            1.0,
            node_name="isaac_goal_navigator",
            status_filename="ros2_navigation_status.json",
            mode_notice="Takeoff and mode changes require explicit Flight setup buttons; exploration never starts automatically.",
            publish_period_s=0.05,
            status_sink=lambda message, report, path: self.notices.submit(
                report["recorded_utc"], (message, report, path)
            ),
        )
        self.create_subscription(
            VehicleStatus,
            topic_for(VehicleStatus, "/fmu/out/vehicle_status"),
            self.on_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TimesyncStatus,
            topic_for(TimesyncStatus, "/fmu/out/timesync_status"),
            lambda message: self.sample_timeline.observe_timesync(message, time.monotonic()),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SensorCombined,
            topic_for(SensorCombined, "/fmu/out/sensor_combined"),
            lambda m: self.pacer.observe(m, time.monotonic()),
            qos_profile_sensor_data,
        )
        self.land_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.create_subscription(
            VehicleCommandAck,
            topic_for(VehicleCommandAck, "/fmu/out/vehicle_command_ack"),
            self.on_command_ack,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleLandDetected,
            topic_for(VehicleLandDetected, "/fmu/out/vehicle_land_detected"),
            self.on_landed,
            qos_profile_sensor_data,
        )
        self.ros_map = RosMapPublisher(self, self.settings.floor_z_m + 1.0)
        self.runtime_status = RUNTIME.report()
        self.last_background_report = 0.0
        self.reporting = LatestWorker("navigation_reporting", self.publish_reports)

    def destroy_node(self):
        self.reporting.close()
        self.notices.close()
        self.exploration.frontier_ledger.close()
        self.diagnostics.close()
        self.sample_timeline.close()
        return super().destroy_node()

    def publish_notice(self, item):
        message, report, path = item
        self.get_logger().info(message)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def on_landed(self, message):
        if message.timestamp == self.landed_stamp:
            return
        self.landed = message.landed
        self.landed_stamp = message.timestamp
        self.landed_wall = time.monotonic()
        self.landed_sim_s = self.pacer.total_s

    def on_command_ack(self, message):
        if (
            message.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
            and message.target_system == 1
            and message.target_component == 191
        ):
            self.mission.acknowledge_land(message.result)

    def on_status(self, status):
        if status.timestamp <= 0 or (
            self.status is not None and status.timestamp == self.status.timestamp
        ):
            return
        self.status = status
        self.status_wall = time.monotonic()
        self.status_sim_s = self.pacer.total_s

    def status_fresh(self, now):
        # PX4 emits status in simulation time. Keep a wall-time cap and require
        # live delivered IMU periods, instead of mistaking slow physics for a mode exit.
        return (
            self.status is not None
            and self.status_wall is not None
            and self.status_sim_s is not None
            and 0 <= now - self.status_wall <= 10.0
            and 0 <= self.pacer.total_s - self.status_sim_s <= 2.0
            # Short clock confirmation freezes motion, not the flight-mode state.
            and self.pacer.last_progress is not None
            and 0 <= now - self.pacer.last_progress <= 1.0
        )

    def on_position(self, p):
        if hasattr(self, "diagnostics"):
            self.diagnostics.emit(
                "dds_position",
                fields=finite(
                    {
                        k: getattr(p, k, None)
                        for k in (
                            "timestamp",
                            "timestamp_sample",
                            "x",
                            "y",
                            "z",
                            "vx",
                            "vy",
                            "vz",
                            "xy_valid",
                            "xy_reset_counter",
                            "heading_reset_counter",
                        )
                    }
                ),
                timestamp_domain="XRCE-adjusted; retained raw, not assumed equal to Isaac time",
            )
        self.position = p
        previous = self.state.last_progress
        super().on_position(p)
        if (
            self.state.last_progress is not None
            and self.state.last_progress != previous
            and self.state.motion_ready(time.monotonic())
        ):
            self.pose_history.append((self.state.last_progress, [p.x, p.y], p.heading))

    def map_pose_matches(self, snap):
        # Map geometry calculation can finish well after the cloud arrives.
        # Compare with the nearest trusted PX4 sample at cloud receipt, not the
        # current moving pose. Both processes use this host's monotonic clock.
        self.pose_check = dict(
            passed=False,
            reason="missing_history",
            cloud_stamp_ns=snap.stamp,
            cloud_receipt_monotonic_s=snap.cloud_wall,
            map_world_xy=list(snap.pose[:2]),
            position_limit_m=0.2,
            heading_limit_deg=15.0,
            history_gap_limit_s=0.25,
        )
        if not self.pose_history:
            return False
        wall, ned_xy, heading = min(
            self.pose_history, key=lambda row: abs(row[0] - snap.cloud_wall)
        )
        self.pose_check.update(
            sample_receipt_monotonic_s=wall,
            sample_offset_s=wall - snap.cloud_wall,
            reason="history_gap",
        )
        if abs(wall - snap.cloud_wall) > 0.25:
            return False
        xy = self.alignment.world(ned_xy)
        yaw = math.pi / 2 - heading + self.alignment.heading_bias
        position_error = float(np.linalg.norm(xy - np.array(snap.pose[:2])))
        heading_error = abs(math.remainder(snap.pose[2] - yaw, 2 * math.pi))
        passed = position_error <= 0.2 and heading_error <= math.radians(15)
        self.pose_check.update(
            passed=passed,
            reason=(
                "matched"
                if passed
                else "position_error" if position_error > 0.2 else "heading_error"
            ),
            px4_world_xy=xy.tolist(),
            position_error_m=position_error,
            heading_error_deg=math.degrees(heading_error),
        )
        return passed

    def flight_active(self, now):
        return (
            self.status_fresh(now)
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and not self.status.failsafe
        )

    def capture_alignment(self, snap, map_ready):
        if self.alignment is not None:
            return
        if not (map_ready and self.stable() and self.pacer.fresh(time.monotonic())):
            self.alignment_settled_since = None
            self.alignment_capture_stamp = None
            self.alignment_capture_scans = 0
            return
        if self.alignment_settled_since is None:
            self.alignment_settled_since = self.pacer.total_s
        if self.alignment_capture_stamp is None or snap.stamp > self.alignment_capture_stamp:
            self.alignment_capture_stamp = snap.stamp
            self.alignment_capture_scans += 1
        if (
            self.pacer.total_s - self.alignment_settled_since < 2.0
            or self.alignment_capture_scans < 3
        ):
            return
        p = self.position
        self.alignment = FrameAlignment(snap.pose[:2], snap.pose[2], [p.x, p.y], p.heading)
        self.alignment_counters = [p.xy_reset_counter, p.heading_reset_counter]
        self.world_pose = self.alignment.world([p.x, p.y])
        self.follower.hold(self.world_pose)
        if self.home_world_xy is None:
            self.home_world_xy = self.world_pose.copy()
            history = getattr(self.dashboard, "history", None)
            if history:
                history.event(
                    "home_captured",
                    "First stable aligned hover saved as home",
                    xy=self.home_world_xy.tolist(),
                    simulation_s=self.pacer.total_s,
                )
        self.say("World/PX4 alignment captured after steady hover and fresh scans.")

    def retire_obsolete_goal(self, error):
        if not self.exploration.enabled or error not in (
            "Choose an observed destination outside the obstacle margin",
            "No route through currently observed free space",
            "Route shortcut crosses insufficiently observed space",
        ):
            return False
        self.stop("Previous frontier is no longer eligible; selecting from the updated map", "HOLD")
        self.target_goal = None
        self.exploration.phase = "SELECTING"
        self.exploration.reason = error
        if self.mission.enabled and self.mission.phase == "PAUSED":
            self.mission.phase = "EXPLORING"
        return True

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

    def stop(self, reason, phase="PAUSED", clear_goal=False, auto_recover=False):
        history = getattr(self.dashboard, "history", None)
        if hasattr(self, "diagnostics") and (
            (self.follower.phase, self.follower.reason) != (phase, reason)
            or phase == "PAUSED"
            and self.exploration.phase != "PAUSED"
        ):
            self.diagnostics.emit(
                "stop" if phase == "PAUSED" else "controller_transition",
                reason=reason,
                phase=phase,
                clock=self.trajectory_clock.report(time.monotonic()),
                tracking=getattr(self.follower, "tracking", None),
                map_health=getattr(self, "map_health", None),
            )
        if history and (
            (self.follower.phase, self.follower.reason) != (phase, reason)
            or self.exploration.phase != "PAUSED"
            and phase == "PAUSED"
        ):
            history.event(
                "stop_decision",
                reason,
                phase=phase,
                simulation_s=self.pacer.total_s,
                goal=self.target_goal.tolist() if self.target_goal is not None else None,
                pose_check=getattr(self, "pose_check", None),
                map_health=getattr(self, "map_health", None),
                tracking=getattr(self.follower, "tracking", None),
            )
        self.auto_recover = bool(
            auto_recover and (self.mission.enabled or self.exploration.enabled)
        )
        self.recovery_stamp = None
        self.recovery_scans = 0
        self.recovery_started_s = None
        if self.world_pose is not None:
            self.follower.hold(self.world_pose, reason, phase)
        self.plan_generation += 1
        self.settled_since = None
        if clear_goal:
            self.target_goal = None
            self.returning_home = False
            self.exploration.cancel(reason)
            if self.mission.enabled:
                self.mission.cancel(reason)
        elif phase == "PAUSED":
            self.exploration.pause(reason)
            if self.mission.enabled and self.mission.phase != "PAUSED":
                self.mission_resume_phase = (
                    "VERIFYING"
                    if self.mission.phase in ("VERIFYING", "SAVING", "LAND_REQUESTED")
                    else self.mission.phase
                )
                self.mission.phase = "PAUSED"
                self.mission.reason = reason
                self.mission.reset_confirmation()
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
                path = PreparedRoute(plan(snap, start, goal), snap.model, self.settings)
                self.plan_results.put((token, path, None, None))
            except ValueError as exc:
                self.plan_results.put((token, None, str(exc), None))

        self.plan_thread = threading.Thread(target=work, daemon=True)
        self.plan_thread.start()

    def launch_frontier(self, snap):
        ledger = self.exploration.frontier_ledger
        if hasattr(ledger, "ready_for") and not ledger.ready_for(snap):
            return
        if self.plan_thread is not None and self.plan_thread.is_alive():
            return
        self.plan_generation += 1
        token = self.plan_generation
        start = self.world_pose.copy()
        visited = list(self.exploration.visited)
        trail = list(self.exploration.trail)
        catalog = self.exploration.frontier_ledger.snapshot()
        active_id = self.exploration.frontier_ledger.active_id
        mission_requested = self.mission.enabled
        self.target_goal = None
        self.follower.hold(start, "Selecting frontier viewpoint", "PLANNING")

        def work():
            try:
                choice = choose_frontier(
                    snap, start, visited, trail, catalog=catalog, active_id=active_id
                )
                path = PreparedRoute(choice.path, snap.model, self.settings)
                self.plan_results.put((token, path, None, choice))
            except ValueError as exc:
                if mission_requested:
                    evidence = assess_completion(snap, start)
                    self.plan_results.put((token, evidence, None, ("mission_check", snap.stamp)))
                else:
                    evidence = assess_completion(snap, start)
                    self.plan_results.put(
                        (token, evidence, str(exc), ("exploration_check", snap.stamp))
                    )

        self.plan_thread = threading.Thread(target=work, daemon=True)
        self.plan_thread.start()

    def launch_mission_check(self, snap):
        if self.plan_thread is not None and self.plan_thread.is_alive():
            return
        self.plan_generation += 1
        token = self.plan_generation
        start = self.world_pose.copy()
        self.last_completion_stamp = snap.stamp
        self.mission.phase = "VERIFYING"
        self.mission.reason = "Checking whether the observed component is enclosed"

        def work():
            evidence = assess_completion(snap, start)
            self.plan_results.put((token, evidence, None, ("mission_check", snap.stamp)))

        self.plan_thread = threading.Thread(target=work, daemon=True)
        self.plan_thread.start()

    def launch_reconsideration(self, snap):
        """Evaluate a new map while the existing control stream keeps running."""
        ledger = self.exploration.frontier_ledger
        if hasattr(ledger, "ready_for") and not ledger.ready_for(snap):
            return
        if self.reconsider_thread is not None and self.reconsider_thread.is_alive():
            return
        token = self.plan_generation
        current = self.target_goal.copy()
        visited = list(self.exploration.visited)
        trail = list(self.exploration.trail)
        catalog = self.exploration.frontier_ledger.snapshot()
        active_id = self.exploration.frontier_ledger.active_id
        self.reconsider_mark = self.pacer.total_s
        self.reconsider_stamp = snap.stamp
        self.exploration.reconsiderations += 1
        reference = self.follower.target.copy()
        prefix = CommittedPrefix(self.follower, self.settings.exploration_commit_horizon_s)
        current_route = [reference.tolist()] + [
            p.tolist() for p in self.follower.path[self.follower.index :]
        ]

        def work():
            try:
                gain = visible_unknown(
                    snap.model.grid, snap.model.cell(current), snap.model.resolution
                )
                choice = choose_frontier(
                    snap,
                    prefix.anchor,
                    visited,
                    trail,
                    catalog=catalog,
                    active_id=active_id,
                    current_goal=current,
                )
                choice.path = PreparedRoute(
                    prefix.points + choice.path[1:], snap.model, self.settings
                )
                choice.path.commitment = prefix
                choice.route_length_m += prefix.length
                choice.selection_reason += f"; {prefix.length:.1f} m committed prefix"
                if snap.model.cell(choice.goal) == snap.model.cell(current):
                    _, _, cost = routing_geometry(snap)

                    def route_cost(route):
                        return sum(
                            segment_cost(snap.model.cell(a), snap.model.cell(b), cost)
                            for a, b in zip(route, route[1:])
                        )

                    choice.path.recenter_improves = (
                        route_cost(choice.path) < route_cost(current_route) * 0.95
                    )
                self.reconsider_results.put((token, current, gain, choice, None))
            except ValueError as exc:
                self.reconsider_results.put((token, current, None, None, str(exc)))

        self.reconsider_thread = threading.Thread(target=work, daemon=True)
        self.reconsider_thread.start()

    def reconsideration_tick(self, snap):
        session = self.exploration
        if not (
            session.enabled
            and session.continuous
            and session.phase == "TRAVELLING"
            and self.follower.phase == "NAVIGATING"
            and self.follower.braking_envelope is None
            and self.target_goal is not None
        ):
            return
        try:
            while True:
                token, previous, gain, choice, error = self.reconsider_results.get_nowait()
                if token != self.plan_generation or not np.array_equal(previous, self.target_goal):
                    continue
                session.current_target_gain_m2 = gain
                if error:
                    session.reconsider_reason = error
                    continue
                same_goal = snap.model.cell(choice.goal) == snap.model.cell(self.target_goal)
                # Preserve the moving reference. Validate both physical-to-reference
                # space and the updated route against the newest footprint map.
                try:
                    if isinstance(choice.path, PreparedRoute):
                        commitment = choice.path.commitment
                        path = (
                            commitment.rebase(choice.path, self.follower)
                            if commitment is not None
                            else choice.path.with_start(self.follower.target)
                        )
                    else:
                        path = [self.follower.target.tolist()] + choice.path[1:]
                except ValueError as exc:
                    session.reconsider_reason = str(exc)
                    continue
                if not (
                    route_safe(snap, path)
                    and segment_safe(
                        snap.body_free, snap.model.cell(self.world_pose), snap.model.cell(path[0])
                    )
                ):
                    session.reconsider_reason = (
                        "New route no longer observed; retaining current route"
                    )
                    continue
                session.frontier_ledger.apply_choice(choice)
                if same_goal:
                    # A wider observed corridor can move its center while the
                    # useful destination stays the same. Re-evaluate the route
                    # itself, rather than waiting for a different frontier.
                    improves = getattr(path, "recenter_improves", None)
                    if improves is None:  # Plain-route fixtures and older callers.
                        _, _, cost = routing_geometry(snap)
                        current = [self.follower.target.tolist()] + [
                            p.tolist() for p in self.follower.path[self.follower.index :]
                        ]

                        def route_cost(route):
                            return sum(
                                segment_cost(snap.model.cell(a), snap.model.cell(b), cost)
                                for a, b in zip(route, route[1:])
                            )

                        improves = route_cost(path) < route_cost(current) * 0.95
                    if not improves:
                        session.reconsider_reason = (
                            "Same destination; current route remains competitive"
                        )
                        continue
                    self.follower.retarget(path)
                    self.plan_generation += 1
                    session.reconsider_reason = (
                        "Route updated toward known-space centers; destination unchanged; "
                        "near-term route committed"
                    )
                    history = getattr(self.dashboard, "history", None)
                    if history:
                        history.event(
                            "route_recentered",
                            session.reconsider_reason,
                            goal=self.target_goal.tolist(),
                            path=path,
                            simulation_s=self.pacer.total_s,
                        )
                    continue
                self.target_goal = np.array(choice.goal)
                self.follower.retarget(path)
                self.plan_generation += 1
                self.settled_since = None
                session.target_switches += 1
                session.expected_gain_m2 = choice.expected_new_area_m2
                session.current_target_gain_m2 = choice.expected_new_area_m2
                session.frontier_count = choice.frontier_cells
                session.reason = session.selection_reason = choice.selection_reason
                session.corner_xy = choice.corner_xy
                session.hidden_area_m2 = choice.hidden_area_m2
                session.wall_endings_xy = choice.wall_endings_xy or []
                session.reconsider_reason = (
                    "Target changed from a fresh map; near-term route committed"
                )
                self.say(
                    f"Frontier changed during travel: ({previous[0]:.1f}, {previous[1]:.1f}) "
                    f"-> ({choice.goal[0]:.1f}, {choice.goal[1]:.1f}) m"
                )
        except Empty:
            pass
        if (
            (
                self.reconsider_mark is None
                or self.pacer.total_s - self.reconsider_mark
                >= self.settings.exploration_replan_interval_s
            )
            and snap.stamp != self.reconsider_stamp
            and self.plan_results.empty()
            and (self.plan_thread is None or not self.plan_thread.is_alive())
        ):
            self.launch_reconsideration(snap)

    def launch_map_save(self, snap):
        self.plan_generation += 1
        token = self.plan_generation
        evidence = self.mission.evidence
        history = dict(
            visited_viewpoints=list(self.exploration.visited),
            measured_trail=list(self.exploration.trail),
            selection_policy="Nearest reachable frontier by planned flight route length",
        )
        folder = (
            Path(__file__).parent
            / "logs"
            / ("mission_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"))
        )

        def work():
            try:
                saved = save_map(snap, folder, evidence)
                (folder / "history.json").write_text(
                    json.dumps(history, indent=2) + "\n", encoding="utf-8"
                )
                self.plan_results.put((token, saved, None, "mission_saved"))
            except (OSError, ValueError) as exc:
                self.plan_results.put((token, None, str(exc), "mission_saved"))

        self.plan_thread = threading.Thread(target=work, daemon=True)
        self.plan_thread.start()

    def mission_tick(self, now, snap, map_ready):
        mission = self.mission
        if not mission.enabled or mission.phase == "PAUSED":
            return
        if not map_ready:
            self.stop("Mission needs a fresh aligned map; Resume when restored")
            return
        if mission.phase == "LAND_REQUESTED" and now - mission.request_started > 10.0:
            mission.incomplete("PX4 did not enter Land mode within 10 seconds; holding")
            self.follower.hold(self.world_pose, mission.reason, "HOLD")
            return
        if (
            mission.phase == "LAND_REQUESTED"
            and mission.land_attempts == 0
            and not np.array_equal(snap.model.grid, self.certified_grid)
        ):
            mission.phase = "VERIFYING"
            mission.reset_confirmation()
            self.last_completion_stamp = None
            return
        idle = self.plan_results.empty() and (
            self.plan_thread is None or not self.plan_thread.is_alive()
        )
        if mission.phase == "VERIFYING" and idle and self.stable():
            if snap.stamp != self.last_completion_stamp:
                self.launch_mission_check(snap)
        elif mission.phase == "SAVING" and idle and self.stable():
            if np.array_equal(snap.model.grid, self.certified_grid):
                self.launch_map_save(snap)
            else:
                mission.phase = "VERIFYING"
                mission.reset_confirmation()
                self.last_completion_stamp = None
        elif (
            mission.phase == "LAND_REQUESTED"
            and self.stable()
            and self.state.motion_ready(now)
            and self.pacer.fresh(now)
            and self.trajectory_clock.fresh(now)
            and self.land_pub.get_subscription_count()
            and mission.landing_due(now)
        ):
            message = VehicleCommand()
            message.timestamp = PX4_RECEIPT_TIMESTAMP
            message.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
            # NaN means land at current position, without a global destination.
            message.param4 = message.param5 = message.param6 = message.param7 = math.nan
            message.target_system = message.source_system = 1
            message.target_component = 1
            message.source_component = 191
            message.from_external = True
            self.land_pub.publish(message)
            self.say("Completion verified and map saved; requesting PX4 Land at current position")

    def recovery_tick(self, now, snap, map_ready):
        if not (
            self.auto_recover
            and (
                self.mission.enabled
                and self.mission.phase == "PAUSED"
                or self.exploration.enabled
                and self.exploration.phase == "PAUSED"
            )
        ):
            return
        if not (
            map_ready
            and self.flight_active(now)
            and self.stable()
            and self.state.motion_ready(now)
            and self.pacer.fresh(now)
            and self.trajectory_clock.fresh(now)
        ):
            self.recovery_stamp = None
            self.recovery_scans = 0
            self.recovery_started_s = None
            return
        if self.recovery_started_s is None:
            self.recovery_started_s = self.pacer.total_s
        if self.recovery_stamp is None or snap.stamp > self.recovery_stamp:
            self.recovery_stamp = snap.stamp
            self.recovery_scans += 1
        if (
            self.recovery_scans >= 3
            and self.pacer.total_s - self.recovery_started_s >= 1.0
            and self.commands.empty()
        ):
            try:
                self.commands.put_nowait(dict(action="resume"))
            except Full:
                return
            self.auto_recover = False
            self.say(
                "Three fresh aligned scans and stable hover restored; replanning exploration route"
            )

    def complete_exploration(self, snap, evidence):
        reason = "Exploration complete: " + evidence.reason
        self.stop(reason, "COMPLETE")
        self.target_goal = None
        session = self.exploration
        session.enabled = False
        session.phase = "COMPLETE"
        session.reason = reason
        session.wait_stamp = None
        session.frontier_count = 0
        session.current_target_gain_m2 = 0.0
        if self.mission.enabled:
            if self.mission.confirm(evidence, snap.stamp, self.pacer.total_s):
                self.certified_grid = snap.model.grid.copy()
            session.phase = self.mission.phase
            session.reason = self.mission.reason
            self.follower.hold(self.world_pose, self.mission.reason, "HOLD")

    def exploration_tick(self, now, snap, map_ready):
        session = self.exploration
        if map_ready:
            try:
                changes = session.observe(snap, self.pacer.total_s)
            except ValueError as exc:
                self.stop(str(exc))
                return
            history = getattr(self.dashboard, "history", None)
            if changes and history:
                history.event(
                    "frontier_catalog_updated",
                    "Fresh map updated frontier boundaries",
                    **changes,
                    simulation_s=self.pacer.total_s,
                )
        if session.phase == "PAUSED":
            return
        if map_ready and (session.enabled or session.phase == "FINISHED"):
            ledger = session.frontier_ledger
            evidence = ledger.completion_for(snap) if hasattr(ledger, "completion_for") else None
            if evidence is not None and evidence.complete:
                self.complete_exploration(snap, evidence)
                return
        if not session.enabled:
            return
        if not map_ready:
            self.stop("Exploration needs a fresh map; Resume when ready")
            return
        if session.phase == "SCANNING":
            session.scanned(snap)
            if not session.enabled:
                self.target_goal = None
                self.stop(session.reason, "HOLD")
                if self.mission.enabled:
                    self.launch_mission_check(snap)
                return
        if (
            session.phase == "SELECTING"
            and self.stable()
            and self.state.motion_ready(now)
            and self.pacer.fresh(now)
            and self.trajectory_clock.fresh(now)
            and self.plan_results.empty()
            and (self.plan_thread is None or not self.plan_thread.is_alive())
        ):
            self.launch_frontier(snap)

    def commands_tick(self, now, snap, map_ready):
        pending = []
        while True:
            try:
                pending.append(self.commands.get_nowait())
            except Empty:
                break
        holds = [
            item
            for item in pending
            if item["action"] == "hold" and now <= item.get("_deadline", math.inf)
        ]
        if holds:
            chosen = holds[-1]
            for item in pending:
                if item is not chosen:
                    self.command_feedback(item, "cancelled", "Superseded by operator Hold")
            pending = [chosen]
        for item in pending:
            action = item["action"]
            if now > item.get("_deadline", math.inf):
                self.command_feedback(
                    item, "expired", "Request expired before the controller could act"
                )
                continue
            if action == "hold":
                self.command_feedback(self.active_ui_command, "cancelled", "Operator selected Hold")
                self.active_ui_command = item
                self.stop("Operator hold", "HOLD", True)
                self.command_feedback(item, "accepted", "Hold accepted; waiting for stable hover")
                continue
            if action == "home" and self.home_world_xy is None:
                self.command_feedback(
                    item,
                    "rejected",
                    "Home is not captured yet; establish the first stable mapped hover",
                )
                continue
            if not (
                self.ready_samples >= 50
                and self.flight_active(now)
                and map_ready
                and self.alignment is not None
                and self.state.motion_ready(now)
                and self.pacer.fresh(now)
                and self.trajectory_clock.fresh(now)
                and (
                    self.stable()
                    or (
                        action == "home"
                        and abs(self.position.z + 1.0) <= 0.15
                        and self.position.v_xy_valid
                        and self.position.v_z_valid
                        and all(
                            math.isfinite(v)
                            for v in (self.position.vx, self.position.vy, self.position.vz)
                        )
                    )
                )
            ):
                self.dashboard.note = self.flight_readiness(now, map_ready)["next_step"]
                self.command_feedback(item, "rejected", self.dashboard.note)
                continue
            if action == "resume" and self.follower.phase != "PAUSED":
                self.command_feedback(item, "rejected", "There is no paused goal to resume")
                continue
            if action in ("explore", "mission") and self.exploration.enabled:
                self.command_feedback(
                    item,
                    "rejected",
                    "Exploration already exists; use Resume if paused, or Hold before starting a new session",
                )
                continue
            self.command_feedback(
                self.active_ui_command, "cancelled", "Replaced by a new operator command"
            )
            self.active_ui_command = item
            self.command_feedback(item, "accepted", "Accepted; preparing " + action)
            if action == "home":
                self.exploration.cancel("optimizedRoutePlanner: return home selected")
                if self.mission.enabled:
                    self.mission.cancel("Return home replaces automatic exploration and landing")
                self.returning_home = True
                self.target_goal = self.home_world_xy.copy()
                self.stop(
                    "optimizedRoutePlanner: braking to hover before planning home", "PLANNING"
                )
                # The control loop launches the planner after measured motion settles.
            elif action == "goal":
                self.returning_home = False
                self.exploration.cancel("Manual destination selected")
                if self.mission.enabled:
                    self.mission.cancel("Manual destination selected")
                self.target_goal = np.array(item["point"], dtype=float)
                self.stop("Goal selected", "PLANNING")
                self.launch_plan(snap)
            elif action in ("explore", "mission"):
                self.returning_home = False
                if action == "mission":
                    self.mission.start()
                    self.last_completion_stamp = None
                elif self.mission.enabled:
                    self.mission.cancel("Exploration without automatic landing selected")
                self.exploration.start(
                    snap, continuous=item.get("continuous", True), simulation_s=self.pacer.total_s
                )
                self.reconsider_mark = self.reconsider_stamp = None
                self.target_goal = None
                self.stop("Exploration started", "PLANNING")
                self.launch_frontier(snap)
            elif action == "resume":
                if self.mission.enabled and self.mission.phase == "PAUSED":
                    self.mission.phase = self.mission_resume_phase or "EXPLORING"
                if self.exploration.enabled and self.exploration.phase == "PAUSED":
                    self.exploration.phase = (
                        "TRAVELLING" if self.target_goal is not None else "SELECTING"
                    )
                if self.target_goal is not None:
                    self.stop("Resuming goal", "PLANNING")
                    self.launch_plan(snap)

    def command_feedback(self, item, status, reason):
        callback = getattr(self.dashboard, "acknowledge_command", None)
        if item and callback:
            callback(item.get("request_id"), status, reason)

    def update_command_progress(self):
        item = self.active_ui_command
        if not item:
            return
        phase = self.follower.phase
        status, reason = "running", self.follower.reason
        terminal = False
        if self.mission.phase in ("LANDING", "LANDED") and item["action"] == "mission":
            status, reason = (
                "completed" if self.mission.phase == "LANDED" else "running"
            ), self.mission.reason
            terminal = self.mission.phase == "LANDED"
        elif phase in ("PAUSED", "BLOCKED"):
            status = phase.lower()
        elif phase == "REACHED" and item["action"] in ("goal", "resume", "home"):
            status, terminal = "completed", True
        elif item["action"] == "hold" and self.stable():
            status, reason, terminal = "completed", "Stable hover confirmed", True
        elif (item["action"] == "explore" and not self.exploration.enabled) or (
            item["action"] == "mission" and not self.mission.enabled
        ):
            status = "completed" if self.exploration.phase == "COMPLETE" else "stopped"
            reason, terminal = self.exploration.reason, True
        self.command_feedback(item, status, reason)
        if terminal:
            self.active_ui_command = None

    def reject_pending_commands(self, reason):
        while True:
            try:
                item = self.commands.get_nowait()
            except Empty:
                return
            self.command_feedback(item, "rejected", reason)

    def publish_target(self):
        now = time.monotonic()
        retained_acceleration = self.acceleration_reference_ned.copy()
        self.velocity_reference_ned = np.zeros(2)
        self.acceleration_reference_ned = np.zeros(2)
        gap = now - self.last_callback if self.last_callback is not None else 0.0
        self.last_callback = now
        dt = self.trajectory_clock.consume(now)
        clock_ready = dt is not None
        dt = dt if clock_ready else 0.0
        decision = getattr(self.trajectory_clock, "last_decision", None)
        reference_stamp = decision["source_stamp_ns"] if decision is not None else None
        if self.sample_timeline is not None:
            self.sample_timeline.poll(now)
        if not clock_ready:
            self.reference_history.clear()
        imu_progress = self.pacer.total_s - self.pacer_mark
        self.unclocked_imu_s = (
            min(0.25, self.unclocked_imu_s + max(0, imu_progress))
            if clock_ready and dt == 0
            else 0.0
        )
        self.pacer_mark = self.pacer.total_s
        status_fresh = self.status_fresh(now) and not self.status.failsafe
        landing = self.mission.observe_landing(
            fresh=status_fresh,
            land_mode=status_fresh
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
            armed=status_fresh and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            disarmed=status_fresh
            and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED,
            landed=self.landed,
            landed_fresh=self.landed_wall is not None
            and self.landed_sim_s is not None
            and 0 <= now - self.landed_wall <= 10.0
            and 0 <= self.pacer.total_s - self.landed_sim_s <= 2.0,
        )
        if landing or self.mission.phase in ("LANDING", "LANDED"):
            self.reject_pending_commands("PX4 owns landing; reset before requesting another flight")
            # Native PX4 now owns descent. Do not publish a competing hover target.
            self.follower.phase = self.mission.phase
            self.follower.reason = self.mission.reason
            snap, failure = self.feed.get()
            self.update_dashboard(now, snap, False)
            return
        if not self.state.fresh(now):
            self.reject_pending_commands(
                "Fresh PX4 position is unavailable; no flight request was executed"
            )
            self.ready_samples = 0
            self.exploration.pause("Position unavailable; targets paused")
            if self.mission.enabled or self.follower.phase in (
                "NAVIGATING",
                "SETTLING",
                "PLANNING",
                "BLOCKED",
            ):
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
            self.reject_pending_commands("Another flight-target publisher is present")
            self.update_dashboard(now, None, False)
            return
        p = self.position
        snap, failure = self.feed.get()
        map_ready = snap is not None and not failure and snap.fresh(now)
        self.map_health = snap.health(now) if snap is not None else None
        transient_map_gap = (
            not map_ready
            and failure is None
            and snap is not None
            and (
                snap.valid
                or snap.reason
                in (
                    "Cloud missing or stale",
                    "Simulation clock missing or stopped",
                    "Confirming live scans",
                )
            )
        )
        if not map_ready and failure is None and self.map_health:
            h = self.map_health
            failure = (
                f"Map inputs unavailable: {snap.reason}; publication/cloud/clock ages "
                f"{h['publication_age_s']:.2f}/{h['cloud_age_s']:.2f}/{h['clock_age_s']:.2f} s"
            )
        if self.hold_yaw is None:
            self.hold_yaw = p.heading
        self.capture_alignment(snap, map_ready)
        if self.alignment is not None:
            self.world_pose = self.alignment.world([p.x, p.y])
            if map_ready and not self.map_pose_matches(snap):
                map_ready = False
                check = self.pose_check
                if check["reason"] == "position_error":
                    detail = f"position error {check['position_error_m']:.3f} m > 0.200 m"
                elif check["reason"] == "heading_error":
                    detail = f"heading error {check['heading_error_deg']:.1f} deg > 15.0 deg"
                elif check["reason"] == "history_gap":
                    detail = f"pose sample gap {abs(check['sample_offset_s']):.3f} s > 0.250 s"
                else:
                    detail = "missing PX4 pose history"
                failure = f"Capture-time world/PX4 check: {detail}; holding"
        if self.reference is None:
            self.reference = np.array(self.state.anchor[:2])
        active = self.follower.phase in ("NAVIGATING", "SETTLING", "PLANNING", "BLOCKED")
        if self.alignment is not None and not self.flight_active(now):
            self.hold_yaw = p.heading
            if not self.status_fresh(now):
                self.stop("PX4 status or delivered IMU stale; hold until Resume")
            else:
                if self.exploration.enabled:
                    self.exploration.cancel("PX4 left armed Offboard")
                if self.mission.enabled:
                    self.mission.cancel("PX4 left armed Offboard before mission landing")
                if active:
                    self.stop("PX4 left armed Offboard", "HOLD", True)
                else:
                    self.follower.hold(
                        self.world_pose, "Native PX4 mode; ready for Offboard", "HOLD"
                    )
        elif active and (not map_ready or not clock_ready or gap > 0.5 or gap < 0):
            self.stop(
                failure
                or (
                    "Simulation clock unavailable or discontinuous; hold until Resume"
                    if not clock_ready
                    else (
                        "Map data unavailable; hold until Resume"
                        if not map_ready
                        else "Control callback delayed"
                    )
                ),
                auto_recover=transient_map_gap and 0 <= gap <= 0.5,
            )
        elif active and (
            abs(p.z + 1.0) > 0.25
            or not p.v_xy_valid
            or not p.v_z_valid
            or not all(math.isfinite(v) for v in (p.vx, p.vy, p.vz))
        ):
            self.stop("Height tracking or velocity validity failed")
        self.recovery_tick(now, snap, map_ready)
        self.commands_tick(now, snap, map_ready)
        if self.flight_active(now):
            self.exploration_tick(now, snap, map_ready)
            self.mission_tick(now, snap, map_ready)
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
                    if isinstance(choice, tuple) and choice[0] == "exploration_check":
                        if choice[1] != snap.stamp:
                            continue
                        if path.complete:
                            self.complete_exploration(snap, path)
                            continue
                        choice = "exploration"
                    if isinstance(choice, tuple) and choice[0] == "mission_check":
                        if choice[1] != snap.stamp:
                            continue
                        if path.complete:
                            self.complete_exploration(snap, path)
                            continue
                        self.exploration.enabled = False
                        self.target_goal = None
                        if self.mission.confirm(path, choice[1], self.pacer.total_s):
                            self.certified_grid = snap.model.grid.copy()
                        self.follower.hold(self.world_pose, self.mission.reason, "HOLD")
                        self.exploration.phase = self.mission.phase
                        self.exploration.reason = self.mission.reason
                        continue
                    if choice == "mission_saved":
                        if error:
                            self.mission.incomplete("Could not save mission map: " + error)
                        elif np.array_equal(snap.model.grid, self.certified_grid):
                            self.mission.request_landing(path, now)
                        else:
                            self.mission.phase = "VERIFYING"
                            self.mission.reset_confirmation()
                            self.last_completion_stamp = None
                        self.follower.hold(self.world_pose, self.mission.reason, "HOLD")
                        continue
                    if error and choice == "exploration":
                        self.exploration.enabled = False
                        self.exploration.phase = "FINISHED"
                        self.exploration.reason = error
                        self.target_goal = None
                        self.stop(error, "HOLD")
                    elif error:
                        if self.retire_obsolete_goal(error):
                            continue
                        self.follower.hold(self.world_pose, error, "BLOCKED")
                        if self.exploration.enabled:
                            self.stop(error)
                    elif route_safe(snap, path):
                        if choice is not None:
                            self.exploration.frontier_ledger.apply_choice(choice)
                            self.target_goal = np.array(choice.goal)
                            self.exploration.phase = "TRAVELLING"
                            self.exploration.expected_gain_m2 = choice.expected_new_area_m2
                            self.exploration.current_target_gain_m2 = choice.expected_new_area_m2
                            self.reconsider_mark = self.pacer.total_s
                            self.reconsider_stamp = snap.stamp
                            self.exploration.frontier_count = choice.frontier_cells
                            self.exploration.reason = choice.selection_reason
                            self.exploration.selection_reason = choice.selection_reason
                            self.exploration.corner_xy = choice.corner_xy
                            self.exploration.hidden_area_m2 = choice.hidden_area_m2
                            self.exploration.wall_endings_xy = choice.wall_endings_xy or []
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
                if not remaining_safe(self.follower, snap, self.world_pose):
                    self.stop("Route changed; waiting to replan", "BLOCKED")
                self.last_map_scans = snap.scans
            if self.state.motion_ready(now) and self.pacer.fresh(now):
                self.reconsideration_tick(snap)
                self.exploration.record_pose(self.world_pose)
                observed_corridor = remaining_safe(self.follower, snap, self.world_pose)
                previous_target = (
                    self.follower.target.copy() if self.follower.target is not None else None
                )
                previous_velocity = self.follower.velocity.copy()
                tracking_reference = None
                self.tracking_alignment = dict(
                    mode="prediction_fallback", reason="clock_mapping_unavailable"
                )
                if self.sample_timeline is not None:
                    sample_stamp, failure_reason = self.sample_timeline.sample_time(p, now)
                    if sample_stamp is not None and (
                        reference_stamp is None
                        or not -100_000_000 <= reference_stamp - sample_stamp <= 250_000_000
                    ):
                        sample_stamp, failure_reason = None, "sample_reference_clock_skew"
                    if self.reference_history.epoch is not None and (
                        self.reference_history.epoch[0] != self.sample_timeline.clock.epoch
                        or self.reference_history.epoch[1] != self.follower.phase
                        or self.reference_history.epoch[2] != tuple(self.alignment_counters or [])
                        or self.reference_history.epoch[3] != self.state.clock_rebases
                    ):
                        self.reference_history.clear()
                    if sample_stamp is not None:
                        tracking_reference, alignment_mode = self.reference_history.at(sample_stamp)
                        self.tracking_alignment = dict(
                            mode=(
                                alignment_mode
                                if tracking_reference is not None
                                else "prediction_fallback"
                            ),
                            reason=None if tracking_reference is not None else alignment_mode,
                            sample_sim_ns=sample_stamp,
                            reference_sim_ns=reference_stamp,
                            sample_age_sim_s=(reference_stamp - sample_stamp) / 1e9,
                        )
                    else:
                        self.tracking_alignment["reason"] = failure_reason
                route_direction = self.follower.heading_direction()
                heading_error = (
                    math.remainder(p.heading - self.alignment.yaw_ned(route_direction), 2 * math.pi)
                    if route_direction is not None and np.linalg.norm(route_direction) > 1e-8
                    else None
                )
                self.follower.tick(
                    dt,
                    self.world_pose,
                    snap.hits,
                    observed_corridor=observed_corridor,
                    heading_error=heading_error,
                    measured_speed=math.hypot(p.vx, p.vy),
                    measured_velocity=self.alignment.world_velocity([p.vx, p.vy]),
                    braking_safe=lambda points: swept_space_safe(snap, points),
                    # DDS telemetry may precede the corresponding Zenoh clock
                    # packet. Delivered IMU periods give a bounded prediction
                    # interval, but never advance trajectory position twice.
                    tracking_dt=max(dt, self.unclocked_imu_s),
                    tracking_reference=tracking_reference,
                    motion_safe=lambda a, b: segment_safe(
                        snap.body_free, snap.model.cell(a), snap.model.cell(b)
                    ),
                )
                self.follower.speed_decision["tracking_alignment"] = self.tracking_alignment
                if self.follower.phase == "PAUSED":
                    self.stop(self.follower.reason, "PAUSED")
                if (
                    previous_target is not None
                    and self.follower.phase == "NAVIGATING"
                    and 0 < dt <= 0.5
                    and 0 <= gap <= 0.5
                ):
                    # Feed forward only the validated reference's actual advance,
                    # including headroom, braking and bend limits. A held/reset
                    # target never creates a velocity from a position jump.
                    world_velocity = (self.follower.target - previous_target) / dt
                    self.velocity_reference_ned = self.alignment.ned_velocity(world_velocity)
                    world_acceleration = acceleration_feedforward(
                        previous_velocity, world_velocity, dt, self.settings.acceleration_m_s2
                    )
                    self.acceleration_reference_ned = (
                        self.alignment.ned_velocity(world_acceleration)
                        if world_acceleration is not None
                        else np.full(2, math.nan)
                    )
                elif (
                    self.follower.phase == "NAVIGATING"
                    and clock_ready
                    and dt == 0
                    and self.trajectory_clock.fresh(now)
                    and self.pacer.fresh(now)
                    and 0 <= gap <= 0.5
                ):
                    # Sparse clock publications are not stop commands. Keep the
                    # validated velocity between samples while both clocks are
                    # fresh. Requiring an IMU packet in this exact callback
                    # pulsed feedforward to zero between normal packet batches.
                    # Clock freshness bounds this hold to 0.25 wall seconds;
                    # stale clocks, invalid state and stopped phases still clear it.
                    self.velocity_reference_ned = self.alignment.ned_velocity(
                        self.follower.velocity
                    )
                    self.acceleration_reference_ned = retained_acceleration
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
                    self.follower.reason = (
                        "Home reached; holding 1 m hover. Land here remains an explicit choice."
                        if self.returning_home
                        else "Destination reached. Choose another goal."
                    )
                    if self.exploration.enabled:
                        self.exploration.arrived(snap, self.target_goal)
                        self.follower.reason = (
                            "Exploration viewpoint reached; scanning before choosing again"
                        )
                    self.say(self.follower.reason)
        if self.alignment is not None and self.follower.target is not None:
            self.reference = self.alignment.ned(self.follower.target)
        self.update_yaw(dt, now, map_ready, gap)
        mode, target = hover_messages(PX4_RECEIPT_TIMESTAMP, *self.reference, 1.0, self.hold_yaw)
        # Position stays the primary Offboard control mode. PX4 adds finite XY
        # velocity as feedforward; vertical control remains position-only.
        target.velocity = [*map(float, self.velocity_reference_ned), math.nan]
        target.acceleration = [*map(float, self.acceleration_reference_ned), math.nan]
        self.mode_pub.publish(mode)
        self.target_pub.publish(target)
        if self.follower.target is not None:
            self.reference_history.record(
                reference_stamp,
                self.follower.target,
                (
                    self.alignment.world_velocity(self.velocity_reference_ned)
                    if self.alignment is not None
                    else [0, 0]
                ),
                (
                    self.sample_timeline.clock.epoch if self.sample_timeline is not None else None,
                    self.follower.phase,
                    tuple(self.alignment_counters or []),
                    self.state.clock_rebases,
                ),
            )
        self.update_dashboard(now, snap, map_ready)
        if hasattr(self, "diagnostics"):
            self.diagnostics.emit(
                "reference",
                sim_ns=reference_stamp,
                world_xy=(
                    self.follower.target.tolist()
                    if self.alignment is not None and self.follower.target is not None
                    else None
                ),
                ned_xy=self.reference.tolist(),
                velocity_ned_xy=self.velocity_reference_ned.tolist(),
                acceleration_ned_xy=finite(self.acceleration_reference_ned.tolist()),
                tracking_alignment=self.tracking_alignment,
                feedforward_held_between_samples=bool(
                    dt == 0 and np.linalg.norm(self.velocity_reference_ned) > 1e-8
                ),
                alignment=(
                    dict(
                        rotation=self.alignment.rotation.tolist(),
                        offset=self.alignment.offset.tolist(),
                        reset_counters=self.alignment_counters,
                    )
                    if self.alignment is not None
                    else None
                ),
                phase=self.follower.phase,
                armed_offboard=self.flight_active(now),
                trajectory_dt_s=dt,
                imu_progress_s=imu_progress,
                speed_decision=self.follower.speed_report(
                    (
                        math.hypot(p.vx, p.vy)
                        if p.v_xy_valid and all(math.isfinite(v) for v in (p.vx, p.vy))
                        else None
                    ),
                    np.linalg.norm(self.velocity_reference_ned),
                ),
                clock=self.trajectory_clock.report(now),
            )
            self.diagnostics.emit(
                "cycle",
                callback_gap_s=gap,
                callback_duration_s=time.monotonic() - now,
                position_receipt_age_s=(
                    now - self.state.last_progress if self.state.last_progress is not None else None
                ),
                clock_ready=clock_ready,
                clock=self.trajectory_clock.report(now),
            )
        if self.mode_pub.get_subscription_count() and self.target_pub.get_subscription_count():
            self.ready_samples += 1
            if self.ready_samples == 50:
                self.say(
                    "READY: select Offboard in PX4, then Navigate from the map. No repeated move commands."
                )
        else:
            self.ready_samples = 0
            self.stop("Waiting for PX4 target readers")

    def update_yaw(self, dt, now, map_ready, callback_gap):
        desired = None
        if (
            map_ready
            and self.flight_active(now)
            and self.state.motion_ready(now)
            and self.pacer.fresh(now)
            and self.trajectory_clock.fresh(now)
            and 0 <= callback_gap <= 0.5
            and self.alignment is not None
            and self.follower.phase == "NAVIGATING"
            and self.follower.target is not None
            and self.follower.index < len(self.follower.path)
        ):
            direction = self.follower.heading_direction()
            if direction is not None and np.linalg.norm(direction) > 0.03:
                desired = self.alignment.yaw_ned(direction)
        self.yaw_desired = desired
        self.hold_yaw, self.yaw_rate = smooth_yaw(
            self.hold_yaw,
            self.yaw_rate,
            desired,
            dt,
            math.radians(self.settings.yaw_rate_deg_s),
            math.radians(self.settings.yaw_acceleration_deg_s2),
        )

    def update_dashboard(self, now, snap, map_ready):
        self.update_command_progress()
        p = self.position
        direction = None
        if self.follower.path and self.world_pose is not None:
            direction = self.follower.path[self.follower.index] - self.world_pose
        ahead = (
            clearance_ahead(self.world_pose, direction, snap.hits)
            if snap is not None and direction is not None
            else None
        )
        readiness = {
            "flight-target connection": self.ready_samples >= 50,
            "live map aligned with the flight controller": map_ready,
            "coordinate alignment": self.alignment is not None,
            "armed Offboard mode": self.flight_active(now),
            "stable hover": self.stable(),
            "fresh position": self.state.motion_ready(now),
            "fresh simulation timing": self.pacer.fresh(now) and self.trajectory_clock.fresh(now),
        }
        report = dict(
            recorded_utc=datetime.now(timezone.utc).isoformat(),
            phase=self.follower.phase,
            reason=self.follower.reason,
            ready=all(readiness.values()),
            readiness_reason=(
                "Ready"
                if all(readiness.values())
                else "Waiting for: "
                + ", ".join(name for name, valid in readiness.items() if not valid)
            ),
            flight=self.flight_readiness(now, map_ready),
            path=(
                []
                if self.follower.phase in ("COMPLETE", "REACHED")
                else (
                    [self.follower.target.tolist(), self.follower.braking_envelope[1].tolist()]
                    if self.follower.braking_envelope is not None
                    else [q.tolist() for q in self.follower.path]
                )
            ),
            path_kind=(
                "reserved_braking_segment"
                if self.follower.braking_envelope is not None
                else "onward_route"
            ),
            braking_envelope_xy=(
                self.follower.braking_envelope.tolist()
                if self.follower.braking_envelope is not None
                else None
            ),
            trail=self.exploration.trail[-1000:],
            visited_viewpoints=list(self.exploration.visited),
            goal=self.target_goal.tolist() if self.target_goal is not None else None,
            route_planner="optimizedRoutePlanner",
            planning_policy="prefer_known_space_centers_and_clearance",
            home_xy=self.home_world_xy.tolist() if self.home_world_xy is not None else None,
            returning_home=self.returning_home,
            remaining_m=(
                float(np.linalg.norm(self.world_pose - self.target_goal))
                if self.world_pose is not None and self.target_goal is not None
                else None
            ),
            clearance_m=ahead,
            speed_m_s=self.follower.speed,
            speed_decision=self.follower.speed_report(
                (
                    math.hypot(p.vx, p.vy)
                    if p is not None
                    and p.v_xy_valid
                    and all(math.isfinite(v) for v in (p.vx, p.vy))
                    else None
                ),
                np.linalg.norm(self.velocity_reference_ned),
            ),
            velocity_reference_ned_xy=self.velocity_reference_ned.tolist(),
            acceleration_reference_ned_xy=finite(self.acceleration_reference_ned.tolist()),
            tracking_alignment=self.tracking_alignment,
            target_stream_hz=20,
            control_profile="position_with_route_velocity_feedforward",
            motion_policy="certified_stopping_space_then_replan",
            runtime=getattr(self, "runtime_status", None),
            yaw_mode="face_route_smoothly",
            yaw_reference_ned_rad=self.hold_yaw,
            yaw_desired_ned_rad=self.yaw_desired,
            yaw_rate_reference_rad_s=self.yaw_rate,
            yaw_measured_ned_rad=(
                float(p.heading) if p is not None and math.isfinite(p.heading) else None
            ),
            height_m=-float(p.z) if p is not None and math.isfinite(p.z) else None,
            armed_offboard=self.flight_active(now),
            clock_rebases=self.state.clock_rebases,
            reference_ned_xy=self.reference.tolist() if self.reference is not None else None,
            measured_world_xy=self.world_pose.tolist() if self.world_pose is not None else None,
            pose_check=getattr(self, "pose_check", None),
            map_health=getattr(self, "map_health", None),
            measured_speed_m_s=(
                math.hypot(p.vx, p.vy)
                if p is not None and p.v_xy_valid and all(math.isfinite(v) for v in (p.vx, p.vy))
                else None
            ),
            delivered_simulation_s=self.pacer.total_s,
            trajectory_simulation_s=self.trajectory_clock.total_s,
            trajectory_clock=self.trajectory_clock.report(now),
            tracking=self.follower.tracking,
            settings=asdict(self.settings),
            mission=self.mission.report(),
            automatic_map_recovery=self.auto_recover,
            reporting_error=self.reporting.latest()[1] if hasattr(self, "reporting") else None,
            exploration=dict(
                enabled=self.exploration.enabled,
                phase=self.exploration.phase,
                reason=self.exploration.reason,
                viewpoints_reached=self.exploration.reached,
                new_observed_area_m2=self.exploration.area_gain_m2,
                expected_new_area_m2=self.exploration.expected_gain_m2,
                frontier_cells=self.exploration.frontier_count,
                selection_reason=self.exploration.selection_reason,
                policy="persistent_frontiers_nearest_planned_flight_route",
                frontier_catalog=self.exploration.frontier_ledger.report(),
                corner_xy=self.exploration.corner_xy,
                hidden_area_m2=self.exploration.hidden_area_m2,
                wall_endings_xy=self.exploration.wall_endings_xy,
                continuous=self.exploration.continuous,
                reconsiderations=self.exploration.reconsiderations,
                target_switches=self.exploration.target_switches,
                current_target_gain_m2=self.exploration.current_target_gain_m2,
                reconsider_reason=self.exploration.reconsider_reason,
                distance_flown_m=self.exploration.distance_flown_m,
                elapsed_simulation_s=self.exploration.elapsed_s,
                observed_area_per_m=(
                    self.exploration.area_gain_m2 / self.exploration.distance_flown_m
                    if self.exploration.distance_flown_m >= 0.1
                    else None
                ),
            ),
        )
        self.dashboard.observe(report, snap, now)
        if hasattr(self, "reporting"):
            self.reporting.submit(now, (report, snap, now))

    def publish_reports(self, item):
        """No control state mutation; ROS visualization and disk I/O run here."""
        report, snap, captured = item
        if captured - self.last_background_report < 0.5:
            return
        self.last_background_report = captured
        self.runtime_status = RUNTIME.report()
        self.ros_map.publish(snap, report["path"], report["trail"])
        path = Path(__file__).parent / "logs" / "navigation_state.json"
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(report, allow_nan=False, indent=2) + "\n", encoding="utf-8")
        temp.replace(path)
        with path.with_name("navigation_trace.jsonl").open("a", encoding="utf-8") as trace:
            trace.write(json.dumps(report, allow_nan=False, separators=(",", ":")) + "\n")

    def flight_readiness(self, now, map_ready):
        status = self.status
        p = self.position
        status_fresh = self.status_fresh(now)
        telemetry = (
            self.state.motion_ready(now)
            and self.pacer.fresh(now)
            and self.trajectory_clock.fresh(now)
            and self.trajectory_clock.fresh(now)
        )
        armed = status_fresh and status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        disarmed = status_fresh and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        landed_fresh = (
            self.landed_wall is not None
            and self.landed_sim_s is not None
            and 0 <= now - self.landed_wall <= 10
            and 0 <= self.pacer.total_s - self.landed_sim_s <= 2
        )
        low_speed = (
            p is not None
            and p.v_xy_valid
            and p.v_z_valid
            and all(math.isfinite(v) for v in (p.vx, p.vy, p.vz))
            and math.hypot(p.vx, p.vy) <= 0.1
            and abs(p.vz) <= 0.1
        )
        command_link = self.land_pub.get_subscription_count() > 0
        takeoff = {
            "fresh PX4 state and simulation": status_fresh and telemetry,
            "PX4 preflight checks": status_fresh
            and not status.failsafe
            and status.pre_flight_checks_pass,
            "landed and disarmed": disarmed and landed_fresh and self.landed,
            "stationary near the local ground origin": low_speed
            and p is not None
            and math.isfinite(p.z)
            and abs(p.z) < 0.15,
            "PX4 command connection": command_link,
            "reset after completed mission": self.mission.phase
            not in ("LAND_REQUESTED", "LANDING", "LANDED"),
        }
        offboard = {
            "fresh PX4 state and simulation": status_fresh and telemetry,
            "armed without failsafe": armed and not status.failsafe,
            "stable 1 m hover": self.stable(),
            "live aligned map": map_ready and self.alignment is not None,
            "flight-target stream": self.ready_samples >= 50,
            "PX4 command connection": command_link,
        }

        def reason(checks):
            return "Waiting for: " + ", ".join(k for k, good in checks.items() if not good)

        if not status_fresh or not telemetry:
            next_step = (
                "Click Restart experiment to recover the PX4 position-origin change."
                if self.state.fault
                else "Waiting for fresh PX4 position and simulation. Keep Isaac playing."
            )
        elif disarmed:
            next_step = (
                "1. Click Take off to 1 m. This explicitly arms the simulated drone and climbs."
                if all(takeoff.values())
                else "On the ground. " + reason(takeoff)
            )
        elif not self.stable():
            next_step = (
                "Armed on the ground. Use Land here or Reset before another takeoff."
                if landed_fresh and self.landed
                else "Waiting for a stable 1 m hover. Mapping starts at this height."
            )
        elif not map_ready or self.alignment is None:
            next_step = "Waiting for fresh LiDAR and coordinate alignment at the 1 m hover."
        elif not self.flight_active(now):
            next_step = (
                "2. Click Enable navigation to hand control to the navigator (PX4 Offboard)."
            )
        else:
            next_step = "3. Ready: click Explore unseen space, Explore and land here, or choose a destination."
        modes = {
            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "Hold",
            VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: "Takeoff",
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "Land",
            VehicleStatus.NAVIGATION_STATE_OFFBOARD: "Offboard",
        }
        return dict(
            updated_monotonic_s=now,
            checks=dict(
                telemetry_fresh=bool(status_fresh and telemetry),
                preflight_ok=bool(takeoff["PX4 preflight checks"]),
                landed=bool(self.landed) if landed_fresh else None,
                stationary=bool(low_speed) if status_fresh and telemetry else None,
                ground_origin=bool(takeoff["stationary near the local ground origin"]),
                hover_stable=bool(self.stable()) if status_fresh and telemetry else None,
                map_aligned=bool(map_ready and self.alignment is not None),
                command_connected=bool(command_link),
                target_stream_ready=bool(self.ready_samples >= 50),
                origin_fault=bool(self.state.fault),
            ),
            armed=bool(armed),
            mode=modes.get(status.nav_state, "Other") if status_fresh else "Unavailable",
            can_takeoff=all(takeoff.values()),
            can_takeoff_reason=next_step if self.state.fault else reason(takeoff),
            can_enable_navigation=all(offboard.values()),
            can_enable_navigation_reason=reason(offboard),
            can_land=bool(armed and command_link),
            can_land_reason="Landing requires fresh armed PX4 state and a command connection",
            can_hold=bool(status_fresh and command_link),
            can_hold_reason="Waiting for fresh PX4 state and command connection",
            landed_disarmed=bool(disarmed and landed_fresh and self.landed),
            native_hold_stable=bool(
                status_fresh
                and status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                and low_speed
            ),
            next_step=next_step,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--observe", action="store_true", help="Map and route preview only; NO flight publishers"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="Optional bounded observer duration"
    )
    parser.add_argument("--floor-z", type=float, help="Override the configured floor height")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--settings", type=Path, help="Launch-time navigation settings JSON")
    args = parser.parse_args()
    if (
        (args.floor_z is not None and not math.isfinite(args.floor_z))
        or not math.isfinite(args.duration)
        or args.duration < 0
    ):
        parser.error("Invalid floor/duration")
    if args.duration and not args.observe:
        parser.error("Bounded duration is available only for observation")
    try:
        settings = load_settings(args.settings)
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))
    floor_z = settings.floor_z_m if args.floor_z is None else args.floor_z
    feed = MapFeed(
        floor_z,
        settings.near_range_m,
        settings.obstacle_margin_m,
        settings.body_radius_m,
        settings=settings,
    )
    dashboard = None
    node = None
    reload_requested = False
    try:
        commands = None if args.observe else Queue(maxsize=8)
        history = RunHistory(
            Path(__file__).parent
            / "logs"
            / ("decisions_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f") + ".jsonl")
        )
        dashboard = Dashboard(feed, commands, args.port, history=history)
        reload_context = (
            restore_reload(Path(__file__).parent, dashboard) if not args.observe else None
        )
        dashboard.report["settings"] = asdict(settings)
        print("Navigator:", dashboard.url, flush=True)
        rclpy.init()
        if rclpy.get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
            raise RuntimeError("Use tools/ros2_navigate.cmd")
        if args.observe:
            from rclpy.node import Node

            node = Node("isaac_map_viewer")
            visualization = RosMapPublisher(node, floor_z + 1.0)
            print("MAP PREVIEW ONLY: no PX4 targets, arming, or mode commands.", flush=True)
            end = time.monotonic() + args.duration if args.duration else math.inf
            while time.monotonic() < end:
                snap, failure = feed.get()
                visualization.publish(snap, dashboard.preview)
                dashboard.observe(dashboard.report, snap, time.monotonic())
                time.sleep(0.25)
        else:
            node = GoalNavigator(feed, dashboard, commands, settings)
            flight_backend = NativeFlightBackend()
            dashboard.flight_controls = FlightControls(
                lambda: dashboard.report,
                dashboard.acknowledge_command,
                flight_backend,
                enabled=lambda: dashboard.flight_available
                and not (dashboard.reset and dashboard.reset.report()["busy"]),
            )
            reset = dashboard.reset = ExperimentReset(backend=flight_backend)
            if reload_context:
                reset.state.update(reload_context["reset"])
                request_id = reload_context.get("request_id")
                if request_id:
                    dashboard.receipts.begin(dict(action="restart", request_id=request_id))
                    dashboard.acknowledge_command(
                        request_id,
                        "completed",
                        "Reset complete; updated navigator loaded, drone disarmed",
                    )
            handled_reset = True
            while rclpy.ok():
                if reset.quiesce.is_set() and not reset.released.is_set():
                    handled_reset = False
                    if node is not None:
                        node.destroy_node()
                        node = None
                    feed.close()
                    dashboard.feed = EmptyFeed()
                    dashboard.flight_available = False
                    dashboard.preview = []
                    dashboard.preview_goal = None
                    dashboard.report = dict(
                        phase="RESTARTING", reason="Resetting experiment", ready=False
                    )
                    while not commands.empty():
                        try:
                            pending = commands.get_nowait()
                            dashboard.acknowledge_command(
                                pending.get("request_id"),
                                "cancelled",
                                "Experiment reset cancelled the pending request",
                            )
                        except Empty:
                            break
                    dashboard.history.close()
                    dashboard.previous_history = dashboard.history
                    reset.released.set()
                if reset.report()["busy"]:
                    handled_reset = False
                if reset.done.is_set() and not handled_reset:
                    if reset.success:
                        prepare_reload(
                            Path(__file__).parent,
                            reset.report(),
                            dashboard.previous_history,
                            dashboard.reset_request_id,
                        )
                        reload_requested = True
                        break
                    elif node is None:
                        dashboard.report = dict(
                            phase="RESET_FAILED", reason=reset.report()["reason"], ready=False
                        )
                    handled_reset = True
                if node is not None:
                    rclpy.spin_once(node, timeout_sec=0.05)
                else:
                    time.sleep(0.05)
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
    return RELOAD_EXIT_CODE if reload_requested else 0


if __name__ == "__main__":
    sys.exit(main())
