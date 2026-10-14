"""Observe PX4 state for a bounded interval. No flight publishers or commands."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import get_rmw_implementation_identifier


def topic_name(base, message_type):
    version = getattr(message_type, "MESSAGE_VERSION", 0)
    return "/fmu/out/" + base + (f"_v{version}" if version else "")


def motion_stats(samples, end_wall):
    tail = [sample for sample in samples if end_wall - sample[0] <= 5.0]
    if len(tail) < 2:
        return None
    z_values = [sample[3] for sample in tail]
    return dict(
        window_wall_s=tail[-1][0] - tail[0][0],
        samples=len(tail),
        mean_z_ned_m=sum(z_values) / len(z_values),
        z_span_m=max(z_values) - min(z_values),
        max_abs_vz_m_s=max(abs(sample[6]) for sample in tail),
        max_horizontal_speed_m_s=max(math.hypot(sample[4], sample[5]) for sample in tail),
        horizontal_span_m=math.hypot(
            max(sample[1] for sample in tail) - min(sample[1] for sample in tail),
            max(sample[2] for sample in tail) - min(sample[2] for sample in tail),
        ),
        note="Up to five seconds of valid advancing position samples; "
        "these metrics alone do not prove a stable physical hover.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error("duration must be 1..60 wall-clock seconds")
    rclpy.init()
    node = rclpy.create_node("isaac_px4_state_probe")
    if get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
        node.destroy_node()
        rclpy.shutdown()
        parser.error("Use px4_state_probe.cmd to select the PX4 Fast DDS environment")
    streams = {}
    positions = []
    offboard_positions = []
    status_history = []
    start_wall = time.monotonic()
    subscriptions = []
    fields = {
        "vehicle_local_position": (
            VehicleLocalPosition,
            (
                "xy_valid",
                "z_valid",
                "v_xy_valid",
                "v_z_valid",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "heading",
                "heading_good_for_control",
                "xy_reset_counter",
                "z_reset_counter",
                "heading_reset_counter",
            ),
        ),
        "vehicle_status": (
            VehicleStatus,
            ("arming_state", "nav_state", "failsafe", "pre_flight_checks_pass"),
        ),
        "vehicle_land_detected": (
            VehicleLandDetected,
            ("landed", "ground_contact", "maybe_landed"),
        ),
    }
    for name, (message_type, wanted_fields) in fields.items():
        topic = topic_name(name, message_type)
        state = dict(
            topic=topic,
            messages=0,
            advances=0,
            backwards=0,
            stamp_us=None,
            arrival_wall=None,
            latest=None,
        )
        streams[name] = state

        def observe(message, state=state, wanted_fields=wanted_fields):
            stamp = int(message.timestamp)
            previous = state["stamp_us"]
            state["messages"] += 1
            if previous is not None:
                state["advances"] += int(stamp > previous)
                state["backwards"] += int(stamp < previous)
            # Duplicate packets must not refresh an apparently live state.
            if previous is None or stamp > previous:
                state["arrival_wall"] = time.monotonic()
            state["stamp_us"] = stamp
            values = {
                field: getattr(message, field) for field in wanted_fields if hasattr(message, field)
            }
            state["latest"] = {
                field: (None if isinstance(value, float) and not math.isfinite(value) else value)
                for field, value in values.items()
            }
            if "nav_state" in values:
                status = dict(
                    arming_state=values["arming_state"],
                    nav_state=values["nav_state"],
                    failsafe=values["failsafe"],
                )
                if not status_history or any(
                    status_history[-1][key] != value for key, value in status.items()
                ):
                    status_history.append(dict(wall_s=time.monotonic() - start_wall, **status))
            if "x" in values and values.get("xy_valid") and values.get("z_valid"):
                sample = [values.get(field) for field in ("x", "y", "z", "vx", "vy", "vz")]
                if all(value is not None and math.isfinite(value) for value in sample):
                    if previous is None or stamp > previous:
                        entry = (time.monotonic(), *sample)
                        positions.append(entry)
                        status = streams.get("vehicle_status", {}).get("latest") or {}
                        if (
                            status.get("nav_state") == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                            and status.get("arming_state") == VehicleStatus.ARMING_STATE_ARMED
                        ):
                            offboard_positions.append(entry)

        subscriptions.append(
            node.create_subscription(message_type, topic, observe, qos_profile_sensor_data)
        )
    started_utc = datetime.now(timezone.utc).isoformat()
    print(f"Observing PX4 for {args.duration:g} s; no flight outputs.", flush=True)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
        now = time.monotonic()
        issues = []
        for name, state in streams.items():
            arrival = state.pop("arrival_wall")
            state["last_progress_age_s"] = None if arrival is None else now - arrival
            state["publishers"] = node.count_publishers(state["topic"])
            if state["messages"] < 3 or state["advances"] < 2:
                issues.append(f"{name}: insufficient advancing messages")
            if arrival is None or now - arrival > 1:
                issues.append(f"{name}: no fresh progressing state within 1 s")
            if not state["stamp_us"] or state["backwards"]:
                issues.append(f"{name}: missing/zero/reset timestamp")
        position = streams["vehicle_local_position"]["latest"] or {}
        if not (
            position.get("xy_valid")
            and position.get("z_valid")
            and all(position.get(field) is not None for field in ("x", "y", "z", "heading"))
        ):
            issues.append("Position/heading missing, nonfinite or invalid")
        motion = motion_stats(positions, now)
        result = dict(
            started_utc=started_utc,
            middleware=get_rmw_implementation_identifier(),
            streams=streams,
            state_streams_pass=not issues,
            issues=issues,
            recent_motion=motion,
            status_history=status_history,
            offboard_nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            offboard_samples=len(offboard_positions),
            last_observed_offboard_motion=(
                motion_stats(offboard_positions, offboard_positions[-1][0])
                if offboard_positions
                else None
            ),
            setpoint_publishers={
                topic: node.count_publishers(topic)
                for topic in ("/fmu/in/offboard_control_mode", "/fmu/in/trajectory_setpoint")
            },
            note="Historical subscription evidence. Stream pass is not flight readiness; "
            "this tool has no flight publishers and never arms or selects a mode.",
        )
        encoded = json.dumps(result, indent=2, allow_nan=False)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(encoded + "\n", encoding="utf-8")
        print(encoded)
        return 0 if result["state_streams_pass"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
