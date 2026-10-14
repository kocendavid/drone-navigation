"""Summarize recorded PhysX samples and the corresponding ROS command proof."""

import json
import sys
from pathlib import Path

directory = Path(sys.argv[1])
ros_report_path = Path(sys.argv[2])
samples = [
    json.loads(line) for line in (directory / "logs/isaac_bridge.jsonl").read_text().splitlines()
]
ros_report = json.loads(ros_report_path.read_text())
armed = [s for s in samples if s["armed"] and s["actuators_fresh"]]
z0 = samples[0]["position_enu_m"][2]
max_z = max(s["position_enu_m"][2] for s in samples)
result = {
    "scene": "Private Isaac PhysX chassis, mass 1.5 kg; user scene unmodified",
    "initial_z_m": z0,
    "max_z_m": max_z,
    "final_z_m": samples[-1]["position_enu_m"][2],
    "armed_log_samples": len(armed),
    "max_total_thrust_n": max(sum(s["thrust_n"]) for s in samples),
    "actuator_packets": samples[-1]["actuator_packets"],
    "sensor_packets": samples[-1]["sensor_packets"],
    "body_reacted": bool(armed and max_z - z0 > 0.2),
    "final_disarmed": not samples[-1]["armed"],
    "offboard_ros_proof_passed": ros_report["passed"],
    "ros_report": str(ros_report_path),
    "trace": str(directory / "logs/isaac_bridge.jsonl"),
}
result["passed"] = bool(
    result["body_reacted"] and result["final_disarmed"] and result["offboard_ros_proof_passed"]
)
(directory / "verified_result.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
raise SystemExit(0 if result["passed"] else 1)
