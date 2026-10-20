"""Configure snapshot SENSOR sweeps while Isaac is stopped; no flight commands."""

from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import carb.settings
import omni.timeline
import omni.usd

PROJECT = Path(__file__).resolve().parents[1]
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

# Kit can cache old loader code across resets. Compile the settings helper's
# actual source, as the bridge loader does for the flight/sensor bridge.
source = PROJECT / "lidar_tools" / "sensor_config.py"
namespace = dict(
    __name__="lidar_tools.sensor_config", __package__="lidar_tools", __file__=str(source)
)
exec(compile(source.read_bytes(), str(source), "exec"), namespace)
configure_sensor = namespace["configure_sensor"]

stage = omni.usd.get_context().get_stage()
path = "/World/quadcopter/chassis/Example_Rotary"
sensor = stage.GetPrimAtPath(path)
assert sensor.IsValid() and sensor.GetTypeName() == "OmniLidar", (
    "Expected the existing OmniLidar at " + path
)
assert (
    omni.timeline.get_timeline_interface().is_stopped()
), "Stop the timeline through the ground-reset procedure before configuring LiDAR"
stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
backup = PROJECT / "logs" / ("lidar_before_compensation_" + stamp)
backup.mkdir(parents=True)
assert stage.GetRootLayer().Export(str(backup / "root.usda"))
assert stage.GetSessionLayer().Export(str(backup / "session.usda"))
desired = float(
    json.loads((PROJECT / "config/navigation_settings.json").read_text())["near_range_m"]
)
configured = configure_sensor(
    sensor, carb.settings.get_settings(), stopped=True, near_range=desired
)
result = dict(
    recorded_utc=datetime.now(timezone.utc).isoformat(),
    sensor_path=path,
    near_range_m=desired,
    backup=str(backup),
    **configured,
    note="Instantaneous SENSOR sweeps use the existing publisher's capture-end timestamps. Scene is not saved automatically.",
)
output = PROJECT / "logs" / "navigation_sensor.json"
output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
print(
    "[Navigation LiDAR] Snapshot compensation configured; SENSOR frame and native timestamps retained"
)
