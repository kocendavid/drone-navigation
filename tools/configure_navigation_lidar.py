"""Operator-run Isaac sensor setting for fixed-height mapped navigation."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import omni.usd

stage = omni.usd.get_context().get_stage()
path = "/World/quadcopter/chassis/Example_Rotary"
sensor = stage.GetPrimAtPath(path)
assert sensor.IsValid() and sensor.GetTypeName() == "OmniLidar", (
    "Expected the existing OmniLidar at " + path
)
near = sensor.GetAttribute("omni:sensor:Core:nearRangeM")
assert near.IsValid(), "Existing sensor near-range attribute missing"
previous = near.Get()
settings_path = Path(__file__).parents[1] / "config" / "navigation_settings.json"
desired = float(json.loads(settings_path.read_text(encoding="utf-8"))["near_range_m"])
assert math.isfinite(desired) and 0.1 <= desired <= 1.0, "Invalid configured sensor near range"
assert near.Set(desired), "Could not set navigation near range"
result = dict(
    recorded_utc=datetime.now(timezone.utc).isoformat(),
    sensor_path=path,
    previous_near_range_m=previous,
    near_range_m=near.Get(),
    note="Operator changed the existing sensor near range; scene is not saved automatically",
)
assert abs(float(result["near_range_m"]) - desired) < 1e-6
output = Path(__file__).parents[1] / "logs" / "navigation_sensor.json"
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
print("[Navigation LiDAR] Near range =", near.Get(), "m; timeline and flight mode unchanged")
