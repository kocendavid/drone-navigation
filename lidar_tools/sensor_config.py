"""Stopped-timeline RTX settings shared by setup and its offline checks."""

import math
from .scan_motion import SCAN_RATE_HZ


# Validated instantaneous full sweeps: tick spacing is not exposure duration.
# Imported by the map worker so setup and confirmation use the same model.
SCAN_EXPOSURE_S = 0.0


SENSOR_VALUES = {
    "omni:sensor:Core:outputMotionCompensationState": "COMPENSATED",
    "omni:sensor:Core:outputFrameOfReference": "SENSOR",
    "omni:sensor:Core:accumulateOutputs": True,
    # The installed SDK's rolling SENSOR compensation failed the yaw replay.
    # A full sweep at frame end avoids mixing poses within one map scan.
    "omni:sensor:Core:instantLidar": SCAN_EXPOSURE_S == 0,
    "omni:sensor:Core:scanRateBaseHz": int(SCAN_RATE_HZ),
    "omni:sensor:tickRate": SCAN_RATE_HZ,
}
RENDERER_VALUES = {
    "/renderer/raytracingMotion/allowRuntimeToggle": True,
    "/renderer/raytracingMotion/enableHydraEngineMasking": True,
    "/renderer/raytracingMotion/enabledForHydraEngines": "0,1,2,3,4",
    "/renderer/raytracingMotion/enabled": True,
}


def configure_sensor(sensor, settings, *, stopped, near_range):
    if not stopped:
        raise RuntimeError(
            "LiDAR compensation requires a stopped timeline and sensor reinitialization"
        )
    if not math.isfinite(near_range) or not 0.1 <= near_range <= 1:
        raise ValueError("Invalid LiDAR near range")
    desired = dict(SENSOR_VALUES, **{"omni:sensor:Core:nearRangeM": near_range})
    attributes = {name: sensor.GetAttribute(name) for name in desired}
    if not all(attribute.IsValid() for attribute in attributes.values()):
        raise RuntimeError(
            "The existing OmniLidar does not expose the required compensation settings"
        )
    before = dict(
        sensor={name: attribute.Get() for name, attribute in attributes.items()},
        renderer={name: settings.get(name) for name in RENDERER_VALUES},
    )
    for name, value in RENDERER_VALUES.items():
        settings.set(name, value)
        if settings.get(name) != value:
            raise RuntimeError("RTX renderer setting was not applied: " + name)
    for name, value in desired.items():
        if not attributes[name].Set(value) or attributes[name].Get() != value:
            raise RuntimeError("LiDAR setting was not applied: " + name)
    return dict(
        previous=before,
        sensor=desired,
        renderer=dict(RENDERER_VALUES),
        validation="Configured; moving-scan validation is still required",
    )
