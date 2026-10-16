"""Read-only post-restart scene/settings audit, run inside Isaac via isaac_client."""

import json
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

PROJECT = Path(__file__).resolve().parents[1]
stage = omni.usd.get_context().get_stage()
assert stage is not None, "Open the experiment scene first"
settings = json.loads((PROJECT / "config/navigation_settings.json").read_text())
cache = UsdGeom.XformCache()
bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
sensor = stage.GetPrimAtPath("/World/quadcopter/chassis/Example_Rotary")
body = stage.GetPrimAtPath("/World/quadcopter/chassis")
assert sensor and body, "Expected sensor and chassis"
body_transform = cache.GetLocalToWorldTransform(body)
relative = cache.GetLocalToWorldTransform(sensor) * body_transform.GetInverse()
maze = stage.GetPrimAtPath("/World/maze")
geometry = []
for prim in Usd.PrimRange(maze):
    if prim.IsA(UsdGeom.Gprim):
        box = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
        geometry.append(
            dict(
                path=str(prim.GetPath()),
                bounds_min=list(box.GetMin()),
                bounds_max=list(box.GetMax()),
                collision=bool(prim.HasAPI(UsdPhysics.CollisionAPI)),
                collision_enabled=prim.GetAttribute("physics:collisionEnabled").Get(),
                visibility=str(UsdGeom.Imageable(prim).ComputeVisibility()),
            )
        )
joints = [
    dict(
        path=str(p.GetPath()),
        lower=p.GetAttribute("physics:lowerLimit").Get(),
        upper=p.GetAttribute("physics:upperLimit").Get(),
    )
    for p in Usd.PrimRange(stage.GetPrimAtPath("/World/quadcopter"))
    if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName().startswith("rotor_arm")
]
nodes = []
for prim in stage.TraverseAll():
    if prim.GetTypeName() == "OmniGraphNode":
        attrs = {
            a.GetName(): str(a.Get())
            for a in prim.GetAttributes()
            if a.GetName()
            in (
                "node:type",
                "inputs:resetOnStop",
                "inputs:frameId",
                "inputs:topicName",
                "inputs:fullScan",
                "inputs:type",
                "inputs:enabled",
                "inputs:renderProductPath",
            )
        }
        if any(t in str(attrs) for t in ("Lidar", "SimTime", "Clock", "Transform")):
            nodes.append(dict(path=str(prim.GetPath()), attributes=attrs))
old_room = stage.GetPrimAtPath("/World/simple_room")
report = dict(
    scene=stage.GetRootLayer().identifier,
    root_dirty=stage.GetRootLayer().dirty,
    session_dirty=stage.GetSessionLayer().dirty,
    stopped=omni.timeline.get_timeline_interface().is_stopped(),
    sensor_attributes={
        a.GetName(): str(a.Get())
        for a in sensor.GetAttributes()
        if a.GetName().startswith("omni:sensor:")
    },
    sensor_to_body_matrix=[list(row) for row in relative],
    chassis_xyz=list(body_transform.ExtractTranslation()),
    joints=joints,
    old_room_active=old_room.IsActive() if old_room else False,
    geometry=geometry,
    nodes=nodes,
    expected_settings=settings,
    note="Read-only composed stage audit; no scene, timeline or flight changes",
)
path = PROJECT / "logs/scene_settings_audit.json"
path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
print("[Scene audit]", path)
print(
    json.dumps(
        dict(
            scene=report["scene"],
            stopped=report["stopped"],
            chassis_xyz=report["chassis_xyz"],
            joints=joints,
            near_range=report["sensor_attributes"].get("omni:sensor:Core:nearRangeM"),
            old_room_active=report["old_room_active"],
            colliders=sum(g["collision"] and g["collision_enabled"] is not False for g in geometry),
        )
    )
)
