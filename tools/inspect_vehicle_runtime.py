"""Read-only live chassis pose and arm-joint inspection inside Isaac."""

import builtins
import json
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

stage = omni.usd.get_context().get_stage()
assert stage is not None, "Open the drone scene first"
body = stage.GetPrimAtPath("/World/quadcopter/chassis")
assert body.IsValid(), "Chassis prim missing"
transform = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
up = transform.TransformDir(Gf.Vec3d(0, 0, 1))
joints = []
for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/quadcopter")):
    if prim.IsA(UsdPhysics.Joint):
        joints.append(
            dict(
                path=str(prim.GetPath()),
                type=prim.GetTypeName(),
                lower=prim.GetAttribute("physics:lowerLimit").Get(),
                upper=prim.GetAttribute("physics:upperLimit").Get(),
            )
        )
report = dict(
    playing=omni.timeline.get_timeline_interface().is_playing(),
    body_world_xyz=list(transform.ExtractTranslation()),
    body_up_world=list(up),
    upright=up[2] > 0.98,
    joints=joints,
    note="Read-only live composed stage; no pose, physics or timeline changes",
)
bridge = getattr(builtins, "_px4_isaac_bridge", None)
if bridge is not None and not bridge.closed and bridge.view is not None:
    position, rotation, velocity, angular_velocity = bridge._body_state()
    physical_up = rotation[:, 2]
    report["physics"] = dict(
        body_world_xyz=position.tolist(),
        body_up_world=physical_up.tolist(),
        velocity_world_m_s=velocity.tolist(),
        angular_velocity_world_rad_s=angular_velocity.tolist(),
        upright=bool(physical_up[2] > 0.98),
    )
output = Path(__file__).parents[1] / "logs" / "vehicle_runtime.json"
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
print("[Vehicle inspection] Chassis XYZ:", tuple(report["body_world_xyz"]))
print("[Vehicle inspection] Up:", tuple(up), "; upright:", report["upright"])
if "physics" in report:
    print("[Vehicle inspection] Physics:", report["physics"])
print("[Vehicle inspection] Report:", output)
