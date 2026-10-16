"""Restore the flight-lab baseline: lock four folding arms in stopped Isaac.

Backs up the scene layer before edits. Changes only rotor_arm0..3 joint limits;
propeller joints, vehicle placement, timeline and PX4 state stay unchanged.
"""

from datetime import datetime
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import UsdPhysics

assert omni.timeline.get_timeline_interface().is_stopped(), "Stop Isaac before locking arms"
stage = omni.usd.get_context().get_stage()
assert stage is not None, "Open the drone scene first"
joints = []
for index in range(4):
    path = f"/World/quadcopter/joints/rotor_arm{index}"
    prim = stage.GetPrimAtPath(path)
    assert prim.IsValid() and prim.IsA(UsdPhysics.RevoluteJoint), f"Missing folding hinge: {path}"
    joints.append(UsdPhysics.RevoluteJoint(prim))

backup = (
    Path(__file__).parents[1]
    / "logs"
    / ("drone_before_arm_lock_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".usda")
)
backup.parent.mkdir(exist_ok=True)
assert stage.GetRootLayer().Export(str(backup)), "Could not back up scene layer"
for joint in joints:
    previous = (joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get())
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(0.0)
    print("[Arm lock]", joint.GetPath(), previous, "-> (0, 0) degrees")
print("[Arm lock] Backup:", backup, "; propeller joints unchanged; scene not saved automatically")
