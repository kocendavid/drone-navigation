"""Print scene units, rigid bodies, joints and rotor candidates without changing USD."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / ".tools"))
from pxr import Usd, UsdGeom, UsdPhysics

stage = Usd.Stage.Open(sys.argv[1])
if stage is None:
    raise SystemExit("Could not open the scene.")
print("Python:", sys.version)
print("Scene:", stage.GetRootLayer().identifier)
print("Units:", UsdGeom.GetStageMetersPerUnit(stage), "Up:", UsdGeom.GetStageUpAxis(stage))
for prim in stage.Traverse():
    name = str(prim.GetPath())
    is_body = prim.HasAPI(UsdPhysics.RigidBodyAPI)
    if (
        is_body
        or prim.IsA(UsdPhysics.Joint)
        or any(token in name.lower() for token in ("rotor", "propeller", "chassis"))
    ):
        entry = {"path": name, "type": prim.GetTypeName(), "rigid_body": is_body}
        if prim.IsA(UsdGeom.Xformable):
            entry["world_xyz"] = list(
                UsdGeom.Xformable(prim)
                .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                .ExtractTranslation()
            )
        if is_body:
            body = UsdPhysics.RigidBodyAPI(prim)
            entry.update(
                enabled=body.GetRigidBodyEnabledAttr().Get(),
                kinematic=body.GetKinematicEnabledAttr().Get(),
            )
            if prim.HasAPI(UsdPhysics.MassAPI):
                mass = UsdPhysics.MassAPI(prim)
                entry["authored_mass_kg"] = mass.GetMassAttr().Get()
        if prim.IsA(UsdPhysics.Joint):
            joint = UsdPhysics.Joint(prim)
            entry["body0"] = [str(path) for path in joint.GetBody0Rel().GetTargets()]
            entry["body1"] = [str(path) for path in joint.GetBody1Rel().GetTargets()]
        print(json.dumps(entry))
print("References:")
for prim in stage.GetRootLayer().rootPrims:
    print(prim.name, prim.referenceList)
