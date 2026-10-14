"""Read authored scene gravity and verify real USD schema defaults offline."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / ".tools")]
from pxr import Sdf, Usd, UsdGeom, UsdPhysics
from px4_isaac.model import resolve_gravity_direction

stage = Usd.Stage.CreateInMemory()
UsdGeom.SetStageUpAxis(stage, "Z")
scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
raw = scene.GetGravityDirectionAttr().Get()
resolved = resolve_gravity_direction(raw, str(UsdGeom.GetStageUpAxis(stage)))
print("USD schema default direction:", tuple(raw), "resolved:", resolved.tolist())
assert resolved.tolist() == [0.0, 0.0, -1.0]
if len(sys.argv) > 1:
    layer = Sdf.Layer.FindOrOpen(sys.argv[1])
    prim = layer.GetPrimAtPath("/PhysicsScene")
    if prim:
        attr = prim.attributes.get("physics:gravityDirection")
        authored = attr.default if attr else None
        print("Saved scene authored direction:", authored, "(None uses the schema default)")
    else:
        print("Saved root layer has no /PhysicsScene opinion.")
