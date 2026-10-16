"""Operator-run stopped-scene preparation: maze collisions and old-room removal."""

from datetime import datetime, timezone
import json
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

PROJECT = Path(__file__).parents[1]
timeline = omni.timeline.get_timeline_interface()
assert timeline.is_stopped(), "Stop Isaac first; this script must not run during flight or Pause"
stage = omni.usd.get_context().get_stage()
assert stage is not None, "Open the maze scene first"
maze = stage.GetPrimAtPath("/World/maze")
floor = stage.GetPrimAtPath("/World/maze/Floor")
body = stage.GetPrimAtPath("/World/quadcopter/chassis")
assert (
    maze.IsValid() and floor.IsValid() and body.IsValid()
), "Expected maze, Floor and chassis prims"
assert (
    abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-6 and UsdGeom.GetStageUpAxis(stage) == "Z"
), "Expected metres and Z up"
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
floor_range = cache.ComputeWorldBound(floor).ComputeAlignedRange()
floor_z = float(floor_range.GetMax()[2])
assert floor_range.GetMax()[2] - floor_range.GetMin()[2] < 0.01, "Expected a flat maze floor"
settings = json.loads((PROJECT / "config" / "navigation_settings.json").read_text(encoding="utf-8"))
assert (
    abs(floor_z - settings["floor_z_m"]) < 0.02
), "Maze floor differs from configured floor; inspect before changing it"

backup = (
    PROJECT / "logs" / ("maze_before_prepare_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".usda")
)
backup.parent.mkdir(exist_ok=True)
assert stage.GetRootLayer().Export(str(backup)), "Could not save pre-change root-layer backup"
colliders = []
geometry = [prim for prim in Usd.PrimRange(maze) if prim.IsA(UsdGeom.Gprim)]
for prim in geometry:
    parent = prim
    while parent and parent != stage.GetPseudoRoot():
        assert not parent.HasAPI(
            UsdPhysics.RigidBodyAPI
        ), "Maze obstacle is dynamic; review its physics first"
        parent = parent.GetParent()
for prim in geometry:
    UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
    if prim.IsA(UsdGeom.Mesh):
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    colliders.append(str(prim.GetPath()))
old_room = stage.GetPrimAtPath("/World/simple_room")
if old_room.IsValid():
    old_room.SetActive(False)

transform = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
xyz = transform.ExtractTranslation()
up = transform.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
report = dict(
    recorded_utc=datetime.now(timezone.utc).isoformat(),
    floor_z_m=floor_z,
    colliders=colliders,
    old_room_active=old_room.IsActive() if old_room.IsValid() else None,
    body_world_xyz=list(xyz),
    body_up_world=list(up),
    upright=up[2] > 0.98,
    backup=str(backup),
    note="Collision settings changed; drone pose, arming, timeline and scene save unchanged",
)
(PROJECT / "logs" / "maze_preparation.json").write_text(
    json.dumps(report, indent=2) + "\n", encoding="utf-8"
)
print(
    "[Maze preparation] Colliders enabled:",
    len(colliders),
    "; old simple_room deactivated; floor Z =",
    floor_z,
)
print("[Maze preparation] Chassis XYZ:", tuple(xyz), "; upright:", report["upright"])
print("[Maze preparation] Root-layer backup:", backup, "; scene is not saved automatically")
