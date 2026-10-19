"""Remove only the legacy lab position override, leaving authored scene edits intact.

Run once inside stopped, disarmed Isaac. Backs up the session layer; does not
save the scene, start simulation, or change mass, joints, or chassis properties.
"""

import builtins
from datetime import datetime, timezone
import json
from pathlib import Path

import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

project = Path(__file__).resolve().parents[1]
stage = omni.usd.get_context().get_stage()
assert stage is not None, "Open the experiment scene first"
assert omni.timeline.get_timeline_interface().is_stopped(), "Stop Isaac first"
bridge = getattr(builtins, "_px4_isaac_bridge", None)
assert bridge is None or not bridge.transport.armed, "Disarm before changing placement"
session = stage.GetSessionLayer()
spec = session.GetPrimAtPath("/World/quadcopter")
assert spec is not None, "No legacy session override exists"
matrix = spec.attributes.get("xformOp:transform")
order = spec.attributes.get("xformOpOrder")
assert matrix is not None and order is not None, "No legacy matrix override exists"
assert list(order.default) == ["xformOp:transform"], "Transform order changed; inspect first"
baseline = json.loads((project / "logs/experiment_baseline.json").read_text(encoding="utf-8"))
assert stage.GetRootLayer().identifier == baseline["scene"], "Baseline belongs to another scene"
expected = Gf.Matrix4d(1)
expected.SetTranslateOnly(Gf.Vec3d(*baseline["start_xyz"]))
assert Gf.IsClose(matrix.default, expected, 1e-6), "Override was edited; inspect first"
stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
backup = project / "logs" / ("before_scene_placement_" + stamp + ".usda")
assert session.Export(str(backup)), "Session backup failed"
spec.RemoveProperty(matrix)
spec.RemoveProperty(order)
body = stage.GetPrimAtPath("/World/quadcopter/chassis")
pose = UsdGeom.XformCache().GetLocalToWorldTransform(body)
print(
    json.dumps(
        dict(
            source="scene_placement",
            start_xyz=list(pose.ExtractTranslation()),
            start_up=list(pose.TransformDir(Gf.Vec3d(0, 0, 1))),
            session_backup=str(backup),
            stopped=True,
        )
    )
)
