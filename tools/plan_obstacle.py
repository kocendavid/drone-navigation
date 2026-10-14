"""Read-only placement values for an operator-created, stationary test box."""

import json
import math
from pathlib import Path
import sys

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom
from isaacsim.core.simulation_manager import SimulationManager

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from px4_isaac.model import rotation_wxyz


def numpy_value(value):
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def main():
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError(
            "Pause Isaac for obstacle placement, then run again. This script does not change the timeline."
        )
    stage = omni.usd.get_context().get_stage()
    if (
        stage is None
        or UsdGeom.GetStageUpAxis(stage) != "Z"
        or abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) > 1e-6
    ):
        raise RuntimeError("A metre-scale, Z-up scene is required")
    config = json.loads((PROJECT / "bridge_config.json").read_text(encoding="utf-8"))
    body = stage.GetPrimAtPath(config["body_prim_path"])
    sensor_path = config["body_prim_path"] + "/Example_Rotary"
    sensor = stage.GetPrimAtPath(sensor_path)
    if not body or not sensor:
        raise RuntimeError(f"Expected chassis and existing LiDAR: {sensor_path}")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    body_usd = cache.GetLocalToWorldTransform(body)
    sensor_in_body = cache.GetLocalToWorldTransform(sensor) * body_usd.GetInverse()
    sensor_axes = np.column_stack(
        [
            list(sensor_in_body.TransformDir(axis))
            for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
        ]
    )
    if not np.allclose(sensor_axes.T @ sensor_axes, np.eye(3), atol=1e-4):
        raise RuntimeError(
            "Sensor-to-body scaling is not unit scale; validate it before placing a measured obstacle"
        )
    sim = SimulationManager.get_physics_simulation_view()
    if sim is not None:
        view = sim.create_rigid_body_view(config["body_prim_path"])
        if view.count != 1:
            raise RuntimeError("Expected one live chassis rigid body")
        pose = numpy_value(view.get_transforms())[0].astype(float)
        body_position = pose[:3]
        body_rotation = rotation_wxyz(pose[[6, 3, 4, 5]])
        source = "paused live physics pose; sensor extrinsic from composed USD"
    elif timeline.get_current_time() == 0:
        body_position = np.asarray(body_usd.ExtractTranslation(), dtype=float)
        body_rotation = np.column_stack(
            [
                list(body_usd.TransformDir(axis))
                for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
            ]
        )
        source = "initial authored USD pose; physics view unavailable"
    else:
        raise RuntimeError(
            "No live physics pose is available; refusing to use a potentially stale USD pose"
        )
    flu_rotation = body_rotation @ rotation_wxyz(config["body_flu_quaternion_wxyz"])
    if abs(flu_rotation[2, 0]) > math.sin(math.radians(5)) or flu_rotation[2, 2] < math.cos(
        math.radians(5)
    ):
        raise RuntimeError("Use a level stationary drone for this first forward-wall experiment")
    sensor_position = body_position + body_rotation @ np.asarray(
        sensor_in_body.ExtractTranslation()
    )
    forward = flu_rotation[:, 0].copy()
    forward[2] = 0
    forward /= np.linalg.norm(forward)
    center = sensor_position + 3.1 * forward
    yaw_deg = math.degrees(math.atan2(forward[1], forward[0]))
    # UI translation is local to /World; check that it is an identity parent.
    world = stage.GetPrimAtPath("/World")
    if not world or not np.allclose(
        np.asarray(cache.GetLocalToWorldTransform(world)), np.eye(4), atol=1e-6
    ):
        raise RuntimeError(
            "/World has a nonidentity transform; world coordinates cannot be pasted as local Translate"
        )
    result = {
        "source": source,
        "sensor_path": sensor_path,
        "sensor_world_xyz_m": sensor_position.tolist(),
        "obstacle_path": "/World/TestObstacle",
        "cube_size": 1.0,
        "translate_m": center.tolist(),
        "rotate_xyz_deg": [0, 0, yaw_deg],
        "scale": [0.2, 2.0, 2.0],
        "near_face_forward_distance_m": 3.0,
        "note": "Placement values only. Create a visible Cube manually; add collision but no rigid body. No scene edits or flight commands.",
    }
    output = PROJECT / "logs" / "obstacle_plan.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("[Obstacle plan] " + source)
    print("[Obstacle plan] Cube Size = 1; Scale XYZ = (0.2, 2, 2)")
    print("[Obstacle plan] Translate XYZ = " + str(tuple(round(v, 4) for v in center)))
    print("[Obstacle plan] Rotate XYZ degrees = " + str((0, 0, round(yaw_deg, 4))))
    print("[Obstacle plan] Near face = 3 m ahead of LiDAR. Report: " + str(output))


if __name__ in ("__main__", "<run_path>"):
    main()
