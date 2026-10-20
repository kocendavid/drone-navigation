"""Isaac-only A/B yaw test using its existing render stream and a temporary sensor.

Stop the navigator while disarmed before running via isaac_client. The existing
render-product camera is restored in finally. No vehicle pose, PX4 command,
timeline, sensor mount, or scene-save operation is performed.
"""

import asyncio
import builtins
import json
import math
from pathlib import Path
import time
import traceback
import socket

import carb.settings
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom
from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
from px4_isaac.model import rotation_wxyz
from lidar_tools.scan_motion import SCAN_PERIOD_S

PROJECT = Path(__file__).resolve().parents[1]
OUTPUT = PROJECT / "results" / "lidar_compensation_validation.json"
ROOT = "/World/LidarCompensationValidation"
SOURCE = "/World/quadcopter/chassis/Example_Rotary"


def require_disarmed():
    bridge = getattr(builtins, "_px4_isaac_bridge", None)
    if bridge is None or bridge.closed or bridge.transport.armed:
        raise RuntimeError("Validation requires the connected, disarmed drone")


def residual(points, walls):
    distances = np.full(len(points), np.inf)
    for low, high in walls:
        distances = np.minimum(
            distances, np.linalg.norm(points - np.clip(points, low, high), axis=1)
        )
    return distances


def set_camera(camera, targets):
    # The live render product is authored in the stronger session layer.
    stage = omni.usd.get_context().get_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        camera.SetTargets(targets)


async def replay(mode, camera, annotator, walls):
    stage = omni.usd.get_context().get_stage()
    root = UsdGeom.Xform.Define(stage, ROOT)
    root.AddTranslateOp().Set(Gf.Vec3d(0, -18, -3.04))
    orient = root.AddOrientOp()
    orient.Set(Gf.Quatf(1, Gf.Vec3f(0)))
    prim = stage.DefinePrim(ROOT + "/" + mode.lower(), "OmniLidar")
    prim.GetReferences().AddInternalReference(SOURCE)
    for key, value in {
        "outputMotionCompensationState": mode,
        "outputFrameOfReference": "SENSOR",
        "instantLidar": mode == "COMPENSATED",
        "elementsCoordsType": "CARTESIAN",
    }.items():
        if not prim.GetAttribute("omni:sensor:Core:" + key).Set(value):
            raise RuntimeError("Cannot configure temporary sensor: " + key)
    set_camera(camera, [prim.GetPath()])
    timeline = omni.timeline.get_timeline_interface()
    deadline, begin, previous, warm = time.monotonic() + 30, None, None, set()
    rows = []
    try:
        while begin is None or timeline.get_current_time() - begin < 3.5:
            require_disarmed()
            if not timeline.is_playing() or time.monotonic() > deadline:
                raise RuntimeError(f"{mode}: replay exceeded 30 seconds; {len(rows)} scans")
            elapsed = 0 if begin is None else timeline.get_current_time() - begin
            angle = math.radians(60) * max(0, elapsed - 1)
            orient.Set(Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(0, 0, math.sin(angle / 2))))
            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            position = np.array(matrix.ExtractTranslation())
            await omni.kit.app.get_app().next_update_async()
            data = annotator.get_data()
            data = data.get("data") if isinstance(data, dict) else data
            if data is None:
                continue
            gmo = parse_generic_model_output_data(data)
            stamp = int(gmo.timestampNs)
            if not gmo.numElements or stamp == previous:
                continue
            previous = stamp
            # Ignore the last original-sensor frame still travelling through
            # the render pipeline after changing the render-product camera.
            if np.linalg.norm(np.array(gmo.frameEnd.posM) - position) > 0.2:
                continue
            if int(gmo.frameOfReference) != 0 or int(gmo.elementsCoordsType) != 0:
                continue
            if int(gmo.motionCompensationState) != (1 if mode == "COMPENSATED" else 0):
                continue
            if begin is None:
                warm.add(stamp)
                if len(warm) >= 3:
                    begin = timeline.get_current_time()
                continue
            xyz = np.column_stack([gmo.x, gmo.y, gmo.z])
            # Compare fixed, named reference conventions. This exposes an
            # incorrect cloud-header reference instead of fitting each scan.
            q = gmo.frameEnd.orientation
            frames = {
                "capture_end": (
                    rotation_wxyz([q[3], q[0], q[1], q[2]]),
                    np.array(gmo.frameEnd.posM),
                )
            }
            if mode == "COMPENSATED":
                if int(gmo.frameEnd.timestampNs) - stamp != round(SCAN_PERIOD_S * 1e9):
                    raise RuntimeError(
                        "SDK snapshot reference differs from configured timestamp alignment"
                    )
            errors = {}
            for name, (rotation, translation) in frames.items():
                world = xyz @ rotation.T + translation
                eligible = world[
                    np.isfinite(world).all(axis=1)
                    & (world[:, 2] > -3.28)
                    & (world[:, 2] < -2.58)
                    & (np.linalg.norm(xyz, axis=1) <= 12)
                    & (np.linalg.norm(xyz, axis=1) >= 0.5)
                ]
                if len(eligible) < 100:
                    raise RuntimeError("Insufficient wall returns")
                values = residual(eligible[:: max(1, len(eligible) // 10000)], walls)
                errors[name] = np.percentile(values, [50, 95, 99]).tolist()
            rows.append(
                dict(
                    elapsed_s=elapsed,
                    stamp_ns=stamp,
                    frame_start_ns=int(gmo.frameStart.timestampNs),
                    frame_end_ns=int(gmo.frameEnd.timestampNs),
                    errors_m=errors,
                )
            )
        stationary = [r for r in rows if 0.4 < r["elapsed_s"] < 0.9]
        moving = [r for r in rows if r["elapsed_s"] > 1.4]
        if len(stationary) < 2 or len(moving) < 10:
            raise RuntimeError("Insufficient independent stationary/yaw scans")
        return dict(
            rows=rows,
            stationary_p95_m={
                name: float(np.median([r["errors_m"][name][1] for r in stationary]))
                for name in errors
            },
            yaw_p95_m={
                name: float(np.median([r["errors_m"][name][1] for r in moving])) for name in errors
            },
        )
    finally:
        (PROJECT / "results" / ("lidar_validation_" + mode.lower() + ".json")).write_text(
            json.dumps(rows, indent=2)
        )


async def validate():
    report = dict(
        status="running",
        scope="Temporary sensor on existing render stream; navigator stopped, drone disarmed; 60 deg/s yaw",
        baseline=dict(instantLidar=False, compensation="NONCOMPENSATED", reference="capture_end"),
        candidate=dict(
            instantLidar=True,
            compensation="COMPENSATED",
            reference="capture_end",
            ros_stamp_offset_s=0,
        ),
    )
    OUTPUT.write_text(json.dumps(report, indent=2))
    camera = original = None
    try:
        require_disarmed()
        try:
            socket.create_connection(("127.0.0.1", 8787), timeout=0.5).close()
        except OSError:
            pass
        else:
            raise RuntimeError("Stop navigator while disarmed before redirecting the render stream")
        if not carb.settings.get_settings().get("/renderer/raytracingMotion/enabled"):
            raise RuntimeError("Motion BVH is not enabled")
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(ROOT).IsValid():
            raise RuntimeError("An earlier validation sensor still exists")
        products = [
            p
            for p in stage.TraverseAll()
            if p.GetTypeName() == "RenderProduct"
            and SOURCE in list(map(str, p.GetRelationship("camera").GetTargets()))
        ]
        if len(products) != 1:
            raise RuntimeError("Expected one original lidar render product")
        camera = products[0].GetRelationship("camera")
        original = camera.GetTargets()
        annotator = rep.AnnotatorRegistry.get_annotator("GenericModelOutput", device="cpu")
        annotator.attach([str(products[0].GetPath())])
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        walls = []
        for prim in stage.TraverseAll():
            if "/maze/Wall" in str(prim.GetPath()) and prim.IsA(UsdGeom.Boundable):
                box = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
                walls.append((np.array(box.GetMin()), np.array(box.GetMax())))
        for mode in ("NONCOMPENSATED", "COMPENSATED"):
            report[mode] = await replay(mode, camera, annotator, walls)
            set_camera(camera, original)
            await omni.kit.app.get_app().next_update_async()
            stage.RemovePrim(ROOT)
            OUTPUT.write_text(json.dumps(report, indent=2))
        raw, corrected = report["NONCOMPENSATED"], report["COMPENSATED"]
        report["passed"] = (
            corrected["stationary_p95_m"]["capture_end"] < 0.08
            and corrected["yaw_p95_m"]["capture_end"] < 0.08
            and corrected["yaw_p95_m"]["capture_end"] < raw["yaw_p95_m"]["capture_end"] / 2
        )
        report["status"] = "passed" if report["passed"] else "failed"
    except Exception:
        report.update(status="failed", passed=False, error=traceback.format_exc())
    finally:
        if camera is not None and original is not None:
            set_camera(camera, original)
            omni.usd.get_context().get_stage().RemovePrim(ROOT)
        OUTPUT.write_text(json.dumps(report, indent=2))
        print("[LiDAR compensation validation] " + report["status"])


task = asyncio.ensure_future(validate())
