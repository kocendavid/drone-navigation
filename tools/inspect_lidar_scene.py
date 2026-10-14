"""Read-only live Isaac inspector, or saved root-layer inventory with --saved."""

import argparse
import json
import os
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]


def serial(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    try:
        return [serial(v) for v in value]
    except TypeError:
        return str(value)


def inspect_live():
    import omni.usd
    import omni.timeline
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No open scene")
    timeline = omni.timeline.get_timeline_interface()
    body_path = json.loads((PROJECT / "bridge_config.json").read_text())["body_prim_path"]
    body = stage.GetPrimAtPath(body_path)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    nodes, sensors, products, obstacles = [], [], [], []
    for prim in stage.TraverseAll():
        kind = prim.GetTypeName()
        path = str(prim.GetPath())
        attrs = {
            a.GetName(): serial(a.Get())
            for a in prim.GetAttributes()
            if a.GetName().startswith(
                ("inputs:", "node:", "omni:sensor:", "_replicator:", "graph:")
            )
        }
        connections = {
            a.GetName(): [str(p) for p in a.GetConnections()]
            for a in prim.GetAttributes()
            if a.GetConnections()
        }
        rels = {
            r.GetName(): [str(p) for p in r.GetTargets()]
            for r in prim.GetRelationships()
            if r.GetTargets()
        }
        entry = {
            "path": path,
            "type": kind,
            "active": prim.IsActive(),
            "attributes": attrs,
            "connections": connections,
            "relationships": rels,
        }
        if kind in ("OmniGraph", "OmniGraphNode"):
            # Include graph evaluation flags alongside node inputs.
            for key in ("evaluationMode", "pipelineStage", "evaluator:type"):
                attr = prim.GetAttribute(key)
                if attr:
                    entry["attributes"][key] = serial(attr.Get())
            nodes.append(entry)
        if "lidar" in kind.lower() or ("lidar" in path.lower() and kind == "Camera"):
            entry["descendant_of_body"] = (
                prim.GetPath().HasPrefix(body.GetPath()) if body else False
            )
            if prim.IsA(UsdGeom.Imageable):
                entry["computed_visibility"] = str(UsdGeom.Imageable(prim).ComputeVisibility())
            if prim.IsA(UsdGeom.Xformable):
                sensor_world = cache.GetLocalToWorldTransform(prim)
                entry["world_translation_stage_units"] = serial(sensor_world.ExtractTranslation())
                if body and body.IsA(UsdGeom.Xformable):
                    relative = sensor_world * cache.GetLocalToWorldTransform(body).GetInverse()
                    entry["sensor_to_body_matrix_usd_row_vectors"] = serial(relative)
                    entry["sensor_origin_in_body_stage_units"] = serial(
                        relative.ExtractTranslation()
                    )
                    entry["sensor_axes_in_body"] = [
                        serial(relative.TransformDir(axis))
                        for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
                    ]
            sensors.append(entry)
        if kind == "RenderProduct":
            products.append(entry)
        if prim.GetName() == "TestObstacle" and prim.IsA(UsdGeom.Xformable):
            entry["local_transform"] = serial(UsdGeom.Xformable(prim).GetLocalTransformation())
            entry["world_translation_stage_units"] = serial(
                cache.GetLocalToWorldTransform(prim).ExtractTranslation()
            )
            if prim.IsA(UsdGeom.Cube):
                entry["cube_size_stage_units"] = UsdGeom.Cube(prim).GetSizeAttr().Get()
            if prim.IsA(UsdGeom.Imageable):
                entry["computed_visibility"] = str(UsdGeom.Imageable(prim).ComputeVisibility())
            obstacles.append(entry)
    result = {
        "source": "live composed stage; no USD edits or timeline changes",
        "root_layer": stage.GetRootLayer().identifier,
        "root_dirty": stage.GetRootLayer().dirty,
        "session_dirty": stage.GetSessionLayer().dirty,
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "timeline_playing": timeline.is_playing(),
        "timeline_time_s": timeline.get_current_time(),
        "rmw_environment": os.environ.get("RMW_IMPLEMENTATION"),
        "domain_environment": os.environ.get("ROS_DOMAIN_ID"),
        "body_prim": body_path,
        "body_exists": bool(body),
        "sensors": sensors,
        "graphs_and_nodes": nodes,
        "render_products": products,
        "test_obstacles": obstacles,
        "note": "USD default transforms only; live PhysX poses may differ while playing. No sensor extrinsic is published.",
    }
    return result


def inspect_saved(path):
    # Isaac standalone Python can use the already installed private USD wheel.
    sys.path.insert(0, str(PROJECT / ".tools"))
    from pxr import Sdf

    layer = Sdf.Layer.FindOrOpen(str(path))
    if layer is None:
        raise RuntimeError(f"Cannot read {path}")
    entries = []

    def visit(prim):
        attrs = {
            str(a.name): {
                "default": serial(a.default),
                "connections": [str(p) for p in a.connectionPathList.GetAppliedItems()],
            }
            for a in prim.attributes.values()
        }
        rels = {
            str(r.name): [str(p) for p in r.targetPathList.GetAppliedItems()]
            for r in prim.relationships.values()
        }
        if prim.typeName in ("OmniGraph", "OmniGraphNode", "OmniLidar", "RenderProduct"):
            entries.append(
                {
                    "path": str(prim.path),
                    "type": prim.typeName,
                    "attributes": attrs,
                    "relationships": rels,
                }
            )
        for child in prim.nameChildren.values():
            visit(child)

    for root in layer.rootPrims:
        visit(root)
    return {
        "source": "saved root layer only; references, payloads and unsaved live changes excluded",
        "root_layer": layer.identifier,
        "entries": entries,
    }


def main():
    # runpy in Script Editor should not parse Isaac launch arguments.
    saved = "--saved" in sys.argv
    if saved:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--saved", type=Path, required=True)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        result, output = inspect_saved(args.saved), args.output
    else:
        result, output = inspect_live(), PROJECT / "logs" / "lidar_scene_live.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"[LiDAR inspection] Report: {output}")
    print("[LiDAR inspection] " + result["source"])
    if not saved:
        print(
            f"[LiDAR inspection] Playing={result['timeline_playing']}; sensors="
            f"{[s['path'] for s in result['sensors']]}; RMW={result['rmw_environment']}"
        )


if __name__ == "__main__" or __name__ == "<run_path>":
    main()
