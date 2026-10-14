"""Read-only OmniGraph runtime snapshots, one second apart, inside Isaac."""

import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path

import carb.settings
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd

PROJECT = Path(__file__).resolve().parents[1]
GRAPH_PATHS = ("/Graph/ROS_Clock", "/Graph/ROS_LidarRTX")
PIPELINE_ATTRIBUTES = (
    "inputs:exec",
    "inputs:execIn",
    "inputs:step",
    "inputs:enabled",
    "inputs:dataPtr",
    "inputs:bufferSize",
    "inputs:cudaDeviceIndex",
    "inputs:width",
    "inputs:height",
    "inputs:timeStamp",
    "inputs:topicName",
    "inputs:frameId",
    "inputs:context",
    "inputs:renderProductPath",
    "outputs:exec",
    "outputs:execOut",
    "outputs:dataPtr",
    "outputs:bufferSize",
    "outputs:cudaDeviceIndex",
    "outputs:width",
    "outputs:height",
    "outputs:simulationTime",
)


def serial(value):
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    try:
        return [serial(v) for v in value]
    except TypeError:
        return str(value)


def snapshot():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No open USD stage")
    manager = omni.kit.app.get_app().get_extension_manager()
    timeline = omni.timeline.get_timeline_interface()
    settings = carb.settings.get_settings()
    result = {
        "utc": datetime.now(timezone.utc).isoformat(),
        "timeline_playing": timeline.is_playing(),
        "timeline_time_s": timeline.get_current_time(),
        "root_layer": stage.GetRootLayer().identifier,
        "extensions": {
            name: manager.is_extension_enabled(name)
            for name in (
                "isaacsim.ros2.bridge",
                "isaacsim.ros2.nodes",
                "isaacsim.ros2.core",
                "isaacsim.core.nodes",
                "isaacsim.sensors.rtx",
                "omni.graph.action",
            )
        },
        "srtx_enabled": settings.get("/exts/omni.replicator.srtx/enabled"),
        "settings": {
            name: settings.get(name)
            for name in (
                "/app/sensors/nv/lidar/outputBufferOnGPU",
                "/renderer/raytracingMotion/enabled",
                "/rtx/raytracingMotion/enabled",
            )
        },
        "graphs": {},
        "nodes": [],
        "render_products": [],
        "render_pipeline_nodes": [],
    }
    # Include a recreated helper graph even if the operator chose a new name.
    graph_paths = set(GRAPH_PATHS)
    for prim in stage.TraverseAll():
        if prim.GetTypeName() == "OmniGraphNode":
            node_type = str(prim.GetAttribute("node:type").Get())
            if node_type.endswith(("ROS2RtxLidarHelper", "ROS2PublishClock")):
                graph_paths.add(str(prim.GetParent().GetPath()))
    for graph_path in sorted(graph_paths):
        try:
            graph = og.get_graph_by_path(graph_path)
            valid = graph is not None and graph.is_valid()
            result["graphs"][graph_path] = {"valid": valid}
            if valid:
                result["graphs"][graph_path].update(
                    pipeline_stage=str(graph.get_pipeline_stage()), disabled=graph.is_disabled()
                )
        except Exception as exc:
            result["graphs"][graph_path] = {"inspection_error": str(exc)}
        root = stage.GetPrimAtPath(graph_path)
        if not root:
            continue
        for prim in root.GetChildren():
            if prim.GetTypeName() != "OmniGraphNode":
                continue
            path = str(prim.GetPath())
            authored_type = prim.GetAttribute("node:type").Get()
            item = {
                "path": path,
                "authored_type": str(authored_type),
                "attributes": {},
                "errors": {},
            }
            try:
                node = og.Controller.node(path)
                item["node_valid"] = node.is_valid()
                item["registered_type_valid"] = node.get_node_type().is_valid()
                item["compute_count"] = node.get_compute_count()
                item["compute_errors"] = node.get_compute_messages(og.Severity.ERROR)
                item["compute_warnings"] = node.get_compute_messages(og.Severity.WARNING)
                runtime_attributes = {attr.get_name(): attr for attr in node.get_attributes()}
                # Scalar settings and small target/path outputs only, never sensor buffers.
                names = [
                    a.GetName()
                    for a in prim.GetAttributes()
                    if a.GetName().startswith(("inputs:", "outputs:"))
                ]
                names += [
                    r.GetName()
                    for r in prim.GetRelationships()
                    if r.GetName().startswith("inputs:")
                ]
                for name in sorted(set(names)):
                    try:
                        attr = runtime_attributes.get(name)
                        if attr is not None and attr.is_valid():
                            item["attributes"][name] = serial(og.Controller.get(attr))
                        else:
                            item["errors"][name] = "Runtime attribute is invalid"
                    except Exception as exc:
                        item["errors"][name] = str(exc)
            except Exception as exc:
                item["inspection_error"] = str(exc)
            result["nodes"].append(item)
    for prim in stage.TraverseAll():
        if prim.GetTypeName() == "OmniGraphNode" and str(prim.GetPath()).startswith("/Render/"):
            path = str(prim.GetPath())
            item = {
                "path": path,
                "authored_type": str(prim.GetAttribute("node:type").Get()),
                "attributes": {},
                "errors": {},
            }
            try:
                node = og.Controller.node(path)
                item["node_valid"] = node.is_valid()
                item["compute_count"] = node.get_compute_count()
                item["compute_errors"] = node.get_compute_messages(og.Severity.ERROR)
                item["compute_warnings"] = node.get_compute_messages(og.Severity.WARNING)
                runtime_attributes = {attr.get_name(): attr for attr in node.get_attributes()}
                # Read scalar pointers and sizes only; never dereference or copy sensor buffers.
                for name in PIPELINE_ATTRIBUTES:
                    try:
                        attr = runtime_attributes.get(name)
                        if attr is not None and attr.is_valid():
                            item["attributes"][name] = serial(og.Controller.get(attr))
                    except Exception as exc:
                        item["errors"][name] = str(exc)
            except Exception as exc:
                item["inspection_error"] = str(exc)
            result["render_pipeline_nodes"].append(item)
        if prim.GetTypeName() == "RenderProduct":
            result["render_products"].append(
                {
                    "path": str(prim.GetPath()),
                    "camera": [str(p) for p in prim.GetRelationship("camera").GetTargets()],
                    "ordered_vars": [
                        str(p) for p in prim.GetRelationship("orderedVars").GetTargets()
                    ],
                }
            )
    return result


async def inspect():
    print("[LiDAR runtime] Reading two snapshots; scene and timeline unchanged.")
    first = snapshot()
    await asyncio.sleep(1.0)
    second = snapshot()
    prior = {node["path"]: node for node in first["nodes"] + first["render_pipeline_nodes"]}
    for node in second["nodes"] + second["render_pipeline_nodes"]:
        previous = prior.get(node["path"], {})
        if "compute_count" in node and "compute_count" in previous:
            node["compute_count_delta"] = node["compute_count"] - previous["compute_count"]
    report = {
        "source": "Read-only OmniGraph runtime; no evaluations forced or state changes",
        "snapshots": [first, second],
    }
    output = PROJECT / "logs" / "lidar_runtime.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"[LiDAR runtime] Report: {output}")
    for node in second["nodes"] + second["render_pipeline_nodes"]:
        print(
            f"[LiDAR runtime] {node['path']}: computes={node.get('compute_count')}, "
            f"delta={node.get('compute_count_delta')}, errors={node.get('compute_errors', [])}, "
            f"inspection_error={node.get('inspection_error')}"
        )


task = asyncio.ensure_future(inspect())
