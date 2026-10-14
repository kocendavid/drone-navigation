"""Private test scene; never opens or edits the user's saved drone scene."""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / ".vendor")]
parser = argparse.ArgumentParser()
parser.add_argument("--seconds", type=float, default=180.0)
args = parser.parse_args()
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "width": 64,
        "height": 64,
        "hide_ui": True,
        "multi_gpu": False,
        "disable_viewport_updates": True,
        "renderer": "MinimalRendering",
        "minimal_shading_mode": 4,
        "sync_loads": False,
    }
)
import omni.usd
import omni.timeline
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
from isaacsim.core.simulation_manager import SimulationManager
from px4_isaac.bridge import IsaacPX4Bridge

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, "Z")
UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
physics = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physics.CreateGravityMagnitudeAttr(9.80665)
PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim()).CreateEnableGPUDynamicsAttr(False)
PhysxSchema.PhysxSceneAPI(physics.GetPrim()).CreateBroadphaseTypeAttr("MBP")
ground = UsdGeom.Cube.Define(stage, "/World/ground")
ground.CreateSizeAttr(1.0)
ground.AddScaleOp().Set(Gf.Vec3d(100, 100, 0.1))
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
UsdGeom.Xform.Define(stage, "/World/test_drone")
body = UsdGeom.Cube.Define(stage, "/World/test_drone/chassis")
body.CreateSizeAttr(0.2)
body.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.15))
UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
UsdPhysics.CollisionAPI.Apply(body.GetPrim())
mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
mass.CreateMassAttr(1.5)
mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.03, 0.03, 0.05))
config = json.loads((ROOT / "bridge_config.json").read_text())
config["body_prim_path"] = "/World/test_drone/chassis"
config["vehicle_root_prim_path"] = "/World/test_drone"
config["vehicle_mass_kg"] = None
for rotor in config["rotors"]:
    rotor["prim_path"] = None
results_dir = ROOT / "results" / time.strftime("%Y%m%d-%H%M%S")
results_dir.mkdir(parents=True)
config_path = results_dir / "validation_config.json"
config_path.write_text(json.dumps(config, indent=2))
SimulationManager.set_device("cpu")
bridge = IsaacPX4Bridge(config_path)
timeline = omni.timeline.get_timeline_interface()
timeline.play()
app.update()
start = time.monotonic()
initial_z = None
max_z = -1e9
armed_samples = 0
max_thrust = 0.0
try:
    while app.is_running() and time.monotonic() - start < args.seconds:
        step_start = time.monotonic()
        SimulationManager.step()
        if bridge.closed:
            raise RuntimeError("Bridge callback failed; inspect the Isaac log.")
        if bridge.last_position is not None:
            z = float(bridge.last_position[2])
            initial_z = z if initial_z is None else initial_z
            max_z = max(max_z, z)
            armed_samples += int(bridge.enabled)
            max_thrust = max(max_thrust, float(sum(bridge.model.thrust)))
            if (
                armed_samples > 0
                and max_z - initial_z > 0.2
                and not bridge.enabled
                and z < initial_z + 0.05
            ):
                # The ROS proof has completed landing/disarm; avoid an idle six-minute run.
                break
        # Tick Kit without stepping timeline physics a second time.
        if int((time.monotonic() - start) * 250) % 250 == 0:
            app.update()
        time.sleep(max(0.0, 0.004 - (time.monotonic() - step_start)))
finally:
    result = {
        "initial_z_m": initial_z,
        "max_z_m": max_z,
        "armed_physics_steps": armed_samples,
        "max_thrust_n": max_thrust,
        "actuator_packets": bridge.transport.received_actuators,
        "body_reacted": armed_samples > 0 and initial_z is not None and max_z - initial_z > 0.2,
    }
    (results_dir / "physics_result.json").write_text(json.dumps(result, indent=2))
    print("PHYSICS RESULT " + json.dumps(result), flush=True)
    bridge.close()
    timeline.stop()
    # Kit 6.1 on this host faults during framework teardown in a second instance.
    # The test owns this entire process; close the bridge and flush its report first.
    sys.stdout.flush()
    sys.stderr.flush()
    import os

    os._exit(0 if result["body_reacted"] else 1)
