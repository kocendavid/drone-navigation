"""Isaac physics callbacks: MAVLink motors -> rotor wrench -> simulated sensors."""

import copy
import json
import runpy
import time
from pathlib import Path
import numpy as np
from .model import RotorModel, SensorModel, rotation_wxyz, resolve_gravity_direction
from .transport import SimulatorTransport, wsl_address
from .mass import configured_link_masses


class IsaacPX4Bridge:
    def __init__(self, config_path):
        import omni.usd
        import omni.timeline
        from pxr import Usd, UsdGeom, UsdPhysics
        from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent

        self.config_path = Path(config_path)
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("Open the drone scene before loading the bridge.")
        if omni.timeline.get_timeline_interface().is_playing():
            raise RuntimeError("Stop Isaac first, load the bridge, then press Play.")
        if (
            abs(UsdGeom.GetStageMetersPerUnit(self.stage) - 1.0) > 1e-6
            or UsdGeom.GetStageUpAxis(self.stage) != "Z"
        ):
            raise RuntimeError("The bridge requires a scene in metres with Z up.")
        if abs(UsdPhysics.GetStageKilogramsPerUnit(self.stage) - 1.0) > 1e-6:
            raise RuntimeError("The bridge requires masses in kilograms.")
        body = self.stage.GetPrimAtPath(self.config["body_prim_path"])
        if not body or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(
                "body_prim_path must name the dynamic chassis rigid body, not the drone root Xform."
            )
        if UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get():
            raise RuntimeError("Disable Kinematic Enabled on the chassis.")
        if self.config.get("rigid_frame", False):
            joints = [
                p
                for p in Usd.PrimRange(
                    self.stage.GetPrimAtPath(self.config["vehicle_root_prim_path"])
                )
                if p.IsA(UsdPhysics.Joint)
            ]
            if len(joints) != 8:
                raise RuntimeError("Rigid frame expects the quadcopter's eight arm/rotor joints")
            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                for joint in joints:
                    joint.SetTypeName("PhysicsFixedJoint")
            print(
                "[PX4 bridge] Rigid frame: eight fixed joints; rotor spin remains in the motor model",
                flush=True,
            )
        overrides = configured_link_masses(self.config)
        if overrides:
            links = {
                str(p.GetPath()): p
                for p in Usd.PrimRange(
                    self.stage.GetPrimAtPath(self.config["vehicle_root_prim_path"])
                )
                if p.HasAPI(UsdPhysics.RigidBodyAPI)
            }
            if set(links) != set(overrides):
                raise RuntimeError("Mass configuration must cover every vehicle rigid link exactly")
            # Session overrides do not overwrite the user's scene or source asset.
            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                for path, value in overrides.items():
                    api = UsdPhysics.MassAPI.Apply(links[path])
                    api.CreateMassAttr(float(value))
                    api.CreateDiagonalInertiaAttr((0.0, 0.0, 0.0))
            print(
                f"[PX4 bridge] Applied session link masses: {sum(overrides.values()):.6f} kg total",
                flush=True,
            )
        scene = self.stage.GetPrimAtPath(self.config["physics_scene_path"])
        if not scene or not scene.IsA(UsdPhysics.Scene):
            raise RuntimeError("Set physics_scene_path to the scene PhysicsScene prim.")
        gravity = UsdPhysics.Scene(scene)
        raw_direction = gravity.GetGravityDirectionAttr().Get()
        direction = resolve_gravity_direction(
            raw_direction, str(UsdGeom.GetStageUpAxis(self.stage))
        )
        print(
            f"[PX4 bridge] Gravity direction: USD={tuple(raw_direction)}, effective={direction.tolist()}",
            flush=True,
        )
        if direction[2] >= 0 or np.linalg.norm(direction[:2]) > 1e-6:
            raise RuntimeError(
                f"Physics gravity must point along world -Z; current direction is {tuple(raw_direction)}."
            )

        self.flu_in_body = rotation_wxyz(self.config["body_flu_quaternion_wxyz"])
        # Read rotor centres from the composed asset in the same body-local frame.
        inverse_body = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0).GetInverse()
        for rotor in self.config["rotors"]:
            prim = (
                self.stage.GetPrimAtPath(rotor.get("prim_path", ""))
                if rotor.get("prim_path")
                else None
            )
            if prim:
                point = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
                rotor["position_flu_m"] = (
                    self.flu_in_body.T @ np.array(inverse_body.Transform(point))
                ).tolist()

        host = self.config["px4_host"]
        if host == "auto":
            host = wsl_address(self.config["wsl_distribution"])
        self.transport = SimulatorTransport(
            host, self.config["px4_simulator_port"], self.config["px4_system_id"]
        )
        self.sensor = SensorModel(self.config)
        self.model = None
        self.view = None
        self.sim_view = None
        self.closed = False
        self.callbacks = []
        self.manager = SimulationManager
        self.time_us = 1000000
        self.next_gps_us = 0
        self.last_log_wall = 0.0
        # Kit can retain old imported Python modules across bridge reloads.
        # Read this observational adapter directly, like the bridge bootstrap.
        diagnostic_source = runpy.run_path(
            str(self.config_path.parent / "navigation" / "diagnostic_wire.py")
        )
        self.diagnostics = diagnostic_source["DiagnosticEmitter"]("isaac_bridge")
        self.last_truth_us = 0
        self.last_position = None
        self.force = np.zeros(3)
        self.torque = np.zeros(3)
        self.enabled = False
        self.log_path = self.config_path.parent / "logs" / "isaac_bridge.jsonl"
        self.log_path.parent.mkdir(exist_ok=True)
        self.log = self.log_path.open("a", encoding="utf-8", buffering=1)
        # This NumPy adapter uses CPU PhysX with readback; RTX sensors can remain enabled.
        SimulationManager.set_device("cpu")
        SimulationManager.set_physics_dt(
            1.0 / self.config["physics_hz"], self.config["physics_scene_path"]
        )
        self.callbacks.append(
            SimulationManager.register_callback(self._pre_step, SimulationEvent.PHYSICS_PRE_STEP)
        )
        self.callbacks.append(
            SimulationManager.register_callback(self._post_step, SimulationEvent.PHYSICS_POST_STEP)
        )
        self.callbacks.append(
            SimulationManager.register_callback(self._on_stop, SimulationEvent.SIMULATION_STOPPED)
        )
        print(
            f'[PX4 bridge] Loaded. UDP -> {host}:{self.config["px4_simulator_port"]}. Press Play.',
            flush=True,
        )

    def _initialize_view(self):
        import omni.physics.tensors as tensors
        from pxr import Usd, UsdPhysics
        import isaacsim.core.experimental.utils.stage as stage_utils

        stage_id = stage_utils.get_stage_id(stage_utils.get_current_stage(backend="usd"))
        self.sim_view = tensors.create_simulation_view("numpy", stage_id=stage_id)
        self.sim_view.set_subspace_roots("/")
        self.view = self.sim_view.create_rigid_body_view(self.config["body_prim_path"])
        if self.view.count != 1:
            raise RuntimeError("Expected one chassis rigid body in the physics tensor view.")
        self.com_body = self.view.get_coms()[0, :3].astype(float).copy()
        root = self.stage.GetPrimAtPath(self.config["vehicle_root_prim_path"])
        paths = [str(p.GetPath()) for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        mass_view = self.sim_view.create_rigid_body_view(paths)
        mass = float(np.sum(mass_view.get_masses()))
        expected_mass = self.config.get("vehicle_mass_kg")
        if expected_mass is not None and not np.isclose(mass, expected_mass, rtol=0.01):
            raise RuntimeError(
                f"Configured mass {expected_mass} kg differs from physical mass {mass} kg"
            )
        self.model = RotorModel(copy.deepcopy(self.config), float(mass))
        self.mass = float(mass)
        print(
            f"[PX4 bridge] Vehicle mass={mass:.4f} kg, k_f={self.model.kf:.8g}, k_m={self.model.km:.8g}.",
            flush=True,
        )
        print(
            "[PX4 bridge] Motors: "
            + ", ".join(f'{r["channel"]}={r["name"]}' for r in self.config["rotors"]),
            flush=True,
        )

    def _body_state(self):
        transform = self.view.get_transforms()[0].astype(float)
        velocity = self.view.get_velocities()[0].astype(float)
        rotation = rotation_wxyz(transform[[6, 3, 4, 5]])
        return transform[:3], rotation, velocity[:3], velocity[3:]

    def _pre_step(self, dt, context):
        if self.closed:
            return
        try:
            if self.view is None:
                self._initialize_view()
            self.transport.receive()
            self.enabled = self.transport.armed and self.transport.fresh(
                self.config["actuator_timeout_s"]
            )
            position, rotation_body, _, _ = self._body_state()
            rotation_flu = rotation_body @ self.flu_in_body
            com_flu = self.flu_in_body.T @ self.com_body
            self.force, self.torque = self.model.step(
                self.transport.controls, dt, self.enabled, com_flu
            )
            if not np.any(self.force) and not np.any(self.torque):
                # Do not keep waking a resting articulation with zero-wrench
                # calls. Disarmed rotors are already zeroed by the model.
                return
            world_com = position + rotation_body @ self.com_body
            # Sum of the four point forces and their moments about COM: one PhysX call,
            # avoiding repeated actor indices. This never overwrites body velocity.
            self.view.apply_forces_and_torques_at_position(
                (rotation_flu @ self.force).astype(np.float32).reshape(1, 3),
                (rotation_flu @ self.torque).astype(np.float32).reshape(1, 3),
                world_com.astype(np.float32).reshape(1, 3),
                np.array([0], dtype=np.uint32),
                True,
            )
        except Exception as error:
            self._fail(error)

    def _post_step(self, dt, context):
        if self.closed or self.view is None:
            return
        try:
            position, rotation_body, velocity, angular_velocity = self._body_state()
            self.time_us += max(1, round(dt * 1e6))
            sample = self.sensor.sample(
                position, rotation_body @ self.flu_in_body, velocity, angular_velocity, dt
            )
            gps_due = self.time_us >= self.next_gps_us
            self.transport.send_sensor(self.time_us, sample, gps_due)
            if gps_due:
                self.next_gps_us = self.time_us + round(1e6 / self.config["gps_hz"])
            self.last_position = position.copy()
            if self.time_us - self.last_truth_us >= 40000:
                self.last_truth_us = self.time_us
                self.diagnostics.emit(
                    "isaac_truth",
                    px4_sim_us=self.time_us,
                    isaac_sim_ns=round(self.manager.get_simulation_time() * 1e9),
                    position_world_xy=position[:2].tolist(),
                    position_world_z=float(position[2]),
                    velocity_world=velocity.tolist(),
                    armed=self.transport.armed,
                    actuators_fresh=self.transport.fresh(self.config["actuator_timeout_s"]),
                )
            if time.monotonic() - self.last_log_wall >= 1.0:
                entry = {
                    "sim_time_us": self.time_us,
                    "armed": self.transport.armed,
                    "actuators_fresh": self.transport.fresh(self.config["actuator_timeout_s"]),
                    "motor_controls": self.transport.controls[:4],
                    "omega_rad_s": self.model.omega.tolist(),
                    "thrust_n": self.model.thrust.tolist(),
                    "force_flu_n": self.force.tolist(),
                    "torque_flu_nm": self.torque.tolist(),
                    "position_enu_m": position.tolist(),
                    "actuator_packets": self.transport.received_actuators,
                    "sensor_packets": self.transport.sent_sensors,
                }
                self.log.write(json.dumps(entry) + "\n")
                print(
                    f'[PX4 bridge] armed={entry["armed"]} fresh={entry["actuators_fresh"]} '
                    f'motors={np.round(entry["motor_controls"], 3).tolist()} '
                    f'thrust={sum(entry["thrust_n"]):.3f} N z={position[2]:.3f} m',
                    flush=True,
                )
                self.last_log_wall = time.monotonic()
        except Exception as error:
            self._fail(error)

    def _fail(self, error):
        import carb

        carb.log_error(f"[PX4 bridge] Stopped: {error}")
        self.close()

    def _on_stop(self, event):
        self.close()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.transport.armed = False
        if self.model is not None:
            self.model.omega.fill(0.0)
        for callback in self.callbacks:
            self.manager.deregister_callback(callback)
        self.callbacks.clear()
        self.transport.close()
        self.log.close()
        self.diagnostics.close()
        print(
            "[PX4 bridge] Closed. Restart PX4 and reload the bridge after an Isaac Stop/reset.",
            flush=True,
        )
