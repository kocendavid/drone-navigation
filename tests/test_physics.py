import copy
import json
import socket
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / ".vendor")]
import numpy as np
from pymavlink.dialects.v20 import common as mavlink
from px4_isaac.model import (
    GRAVITY,
    RotorModel,
    SensorModel,
    rotation_wxyz,
    resolve_gravity_direction,
)
from px4_isaac.transport import SimulatorTransport
from px4_isaac.mass import normalized_link_masses, configured_link_masses

CONFIG = json.loads((ROOT / "bridge_config.json").read_text())
CONFIG["sensor_noise_std"] = {}


class PhysicsTests(unittest.TestCase):
    def bridge_post_step(self, log):
        return SimpleNamespace(
            closed=False,
            view=object(),
            _body_state=lambda: (np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3)),
            time_us=1000000,
            sensor=SimpleNamespace(sample=lambda *args: {}),
            flu_in_body=np.eye(3),
            next_gps_us=0,
            transport=SimpleNamespace(
                send_sensor=Mock(),
                armed=False,
                fresh=lambda timeout: True,
                controls=[0.0] * 16,
                received_actuators=0,
                sent_sensors=0,
            ),
            config=CONFIG,
            last_truth_us=0,
            clock_pairs=SimpleNamespace(emit=Mock()),
            manager=SimpleNamespace(get_simulation_time=lambda: 1.0),
            diagnostics=SimpleNamespace(emit=Mock()),
            last_log_wall=0,
            model=SimpleNamespace(omega=np.zeros(4), thrust=np.zeros(4)),
            force=np.zeros(3),
            torque=np.zeros(3),
            log=log,
            _fail=Mock(),
        )

    def test_bridge_log_disk_and_console_errors_do_not_stop_sensor_publication(self):
        from px4_isaac.bridge import BridgeLog, IsaacPX4Bridge

        for target in ("px4_isaac.bridge.Path.open", "builtins.print"):
            with self.subTest(target=target), tempfile.TemporaryDirectory() as folder:
                log = BridgeLog(Path(folder) / "bridge.jsonl")
                node = self.bridge_post_step(log)
                try:
                    with patch(target, side_effect=OSError("diagnostic output failed")):
                        IsaacPX4Bridge._post_step(node, 0.004, None)
                        with log.condition:
                            self.assertTrue(log.condition.wait_for(lambda: log.error, 2))
                        self.assertIn("diagnostic output failed", log.error)
                        IsaacPX4Bridge._post_step(node, 0.004, None)
                        self.assertEqual(node.transport.send_sensor.call_count, 2)
                        node._fail.assert_not_called()
                        self.assertFalse(node.closed)
                finally:
                    log.close()

    def test_blocked_bridge_log_keeps_callbacks_and_pending_output_bounded(self):
        from px4_isaac.bridge import BridgeLog, IsaacPX4Bridge

        entered, release, finished = threading.Event(), threading.Event(), threading.Event()

        def blocked(logger, entry):
            entered.set()
            release.wait(3)

        with tempfile.TemporaryDirectory() as folder, patch.object(BridgeLog, "_write", blocked):
            log = BridgeLog(Path(folder) / "bridge.jsonl")
            node = self.bridge_post_step(log)
            try:
                IsaacPX4Bridge._post_step(node, 0.004, None)
                self.assertTrue(entered.wait(1))

                def callbacks():
                    for now in (10.0, 12.0, 14.0):
                        with patch("px4_isaac.bridge.time.monotonic", return_value=now):
                            node.last_log_wall = now - 2
                            IsaacPX4Bridge._post_step(node, 0.004, None)
                    finished.set()

                caller = threading.Thread(target=callbacks)
                caller.start()
                self.assertTrue(finished.wait(1), "Physics callback waited for diagnostic output")
                caller.join(1)
                self.assertGreaterEqual(log.superseded, 2)
                self.assertEqual(node.transport.send_sensor.call_count, 4)
                node._fail.assert_not_called()
            finally:
                release.set()
                log.close()

    def test_sensor_transport_failure_still_enters_bridge_failure_path(self):
        from px4_isaac.bridge import IsaacPX4Bridge

        node = self.bridge_post_step(SimpleNamespace())
        node.transport.send_sensor.side_effect = OSError("sensor transport failed")
        IsaacPX4Bridge._post_step(node, 0.004, None)
        node._fail.assert_called_once()
        self.assertEqual(str(node._fail.call_args.args[0]), "sensor transport failed")

    def test_disarmed_zero_wrench_is_not_applied_to_resting_articulation(self):
        from px4_isaac.bridge import IsaacPX4Bridge

        calls = []
        transport = SimpleNamespace(
            armed=False, fresh=lambda timeout: True, controls=[0.5] * 16, receive=lambda: None
        )
        model = RotorModel(copy.deepcopy(CONFIG), 1.5)
        node = SimpleNamespace(
            closed=False,
            view=SimpleNamespace(
                apply_forces_and_torques_at_position=lambda *args: calls.append(args)
            ),
            transport=transport,
            config=CONFIG,
            flu_in_body=np.eye(3),
            com_body=np.zeros(3),
            model=model,
            _body_state=lambda: (np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3)),
            _fail=lambda error: self.fail(str(error)),
        )
        IsaacPX4Bridge._pre_step(node, 0.004, None)
        self.assertEqual(calls, [])
        transport.armed = True
        IsaacPX4Bridge._pre_step(node, 0.004, None)
        self.assertEqual(len(calls), 1)

    def test_mass_rescaling_preserves_relative_link_distribution_and_checks_total(self):
        masses = normalized_link_masses(
            {"/World/quadcopter/chassis": 50000, "/World/quadcopter/rotor0": 100000}, 1.5
        )
        self.assertAlmostEqual(sum(masses.values()), 1.5)
        self.assertAlmostEqual(
            masses["/World/quadcopter/rotor0"] / masses["/World/quadcopter/chassis"], 2
        )
        config = dict(
            vehicle_root_prim_path="/World/quadcopter", vehicle_mass_kg=1.5, link_masses_kg=masses
        )
        self.assertEqual(configured_link_masses(config), masses)
        with self.assertRaises(ValueError):
            configured_link_masses(dict(config, vehicle_mass_kg=2))
        with self.assertRaises(ValueError):
            normalized_link_masses({"bad": 0}, 1.5)
        with self.assertRaises(ValueError):
            configured_link_masses(dict(config, link_masses_kg={"/World/not_the_drone": 1.5}))

    def test_usd_default_gravity_is_negative_stage_up_axis(self):
        np.testing.assert_array_equal(resolve_gravity_direction([0, 0, 0], "Z"), [0, 0, -1])
        np.testing.assert_array_equal(resolve_gravity_direction([0, 0, 0], "Y"), [0, -1, 0])
        np.testing.assert_array_equal(resolve_gravity_direction([0, 0, -10]), [0, 0, -1])
        np.testing.assert_array_equal(resolve_gravity_direction([0, 0, 1]), [0, 0, 1])
        with self.assertRaises(ValueError):
            resolve_gravity_direction([float("nan"), 0, 0])

    def test_hover_balances_weight_and_moments(self):
        model = RotorModel(copy.deepcopy(CONFIG), 1.5)
        force, torque = model.step([0.5] * 16, 1.0, True)
        np.testing.assert_allclose(force, [0, 0, 1.5 * GRAVITY], atol=1e-8)
        np.testing.assert_allclose(torque, 0, atol=1e-8)

    def test_each_motor_roll_pitch_and_yaw_direction(self):
        # FLU: FR rolls right, pitches nose up, turns clockwise.
        expected = [(-1, -1, -1), (1, 1, -1), (1, -1, 1), (-1, 1, 1)]
        for channel, signs in enumerate(expected):
            model = RotorModel(copy.deepcopy(CONFIG), 1.5)
            controls = [0.0] * 16
            controls[channel] = 0.7
            _, torque = model.step(controls, 1.0, True)
            np.testing.assert_array_equal(np.sign(torque), signs)

    def test_cut_and_invalid_motor_commands(self):
        model = RotorModel(copy.deepcopy(CONFIG), 1.5)
        model.step([1.0] * 16, 0.1, True)
        f, t = model.step([1.0] * 16, 0.004, False)
        np.testing.assert_array_equal(f, 0)
        np.testing.assert_array_equal(t, 0)
        model.step([float("nan"), -1.0, 2.0, 0.0] + [0.0] * 12, 1.0, True)
        self.assertEqual(model.omega[0], 0.0)
        self.assertEqual(model.omega[1], 0.0)
        self.assertLessEqual(model.omega[2], CONFIG["omega_max_rad_s"])

    def test_proper_acceleration_stationary_and_freefall(self):
        sensor = SensorModel(CONFIG)
        sample = sensor.sample([0, 0, 0], np.eye(3), [0, 0, 0], [0, 0, 0], 0.004)
        np.testing.assert_allclose(sample["accel_frd"], [0, 0, -GRAVITY])
        sample = sensor.sample([0, 0, 0], np.eye(3), [0, 0, -GRAVITY * 0.004], [0, 0, 0], 0.004)
        np.testing.assert_allclose(sample["accel_frd"], 0, atol=1e-9)

    def test_navigation_and_body_axes(self):
        sensor = SensorModel(CONFIG)
        sensor.sample([10, 20, 0], np.eye(3), [0, 0, 0], [0, 0, 0], 0.004)
        sample = sensor.sample([12, 23, 5], np.eye(3), [2, 3, 4], [1, 2, 3], 0.004)
        np.testing.assert_allclose(sample["position_ned"], [3, 2, -5])
        np.testing.assert_allclose(sample["velocity_ned"], [3, 2, -4])
        np.testing.assert_allclose(sample["gyro_frd"], [1, -2, -3])
        self.assertAlmostEqual(sample["altitude_m"], CONFIG["altitude_m"] + 5)
        self.assertGreater(sample["latitude_deg"], CONFIG["latitude_deg"])
        self.assertGreater(sample["longitude_deg"], CONFIG["longitude_deg"])
        r = rotation_wxyz([np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-8)

    def test_stationary_gyro_survives_px4_fifo_quantization(self):
        noisy_config = json.loads((ROOT / "bridge_config.json").read_text())
        sensor = SensorModel(noisy_config)
        # PX4 simulator gyro 0 is a 2000 deg/s, signed 16-bit FIFO.
        fifo_scale = np.radians(2000.0) / 32768.0
        samples = [
            sensor.sample([0, 0, 0], np.eye(3), [0, 0, 0], [0, 0, 0], 0.004)["gyro_frd"]
            for _ in range(250)
        ]
        quantized = np.trunc(np.array(samples) / fifo_scale)
        for axis in range(3):
            self.assertGreater(len(np.unique(quantized[:, axis])), 3)


class TransportTests(unittest.TestCase):
    def test_real_mavlink_datagrams_and_timeout(self):
        peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer.bind(("127.0.0.1", 0))
        peer.settimeout(1.0)
        transport = SimulatorTransport(*peer.getsockname())
        try:
            sensor = SensorModel(CONFIG)
            sample = sensor.sample([0, 0, 0], np.eye(3), [1, 2, 3], [0, 0, 0], 0.004)
            transport.send_sensor(1000000, sample, True)
            parser = mavlink.MAVLink(None)
            imu_data, destination = peer.recvfrom(65535)
            gps_data, _ = peer.recvfrom(65535)
            imu = parser.parse_buffer(imu_data)[0]
            gps = parser.parse_buffer(gps_data)[0]
            self.assertEqual(imu.get_type(), "HIL_SENSOR")
            self.assertAlmostEqual(imu.zacc, -GRAVITY, places=5)
            self.assertEqual(imu.fields_updated, 0x1FFF)
            self.assertEqual(gps.get_type(), "HIL_GPS")
            self.assertEqual((gps.vn, gps.ve, gps.vd), (200, 100, -300))
            encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
            message = mavlink.MAVLink_hil_actuator_controls_message(
                1000000, [0.6] * 4 + [0.0] * 12, 128, 0
            )
            peer.sendto(message.pack(encoder), destination)
            time.sleep(0.01)
            transport.receive()
            self.assertTrue(transport.armed)
            self.assertTrue(transport.fresh(0.5))
            self.assertEqual(transport.received_actuators, 1)
            transport.last_actuator_wall = time.monotonic() - 1.0
            self.assertFalse(transport.fresh(0.5))
            message.mode = 0
            peer.sendto(message.pack(encoder), destination)
            time.sleep(0.01)
            transport.receive()
            self.assertFalse(transport.armed)
        finally:
            transport.close()
            peer.close()


class LoaderTests(unittest.TestCase):
    def test_fresh_source_bypasses_cached_loader_code(self):
        from bridge_loader import load_bridge_modules
        import px4_isaac.bridge as cached_module

        stale_class = object()
        cached_module.IsaacPX4Bridge = stale_class
        # Kit's cached code path must not be used for project modules.
        with patch(
            "importlib.machinery.SourceFileLoader.get_code",
            side_effect=AssertionError("cached loader used"),
        ):
            bridge_module = load_bridge_modules(ROOT)
        self.assertEqual(Path(bridge_module.__file__), ROOT / "px4_isaac/bridge.py")
        np.testing.assert_array_equal(
            bridge_module.resolve_gravity_direction([0, 0, 0]), [0, 0, -1]
        )
        self.assertIsNot(bridge_module.IsaacPX4Bridge, stale_class)


if __name__ == "__main__":
    unittest.main(verbosity=2)
