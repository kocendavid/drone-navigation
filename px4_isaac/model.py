"""Rotor dynamics and ENU/FLU to NED/FRD sensor conversion, independent of Kit."""

import math
import numpy as np

GRAVITY = 9.80665
ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def resolve_gravity_direction(direction, up_axis="Z"):
    """Resolve USD's zero-vector sentinel to the negative stage up axis."""
    vector = np.asarray(direction, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError("Gravity direction must be a finite three-component vector.")
    norm = np.linalg.norm(vector)
    if norm == 0.0:
        if up_axis not in ("Y", "Z"):
            raise ValueError("Unsupported stage up axis.")
        vector = np.array([0.0, -1.0, 0.0]) if up_axis == "Y" else np.array([0.0, 0.0, -1.0])
        return vector
    return vector / norm


def rotation_wxyz(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-9:
        raise ValueError("Expected a finite, nonzero wxyz quaternion.")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class RotorModel:
    def __init__(self, config, vehicle_mass_kg):
        self.rotors = config["rotors"]
        self.channels = np.array([r["channel"] for r in self.rotors], dtype=int)
        self.positions = np.array([r["position_flu_m"] for r in self.rotors], dtype=float)
        self.signs = np.array([r["yaw_torque_sign"] for r in self.rotors], dtype=float)
        if len(self.rotors) != 4 or sorted(self.channels.tolist()) != list(range(4)):
            raise ValueError("Four unique PX4 motor channels 0..3 are required.")
        if not np.isfinite(self.positions).all() or not np.isin(self.signs, [-1, 1]).all():
            raise ValueError("Invalid rotor positions or yaw signs.")
        if not math.isfinite(vehicle_mass_kg) or vehicle_mass_kg <= 0:
            raise ValueError("Vehicle mass must be positive.")
        self.omega_max = float(config["omega_max_rad_s"])
        hover = float(config["hover_command"])
        if self.omega_max <= 0 or not 0 < hover < 1:
            raise ValueError("Invalid maximum rotor speed or hover command.")
        # A starting estimate when measured propeller coefficients are unavailable.
        estimated_kf = vehicle_mass_kg * GRAVITY / (4 * (hover * self.omega_max) ** 2)
        self.kf = estimated_kf if config.get("k_f") is None else float(config["k_f"])
        self.km = (
            self.kf * float(config["yaw_moment_arm_m"])
            if config.get("k_m") is None
            else float(config["k_m"])
        )
        self.tau_up = float(config["motor_time_constant_up_s"])
        self.tau_down = float(config["motor_time_constant_down_s"])
        if self.kf <= 0 or self.km < 0 or min(self.tau_up, self.tau_down) <= 0:
            raise ValueError("Invalid rotor coefficients or time constants.")
        self.omega = np.zeros(4)
        self.thrust = np.zeros(4)

    def step(self, controls, dt, enabled, center_of_mass_flu_m=(0.0, 0.0, 0.0)):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Physics dt must be positive and finite.")
        if enabled:
            values = np.asarray(controls, dtype=float)[self.channels]
            values = np.clip(np.where(np.isfinite(values), values, 0.0), 0.0, 1.0)
            target = values * self.omega_max
            tau = np.where(target > self.omega, self.tau_up, self.tau_down)
            self.omega += (target - self.omega) * (1 - np.exp(-dt / tau))
        else:
            # Disarm and transport timeout cut all applied thrust immediately.
            self.omega.fill(0.0)
        self.thrust = self.kf * self.omega**2
        rotor_forces = np.column_stack((np.zeros(4), np.zeros(4), self.thrust))
        arms = self.positions - np.asarray(center_of_mass_flu_m)
        torque = np.cross(arms, rotor_forces).sum(axis=0)
        torque[2] += np.sum(self.signs * self.km * self.omega**2)
        return rotor_forces.sum(axis=0), torque


class SensorModel:
    def __init__(self, config):
        self.latitude = float(config["latitude_deg"])
        self.longitude = float(config["longitude_deg"])
        self.altitude = float(config["altitude_m"])
        self.origin = None
        self.previous_velocity = None
        self.mag_enu = np.asarray(config["magnetic_field_enu_gauss"], dtype=float)
        self.noise = config.get("sensor_noise_std", {})
        self.rng = np.random.default_rng(config.get("sensor_noise_seed", 42))

    def sample(self, position_enu, rotation_flu_to_enu, velocity_enu, angular_velocity_enu, dt):
        p, v = np.asarray(position_enu), np.asarray(velocity_enu)
        r = np.asarray(rotation_flu_to_enu)
        if self.origin is None:
            # Preserve the scene ground altitude; set the local horizontal origin at startup.
            self.origin = np.array([p[0], p[1], 0.0])
        acceleration = (
            np.zeros(3) if self.previous_velocity is None else (v - self.previous_velocity) / dt
        )
        self.previous_velocity = v.copy()
        gravity = np.array([0.0, 0.0, -GRAVITY])
        specific_force_frd = FLU_TO_FRD @ r.T @ (acceleration - gravity)
        gyro_frd = FLU_TO_FRD @ r.T @ np.asarray(angular_velocity_enu)
        mag_frd = FLU_TO_FRD @ r.T @ self.mag_enu
        ned = ENU_TO_NED @ (p - self.origin)
        velocity_ned = ENU_TO_NED @ v
        lat = self.latitude + math.degrees(ned[0] / 6378137.0)
        lon = self.longitude + math.degrees(
            ned[1] / (6378137.0 * math.cos(math.radians(self.latitude)))
        )
        altitude = self.altitude - ned[2]
        pressure = 1013.25 * max(0.01, 1 - altitude / 44330.0) ** 5.255
        # Real sensors vary while stationary. PX4's data validators treat long
        # runs of identical floating-point values as a stuck sensor.
        specific_force_frd += self.rng.normal(0.0, self.noise.get("accel_m_s2", 0.0), 3)
        gyro_frd += self.rng.normal(0.0, self.noise.get("gyro_rad_s", 0.0), 3)
        mag_frd += self.rng.normal(0.0, self.noise.get("mag_gauss", 0.0), 3)
        pressure += self.rng.normal(0.0, self.noise.get("pressure_hpa", 0.0))
        return {
            "accel_frd": specific_force_frd,
            "gyro_frd": gyro_frd,
            "mag_frd": mag_frd,
            "latitude_deg": lat,
            "longitude_deg": lon,
            "altitude_m": altitude,
            "pressure_hpa": pressure,
            "velocity_ned": velocity_ned,
            "position_ned": ned,
        }
