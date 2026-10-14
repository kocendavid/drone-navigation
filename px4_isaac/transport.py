"""Nonblocking pymavlink UDP transport; Isaac sends to the WSL simulator listener."""

import ipaddress
import math
import socket
import subprocess
import time
from pymavlink.dialects.v20 import common as mavlink


def wsl_address(distribution="Ubuntu"):
    result = subprocess.run(
        ["wsl.exe", "-d", distribution, "--", "hostname", "-I"],
        capture_output=True,
        text=True,
        check=True,
        timeout=8,
    )
    for value in result.stdout.split():
        address = ipaddress.ip_address(value)
        if address.version == 4 and not address.is_loopback:
            return str(address)
    raise RuntimeError("Cannot determine the WSL IPv4 address. Set px4_host in bridge_config.json.")


class SimulatorTransport:
    def __init__(self, host, port, system_id=1):
        self.peer = (host, int(port))
        self.system_id = int(system_id)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", 0))
        self.socket.setblocking(False)
        self.mav = mavlink.MAVLink(self, srcSystem=system_id, srcComponent=198)
        self.parser = mavlink.MAVLink(None)
        self.parser.robust_parsing = True
        self.controls = [0.0] * 16
        self.armed = False
        self.last_actuator_wall = None
        self.received_actuators = 0
        self.sent_sensors = 0

    def write(self, data):
        return self.socket.sendto(data, self.peer)

    def receive(self):
        for _ in range(128):
            try:
                data, address = self.socket.recvfrom(65535)
            except (BlockingIOError, ConnectionResetError):
                break
            if address != self.peer:
                continue
            for message in self.parser.parse_buffer(data) or []:
                if (
                    message.get_type() != "HIL_ACTUATOR_CONTROLS"
                    or message.get_srcSystem() != self.system_id
                ):
                    continue
                if not any(math.isfinite(float(v)) for v in message.controls[:4]):
                    continue
                self.controls = list(message.controls)
                self.armed = bool(message.mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.last_actuator_wall = time.monotonic()
                self.received_actuators += 1

    def fresh(self, timeout):
        return (
            self.last_actuator_wall is not None
            and time.monotonic() - self.last_actuator_wall < timeout
        )

    def send_sensor(self, timestamp_us, sample, send_gps=False):
        a, g, m = sample["accel_frd"], sample["gyro_frd"], sample["mag_frd"]
        self.mav.hil_sensor_send(
            timestamp_us,
            *a,
            *g,
            *m,
            sample["pressure_hpa"],
            0.0,
            sample["altitude_m"],
            15.0,
            0x1FFF,
            0,
        )
        self.sent_sensors += 1
        if send_gps:
            vn, ve, vd = sample["velocity_ned"]
            speed = math.hypot(vn, ve)
            course = (
                int(round(math.degrees(math.atan2(ve, vn)) % 360 * 100)) % 36000
                if speed > 0.05
                else 65535
            )
            self.mav.hil_gps_send(
                timestamp_us,
                3,
                int(round(sample["latitude_deg"] * 1e7)),
                int(round(sample["longitude_deg"] * 1e7)),
                int(round(sample["altitude_m"] * 1000)),
                50,
                80,
                min(65534, int(round(speed * 100))),
                *[max(-32768, min(32767, int(round(v * 100)))) for v in (vn, ve, vd)],
                course,
                16,
                0,
                0,
            )

    def close(self):
        self.socket.close()
