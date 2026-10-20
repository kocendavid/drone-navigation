"""Passive, source-time flight measurements sampled once per simulation second."""

from bisect import bisect_right
from collections import deque
import csv
import json
import math
from pathlib import Path


FIELDS = (
    "simulation_s",
    "timestamp_unix_s",
    "armed",
    "simulation_height_m",
    "estimated_height_m",
    "commanded_height_m",
    "px4_local_height_m",
    "actual_speed_m_s",
    "estimated_speed_m_s",
    "expected_speed_m_s",
    "selected_speed_m_s",
    "cruise_speed_m_s",
    "actual_vertical_speed_m_s",
    "estimated_vertical_speed_m_s",
    "phase",
    "speed_limit",
    "reason",
    "truth_available",
    "estimate_available",
    "reference_available",
    "setpoint_available",
    "px4_origin_world_z_m",
)


def bracket(rows, stamp, key, gap):
    index = bisect_right([r[key] for r in rows], stamp)
    if index and rows[index - 1][key] == stamp:
        return rows[index - 1], rows[index - 1], 0.0
    if index == 0 or index == len(rows):
        return None
    a, b = rows[index - 1], rows[index]
    distance = b[key] - a[key]
    if not 0 < distance <= gap or a.get("epoch") != b.get("epoch"):
        return None
    return a, b, (stamp - a[key]) / distance


def mix(pair, key):
    a, b, fraction = pair
    x, y = a.get(key), b.get(key)
    if x is None or y is None:
        return None
    return x + (y - x) * fraction


def held(rows, stamp, key, age):
    index = bisect_right([r[key] for r in rows], stamp) - 1
    if index >= 0 and 0 <= stamp - rows[index][key] <= age:
        return rows[index]
    return None


def number(value):
    return isinstance(value, (float, int)) and math.isfinite(value)


class FlightProfile:
    """Bounded buffers, no ROS connection, no interpolation across resets.

    Native PX4 samples use the HIL clock paired with Isaac time by the bridge.
    Arrival times never align motion samples. Wait two simulation seconds for
    independently arriving streams; unavailable values are empty CSV cells.
    """

    def __init__(self, folder, floor_z, wall_clock_offset_s=None):
        if not number(floor_z):
            raise ValueError("A finite mapping floor height is required")
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.floor_z = floor_z
        self.wall_clock_offset_s = wall_clock_offset_s
        self.session = None
        self.part = 0
        self.stream = None
        self.files = []
        self.truth = deque(maxlen=3000)
        self.native = deque(maxlen=3000)
        self.references = deque(maxlen=3000)
        self.setpoints = deque(maxlen=3000)
        self.origin = None
        self.native_epoch = None
        self.reference_session = None
        self.next_second = None
        self.latest = None
        self.samples = 0

    def _start(self, record):
        self.finish()
        if record["session"] == self.session:
            self.part += 1
        else:
            self.part = 0
        self.session = record["session"]
        self.truth.clear()
        self.native.clear()
        self.references.clear()
        self.setpoints.clear()
        self.origin = self.native_epoch = self.reference_session = None
        self.next_second = math.ceil(record["isaac_sim_ns"] / 1e9)
        # Session identifiers are diagnostic UUIDs, never arbitrary file paths.
        name = "".join(c for c in self.session if c.isalnum())[:64]
        self.path = self.folder / f"flight_{name}_{self.part}.csv"
        self.stream = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()
        self.files.append(self.path)
        metadata = dict(
            schema=2,
            session=self.session,
            floor_world_z_m=self.floor_z,
            interval_simulation_s=1,
            timestamp="UTC Unix seconds from the bridge emission clock; missing clock calibration stays unavailable",
            wall_clock_offset_s=self.wall_clock_offset_s,
            alignment="Native PX4/HIL source time paired with Isaac time",
            maximum_interpolation_gap_s=0.2,
            maximum_reference_age_s=0.25,
            height_datum="Height above configured mapping floor. PX4 origin calibrated only from disarmed, source-aligned truth and estimate; missing calibration stays unavailable.",
            predicted_height="PX4 estimated height, not a separate prediction model",
            predicted_speed="Expected horizontal reference speed from navigator; not a prediction of aircraft response",
            speed="Horizontal speed magnitude; vertical speeds recorded separately; samples are instantaneous, not one-second averages",
        )
        self.path.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

    @staticmethod
    def _append(rows, row, key):
        if rows and row[key] == rows[-1][key]:
            rows[-1] = row
        elif not rows or row[key] > rows[-1][key]:
            rows.append(row)

    def observe(self, record):
        kind = record.get("kind")
        if kind == "isaac_truth":
            stamp = record.get("isaac_sim_ns")
            if not number(stamp) or not number(record.get("px4_sim_us")):
                return
            if record["session"] != self.session or (
                self.truth and stamp / 1e9 < self.truth[-1]["sim"]
            ):
                self._start(record)
            velocity = record.get("velocity_world", [None] * 3)
            timestamp = record.get("emitted_unix_s")
            emitted = record.get("emitted_wall_s")
            if not number(timestamp) and number(emitted):
                received_unix, received_wall = record.get("received_unix_s"), record.get(
                    "received_wall_s"
                )
                if number(received_unix) and number(received_wall):
                    timestamp = received_unix + emitted - received_wall
                elif number(self.wall_clock_offset_s):
                    timestamp = emitted + self.wall_clock_offset_s
            self._append(
                self.truth,
                dict(
                    sim=stamp / 1e9,
                    native_us=record["px4_sim_us"],
                    timestamp_unix_s=timestamp if number(timestamp) else None,
                    z=record.get("position_world_z"),
                    vx=velocity[0],
                    vy=velocity[1],
                    vz=velocity[2],
                    armed=bool(record.get("armed")),
                ),
                "sim",
            )
            self._calibrate()
            self._drain(self.truth[-1]["sim"] - 2)
        elif not self.truth:
            return
        elif kind == "px4_position":
            fields = record["fields"]
            stamp = fields.get("timestamp_sample", fields.get("timestamp"))
            if not number(stamp):
                return
            # Reject old-process/old-clock records arriving after a bridge reset.
            if abs(stamp - self.truth[-1]["native_us"]) > 2_000_000:
                return
            epoch = (record.get("px4_pid"), fields.get("z_reset_counter"))
            if epoch != self.native_epoch:
                self.native.clear()
                self.setpoints.clear()
                self.origin = None
                self.native_epoch = epoch
            self._append(
                self.native,
                dict(
                    native_us=stamp,
                    epoch=(epoch, fields.get("vxy_reset_counter"), fields.get("vz_reset_counter")),
                    z=fields.get("z") if fields.get("z_valid") else None,
                    vx=fields.get("vx") if fields.get("v_xy_valid") else None,
                    vy=fields.get("vy") if fields.get("v_xy_valid") else None,
                    vz=fields.get("vz") if fields.get("v_z_valid") else None,
                ),
                "native_us",
            )
            self._calibrate()
        elif kind == "reference" and number(record.get("sim_ns")):
            if abs(record["sim_ns"] / 1e9 - self.truth[-1]["sim"]) > 2:
                return
            if self.reference_session != record["session"]:
                self.references.clear()
                self.reference_session = record["session"]
            self._append(self.references, dict(sim=record["sim_ns"] / 1e9, record=record), "sim")
        elif kind == "px4_setpoint":
            fields = record["fields"]
            stamp = fields.get("timestamp")
            if (
                number(stamp)
                and record.get("px4_pid") == (self.native_epoch or (None,))[0]
                and abs(stamp - self.truth[-1]["native_us"]) <= 2_000_000
            ):
                self._append(self.setpoints, dict(native_us=stamp, record=fields), "native_us")

    def _calibrate(self):
        if not self.native:
            return
        sample = self.native[-1]
        pair = bracket(self.truth, sample["native_us"], "native_us", 200_000)
        if pair and not pair[0]["armed"] and not pair[1]["armed"] and number(sample["z"]):
            z = mix(pair, "z")
            if number(z):
                self.origin = z + sample["z"]

    def _sample(self, second):
        row = {key: None for key in FIELDS}
        row.update(
            simulation_s=second,
            truth_available=False,
            estimate_available=False,
            reference_available=False,
            setpoint_available=False,
            px4_origin_world_z_m=self.origin,
        )
        pair = bracket(self.truth, second, "sim", 0.2)
        if pair:
            row["timestamp_unix_s"] = mix(pair, "timestamp_unix_s")
            row["truth_available"] = True
            row["armed"] = pair[0]["armed"]
            z = mix(pair, "z")
            row["simulation_height_m"] = z - self.floor_z if z is not None else None
            vx, vy = mix(pair, "vx"), mix(pair, "vy")
            row["actual_speed_m_s"] = (
                math.hypot(vx, vy) if vx is not None and vy is not None else None
            )
            row["actual_vertical_speed_m_s"] = mix(pair, "vz")
            native_stamp = mix(pair, "native_us")
            estimate = bracket(self.native, native_stamp, "native_us", 200_000)
            if estimate:
                z = mix(estimate, "z")
                row["px4_local_height_m"] = -z if z is not None else None
                if z is not None and self.origin is not None:
                    row["estimated_height_m"] = self.origin - z - self.floor_z
                vx, vy = mix(estimate, "vx"), mix(estimate, "vy")
                row["estimated_speed_m_s"] = (
                    math.hypot(vx, vy) if vx is not None and vy is not None else None
                )
                vz = mix(estimate, "vz")
                row["estimated_vertical_speed_m_s"] = -vz if vz is not None else None
                row["estimate_available"] = z is not None
            target = held(self.setpoints, native_stamp, "native_us", 250_000)
            if target:
                row["setpoint_available"] = True
                position = target["record"].get("position")
                if position and number(position[2]) and self.origin is not None:
                    row["commanded_height_m"] = self.origin - position[2] - self.floor_z
        reference = held(self.references, second, "sim", 0.25)
        if reference:
            row["reference_available"] = True
            ref = reference["record"]
            speed = ref.get("speed_decision", {})
            row.update(
                expected_speed_m_s=speed.get("expected_speed_m_s"),
                selected_speed_m_s=speed.get("selected_speed_m_s"),
                cruise_speed_m_s=speed.get("requested_cruise_m_s"),
                phase=ref.get("phase"),
                speed_limit=speed.get("dominant_limit"),
                reason=speed.get("reason"),
            )
        return row

    def _drain(self, until):
        while self.next_second is not None and self.next_second <= until:
            row = self._sample(self.next_second)
            self.writer.writerow(row)
            self.latest = row
            self.samples += 1
            self.next_second += 1
        if self.stream:
            self.stream.flush()

    def finish(self):
        if self.stream:
            self._drain(self.truth[-1]["sim"] if self.truth else -1)
            self.stream.close()
            self.stream = None
