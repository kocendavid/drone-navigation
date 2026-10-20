"""Checked Isaac/PX4 clock conversion and bounded reference history."""

from collections import deque
import json
import math
import socket
import time
import uuid

import numpy as np

PORT = 18789


class ClockPairEmitter:
    """Send clock pairs only; physical ground truth never enters navigation."""

    def __init__(self, port=PORT):
        self.session = uuid.uuid4().hex
        self.peer = ("127.0.0.1", port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def emit(self, px4_us, sim_ns):
        try:
            self.socket.sendto(
                json.dumps(
                    dict(
                        session=self.session,
                        px4_us=px4_us,
                        sim_ns=sim_ns,
                        sent_qpc_s=time.perf_counter(),
                    ),
                    allow_nan=False,
                ).encode(),
                self.peer,
            )
        except (OSError, ValueError):
            pass

    def close(self):
        self.socket.close()


class SimulatorClock:
    def __init__(self):
        self.session = None
        self.pair = None
        self.confirmed = 0
        self.epoch = 0
        self.received = None
        self.last_sent = -math.inf

    def observe(self, row, now, qpc):
        session, native, sim, sent = (row[k] for k in ("session", "px4_us", "sim_ns", "sent_qpc_s"))
        if (
            not isinstance(session, str)
            or not session
            or type(native) is not int
            or type(sim) is not int
            or min(native, sim) < 0
            or not math.isfinite(sent)
            or not 0 <= qpc - sent <= 0.25
            or sent <= self.last_sent
        ):
            return
        self.last_sent = sent
        offset = native * 1000 - sim
        if self.pair is not None:
            old_native, old_sim = self.pair
            if session == self.session and (native, sim) == self.pair:
                return  # Repeated samples cannot refresh the mapping watchdog.
            delta = sim - old_sim
            if (
                session != self.session
                or not 0 < delta <= 250_000_000
                or abs(offset - (old_native * 1000 - old_sim)) > 2_000_000
            ):
                self.confirmed = 0
                self.epoch += 1
        self.session = session
        self.pair = (native, sim)
        self.received = now
        self.confirmed += 1

    def convert(self, native_us, now):
        if self.pair is None or self.confirmed < 3 or not 0 <= now - self.received <= 0.25:
            return None
        native, sim = self.pair
        delta = (native_us - native) * 1000
        if not -250_000_000 <= delta <= 100_000_000:
            return None
        return round(sim + delta)


class SampleTimeline:
    def __init__(self, port=PORT):
        self.clock = SimulatorClock()
        self.sync = deque(maxlen=12)
        self.socket = None
        if port is not None:
            candidate = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                candidate.bind(("127.0.0.1", port))
                candidate.setblocking(False)
                self.socket = candidate
            except OSError:
                candidate.close()

    def poll(self, now):
        if self.socket is None:
            return
        for _ in range(32):
            try:
                payload = self.socket.recv(1025)
            except BlockingIOError:
                break
            except OSError:
                return
            try:
                if len(payload) <= 1024:
                    self.clock.observe(json.loads(payload), now, time.perf_counter())
            except (ValueError, TypeError, KeyError, OverflowError):
                continue

    def observe_timesync(self, message, now):
        # PX4 serializes timestamp + (-estimated_offset) for XRCE. The signed
        # estimated_offset itself is not adjusted; recover native time by addition.
        if message.source_protocol != 2 or message.timestamp <= 0:
            return
        native = int(message.timestamp) + int(message.estimated_offset)
        row = (native, int(message.estimated_offset), now)
        if native <= 0 or not 0 <= message.round_trip_time <= 20_000:
            return
        if self.sync and native < self.sync[-1][0]:
            self.sync.clear()
        if not self.sync or row[:2] != self.sync[-1][:2]:
            self.sync.append(row)

    def sample_time(self, position, now):
        self.poll(now)
        stamp, sample = int(position.timestamp), int(position.timestamp_sample)
        if not 0 < sample <= stamp or stamp - sample > 250_000:
            return None, "invalid_sample_timestamp"
        candidates = []
        rows = list(self.sync)
        for index, (native, offset, received) in enumerate(rows):
            published = stamp + offset
            end = rows[index + 1][0] if index + 1 < len(rows) else math.inf
            if not 0 <= now - received <= 2.0 or not native <= published < end:
                continue
            converted = self.clock.convert(sample + offset, now)
            if converted is not None:
                candidates.append(converted)
        if not candidates:
            return None, "clock_mapping_unavailable"
        if max(candidates) - min(candidates) > 2_000_000:
            return None, "ambiguous_timesync_epoch"
        return candidates[-1], None

    def close(self):
        if self.socket is not None:
            self.socket.close()


class ReferenceHistory:
    def __init__(self):
        self.rows = deque(maxlen=128)
        self.epoch = None

    def clear(self):
        self.rows.clear()

    def record(self, stamp, xy, velocity, epoch):
        if stamp is None:
            return
        if epoch != self.epoch or self.rows and stamp < self.rows[-1][0]:
            self.clear()
        self.epoch = epoch
        row = (
            int(stamp),
            np.asarray(xy, dtype=float).copy(),
            np.asarray(velocity, dtype=float).copy(),
        )
        if self.rows and stamp == self.rows[-1][0]:
            self.rows[-1] = row
        else:
            self.rows.append(row)

    def at(self, stamp):
        if not self.rows or stamp < self.rows[0][0]:
            return None, "reference_history_unavailable"
        previous = self.rows[0]
        for row in self.rows:
            if stamp == row[0]:
                return row[1].copy(), "source_time_exact"
            if row[0] > stamp:
                gap = row[0] - previous[0]
                if gap > 250_000_000:
                    return None, "reference_history_gap"
                fraction = (stamp - previous[0]) / gap
                return previous[1] + fraction * (row[1] - previous[1]), "source_time_interpolated"
            previous = row
        # Telemetry can precede the next Isaac clock delivery. Bounded constant
        # velocity prediction uses source time, not receipt time or packet counts.
        ahead = (stamp - previous[0]) / 1e9
        if ahead <= 0.1:
            return previous[1] + previous[2] * ahead, "source_time_predicted"
        return None, "reference_history_too_old"
