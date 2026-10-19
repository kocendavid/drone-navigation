"""Trajectory time comes from Isaac, never from the count of delivered packets."""

import math
import threading


class TrajectoryClock:
    def __init__(self):
        self.lock = threading.RLock()
        self.stamp = None
        self.total_s = 0.0
        self.mark = 0.0
        self.received = None
        self.confirming = 3
        self.last_delta_s = None
        self.discontinuity = None
        self.consume_reason = "missing_clock"
        self.last_decision = None

    def observe(self, stamp_ns, now):
        with self.lock:
            self._observe(stamp_ns, now)

    def invalidate(self):
        with self.lock:
            self.received = None
            self.confirming = 3
            self.mark = self.total_s

    def _observe(self, stamp_ns, now):
        stamp = int(stamp_ns)
        if stamp < 0:
            self.received = None
            return
        previous = self.stamp
        self.stamp = stamp
        if previous is None or stamp == previous:
            return
        delta = (stamp - previous) / 1e9
        self.last_delta_s = delta
        if not 0 < delta <= 0.25:
            self.discontinuity = (
                "clock_moved_backward" if delta < 0 else "clock_step_exceeded_0.25_s"
            )
            self.confirming = 3
            self.received = None
            self.mark = self.total_s
            return
        if self.confirming:
            self.confirming -= 1
            self.mark = self.total_s
        else:
            self.total_s += delta
        self.received = now

    def fresh(self, now):
        with self.lock:
            return (
                self.received is not None
                and 0 <= now - self.received <= 0.25
                and not self.confirming
            )

    def consume(self, now):
        with self.lock:
            elapsed = self.total_s - self.mark
            self.mark = self.total_s
            self.consume_reason = (
                "missing_clock"
                if self.received is None
                else (
                    "confirming_after_discontinuity"
                    if self.confirming
                    else (
                        "clock_update_stale"
                        if not self.fresh(now)
                        else (
                            "unconsumed_interval_exceeded_0.25_s"
                            if not math.isfinite(elapsed) or not 0 <= elapsed <= 0.25
                            else "healthy"
                        )
                    )
                )
            )
            self.last_decision = dict(
                consume_reason=self.consume_reason,
                decision_wall_s=now,
                source_stamp_ns=self.stamp,
                source_delta_s=self.last_delta_s,
                source_age_wall_s=now - self.received if self.received is not None else None,
                unconsumed_sim_s=elapsed,
                confirming=self.confirming,
            )
            if self.consume_reason != "healthy":
                return None
            return elapsed

    def report(self, now):
        with self.lock:
            return dict(
                available=self.fresh(now),
                consume_reason=self.consume_reason,
                source_stamp_ns=self.stamp,
                source_delta_s=self.last_delta_s,
                source_age_wall_s=now - self.received if self.received is not None else None,
                confirming=self.confirming,
                last_discontinuity=self.discontinuity,
                freshness_limit_wall_s=0.25,
                interval_limit_sim_s=0.25,
                decision=self.last_decision,
            )
