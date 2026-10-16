"""Explicit simulator flight buttons; bounded native commands, observed completion."""

import math
import re
import threading
import time

from .reset import ResetBackend, PX4


class NativeFlightBackend(ResetBackend):
    def __init__(self):
        self.operation_lock = threading.RLock()

    def verify_simulator(self):
        processes = self.command(
            ["wsl.exe", "-d", "Ubuntu", "--exec", "pgrep", "-a", "-x", "px4"], timeout=8
        )
        rows = processes.strip().splitlines()
        if len(rows) != 1 or PX4 not in rows[0]:
            raise RuntimeError("Expected one Isaac PX4 simulator; flight command refused")

    def execute(self, action, still_allowed):
        with self.operation_lock:
            self.verify_simulator()
            if action == "takeoff":
                state = self.px4("status")
                if not re.search(r"(?m)^INFO\s+\[commander\]\s+Disarmed\s*$", state):
                    raise ValueError("Takeoff requires disarmed PX4 on the ground")
                altitude = self.command(
                    [
                        "wsl.exe",
                        "-d",
                        "Ubuntu",
                        "--exec",
                        PX4 + "-param",
                        "--instance",
                        "0",
                        "show",
                        "-q",
                        "MIS_TAKEOFF_ALT",
                    ],
                    timeout=8,
                )
                height = float(altitude.strip())
                if not math.isfinite(height) or abs(height - 1.0) > 0.001:
                    raise ValueError("Native MIS_TAKEOFF_ALT must be 1 m; no takeoff command sent")
            # Recheck after potentially slow process/parameter reads. Reset/Hold
            # invalidate the request before a delayed command can arm the drone.
            if not still_allowed():
                return False
            args = {
                "takeoff": ("takeoff",),
                "offboard": ("mode", "offboard"),
                "land": ("land",),
                "native_hold": ("mode", "auto:loiter"),
            }[action]
            self.px4(*args)
            return True

    def hold(self):
        # Reset must follow any command already dispatched, never race it.
        with self.operation_lock:
            return super().hold()


class FlightControls:
    def __init__(self, get_report, feedback, backend=None, enabled=lambda: True):
        self.get_report = get_report
        self.feedback = feedback
        self.backend = backend or NativeFlightBackend()
        self.enabled = enabled
        self.lock = threading.RLock()
        self.generation = 0
        self.active = None
        self.state = dict(busy=False, phase="IDLE", reason="Use Take off to 1 m to begin")

    def report(self):
        with self.lock:
            return dict(self.state)

    def allowed(self, action):
        if not self.enabled():
            raise ValueError("Flight controls are unavailable while resetting the experiment")
        report = self.get_report()
        flight = report.get("flight", {})
        if not 0 <= time.monotonic() - flight.get("updated_monotonic_s", -1e9) <= 1.0:
            raise ValueError("Waiting for fresh flight telemetry")
        key = {
            "takeoff": "can_takeoff",
            "offboard": "can_enable_navigation",
            "land": "can_land",
            "native_hold": "can_hold",
        }[action]
        if not flight.get(key):
            raise ValueError(flight.get(key + "_reason", "Flight prerequisites are not met"))

    def start(self, action, request_id):
        self.allowed(action)
        with self.lock:
            if self.state["busy"]:
                raise ValueError(
                    "A flight action is already in progress; use Hold or Reset to interrupt"
                )
            self.generation += 1
            token = self.generation
            self.active = dict(action=action, id=request_id, started=time.monotonic())
            self.state = dict(busy=True, phase="REQUESTING", reason="Requesting PX4 " + action)
            self.feedback(request_id, "accepted", self.state["reason"])
        threading.Thread(target=self.run, args=(token, action, request_id), daemon=True).start()

    def run(self, token, action, request_id):
        def still_allowed():
            with self.lock:
                if token != self.generation:
                    return False
                if (
                    action in ("takeoff", "offboard")
                    and time.monotonic() - self.active["started"] > 5
                ):
                    raise ValueError(
                        "Flight request expired before dispatch; no native command sent"
                    )
                self.allowed(action)
                return True

        try:
            if not self.backend.execute(action, still_allowed):
                return
            with self.lock:
                if token != self.generation:
                    return
                self.state.update(
                    phase="OBSERVING",
                    reason={
                        "takeoff": "Takeoff requested; waiting for an armed stable 1 m hover and live map",
                        "offboard": "Navigation control requested; waiting for PX4 Offboard and READY",
                        "land": "Landing requested; waiting for landed and disarmed",
                        "native_hold": "Hold requested; waiting for PX4 Hold and low speed",
                    }[action],
                )
                self.feedback(request_id, "running", self.state["reason"])
        except Exception as exc:
            with self.lock:
                if token == self.generation:
                    self.state.update(busy=False, phase="FAILED", reason=str(exc))
                    self.active = None
                    self.feedback(request_id, "failed", str(exc))

    def cancel(self, reason):
        with self.lock:
            self.generation += 1
            if self.active:
                self.feedback(self.active["id"], "cancelled", reason)
            self.active = None
            self.state.update(busy=False, phase="CANCELLED", reason=reason)

    def observe(self, report):
        with self.lock:
            if not self.active or self.state["phase"] != "OBSERVING":
                return
            flight = report.get("flight", {})
            action = self.active["action"]
            complete = {
                "takeoff": flight.get("can_enable_navigation"),
                "offboard": report.get("ready"),
                "land": flight.get("landed_disarmed"),
                "native_hold": flight.get("native_hold_stable"),
            }[action]
            if complete:
                reason = {
                    "takeoff": "Stable 1 m hover and live map confirmed. Next: Enable navigation.",
                    "offboard": "PX4 Offboard and READY confirmed. Next: Explore.",
                    "land": "Landed and disarmed confirmed",
                    "native_hold": "Native PX4 Hold confirmed",
                }[action]
                self.feedback(self.active["id"], "completed", reason)
                self.state.update(busy=False, phase="COMPLETE", reason=reason)
                self.active = None
            elif time.monotonic() - self.active["started"] > 60:
                reason = "Flight state was not confirmed within 60 seconds; check status, Hold, Land or Reset"
                self.feedback(self.active["id"], "failed", reason)
                self.active = None
                self.state.update(busy=False, phase="FAILED", reason=reason)
