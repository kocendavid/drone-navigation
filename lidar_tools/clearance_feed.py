"""Own a perception-only Zenoh worker and read its bounded local output pipe."""

import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
from .flight_gate import ClearanceGate, GateDecision


class ClearanceFeed:
    def __init__(self, floor_z=None):
        self.lock = threading.Lock()
        self.failure = None
        session = str(uuid.uuid4())
        self.gate = ClearanceGate(session)
        command = [
            sys.executable,
            "-u",
            str(Path(__file__).parents[1] / "tools" / "forward_clearance.py"),
            "--stream-session",
            session,
        ]
        if floor_z is not None:
            command += ["--floor-z", str(floor_z)]
        environment = dict(
            os.environ, RMW_IMPLEMENTATION="rmw_zenoh_cpp", ROS_DOMAIN_ID="0", PYTHONUNBUFFERED="1"
        )
        flags = (
            subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
            if os.name == "nt"
            else 0
        )
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=None,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            env=environment,
            creationflags=flags,
        )
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            while True:
                line = self.process.stdout.readline(65537)
                if not line:
                    break
                if len(line) > 65536 or not line.endswith("\n"):
                    raise ValueError("Oversized or incomplete LiDAR pipe record")
                if not line.startswith("{"):
                    # Middleware startup diagnostics are not measurement records.
                    print("[LiDAR worker] " + line.strip(), file=sys.stderr, flush=True)
                    continue
                try:
                    envelope = json.loads(line)
                    with self.lock:
                        self.gate.observe(envelope, time.monotonic())
                except ValueError:
                    with self.lock:
                        self.gate.invalidate("Malformed LiDAR pipe JSON")
        except (OSError, ValueError) as exc:
            with self.lock:
                self.failure = str(exc)
        finally:
            with self.lock:
                self.failure = self.failure or "LiDAR worker output closed"

    def decision(self, now):
        with self.lock:
            if self.failure or self.process.poll() is not None:
                return GateDecision(False, "UNKNOWN", self.failure or "LiDAR worker exited")
            return self.gate.decision(now)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.reader.join(timeout=1)
        self.process.stdout.close()
