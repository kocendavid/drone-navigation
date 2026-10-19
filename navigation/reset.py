"""Bounded simulation reset; the dashboard stays alive while flight nodes are replaced."""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
import threading
import time

from tools.isaac_client import execute
from .mission import assess_completion, save_map


PROJECT = Path(__file__).resolve().parents[1]
PX4 = "/home/finel/PX4-Autopilot/build/px4_sitl_default/bin/px4"


def local_ground_ready(text):
    """Require a valid stationary local estimate, not merely a physical contact."""
    for field in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"):
        if not re.search(r"(?m)^\s+" + field + r":\s+True\s*$", text):
            return False
    values = []
    for field in ("z", "vx", "vy", "vz"):
        match = re.search(r"(?m)^\s+" + field + r":\s+([^\s]+)", text)
        try:
            values.append(float(match.group(1)) if match else math.nan)
        except ValueError:
            return False
    z, vx, vy, vz = values
    return (
        all(math.isfinite(v) for v in values)
        and abs(z) <= 0.15
        and math.hypot(vx, vy) <= 0.1
        and abs(vz) <= 0.1
    )


class EmptyFeed:
    def get(self):
        return None, "Experiment reset: waiting for the new sensor session"


class ResetBackend:
    """Exact local service commands; no arming, takeoff or arbitrary UI shell input."""

    def command(self, args, timeout=35):
        # Windows background services can inherit launcher pipe handles.
        # Wait for the launcher process, rather than waiting for pipe EOF from
        # a service that intentionally remains running.
        with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
            process = subprocess.Popen(
                args,
                stdin=subprocess.DEVNULL,
                stdout=output,
                stderr=errors,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
                raise
            output.seek(0)
            errors.seek(0)
            stdout = output.read(65536).decode("utf-8", errors="replace")
            stderr = errors.read(65536).decode("utf-8", errors="replace")
        if process.returncode:
            raise RuntimeError((stderr or stdout).strip()[-1500:])
        return stdout

    def service(self, action, name):
        return self.command(
            ["powershell.exe", "-NoProfile", "-File", str(PROJECT / "tools/lab.ps1"), action, name]
        )

    def px4(self, *args):
        return self.command(
            ["wsl.exe", "-d", "Ubuntu", "--exec", PX4 + "-commander", "--instance", "0", *args],
            timeout=8,
        )

    def isaac(self, code):
        result = execute(code, timeout=20)
        if result.get("status") != "ok":
            raise RuntimeError(str(result.get("error") or result.get("output") or result)[-1500:])
        return result.get("output", "")

    def inspect_scene(self):
        output = self.isaac(
            "import json, omni.usd, omni.timeline\n"
            "from pxr import UsdGeom, Gf\n"
            "s=omni.usd.get_context().get_stage()\n"
            "assert s is not None, 'Open the experiment scene first'\n"
            "p=s.GetPrimAtPath('/World/quadcopter/chassis')\n"
            "assert p.IsValid(), 'Experiment chassis is missing'\n"
            "m=UsdGeom.XformCache().GetLocalToWorldTransform(p)\n"
            "print(json.dumps(dict(scene=s.GetRootLayer().identifier, "
            "stopped=omni.timeline.get_timeline_interface().is_stopped(), "
            "start_xyz=list(m.ExtractTranslation()), "
            "start_up=list(m.TransformDir(Gf.Vec3d(0,0,1))))))"
        )
        return json.loads(output.strip().splitlines()[-1])

    def hold(self):
        result = subprocess.run(
            ["wsl.exe", "-d", "Ubuntu", "--exec", "pgrep", "-a", "-x", "px4"],
            capture_output=True,
            text=True,
            timeout=8,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        if result.returncode == 1:
            return
        lines = result.stdout.strip().splitlines()
        if result.returncode or len(lines) != 1 or PX4 not in lines[0]:
            raise RuntimeError(
                "Reset requires one Isaac PX4 instance; inspect conflicting simulators"
            )
        state = self.px4("status")
        disarmed = r"(?m)^INFO\s+\[commander\]\s+Disarmed\s*$"
        native_hold = r"(?m)^INFO\s+\[commander\]\s+navigation mode:\s+Hold\s*$"
        if not re.search(disarmed, state):
            self.px4("mode", "auto:loiter")
            state = self.px4("status")
        if not (re.search(disarmed, state) or re.search(native_hold, state)):
            raise RuntimeError("PX4 did not confirm native Hold; flight publishers remain running")

    def stop_scene(self):
        self.isaac("import omni.timeline; omni.timeline.get_timeline_interface().stop()")

    def load_bridge(self):
        self.isaac(
            "import runpy; runpy.run_path(" + repr((PROJECT / "load_bridge.py").as_posix()) + ")"
        )

    def play(self):
        self.isaac("import omni.timeline; omni.timeline.get_timeline_interface().play()")

    def ground_ready(self):
        state = self.px4("status")
        if not re.search(r"(?m)^INFO\s+\[commander\]\s+Disarmed\s*$", state):
            return False
        if "Preflight check: OK" not in self.px4("check"):
            return False
        estimate = self.command(
            [
                "wsl.exe",
                "-d",
                "Ubuntu",
                "--exec",
                PX4 + "-listener",
                "--instance",
                "0",
                "vehicle_local_position",
                "-n",
                "1",
            ],
            timeout=8,
        )
        if not local_ground_ready(estimate):
            return False
        output = self.isaac(
            "import builtins,json,numpy as np\n"
            "b=getattr(builtins,'_px4_isaac_bridge',None)\n"
            "ok=b is not None and not b.closed and b.view is not None\n"
            "if ok:\n"
            " p,r,v,w=b._body_state()\n"
            " ok=(not b.transport.armed and b.transport.fresh(b.config['actuator_timeout_s']) "
            "and r[2,2]>0.98 and np.linalg.norm(v)<0.1 and np.linalg.norm(w)<0.1)\n"
            "print(json.dumps(dict(ready=bool(ok))))"
        )
        return json.loads(output.strip().splitlines()[-1])["ready"]


class ExperimentReset:
    def __init__(self, backend=None, project=PROJECT):
        self.backend = backend or ResetBackend()
        self.project = Path(project)
        self.lock = threading.Lock()
        self.quiesce = threading.Event()
        self.released = threading.Event()
        self.done = threading.Event()
        self.success = False
        self.thread = None
        self.state = dict(
            phase="IDLE", busy=False, reason="Reset returns to the starting pose, disarmed"
        )

    def report(self):
        with self.lock:
            return dict(self.state)

    def update(self, phase, reason, **values):
        with self.lock:
            self.state.update(phase=phase, reason=reason, **values)
            report = dict(self.state)
        if report.get("archive"):
            try:
                (Path(report["archive"]) / "reset.json").write_text(
                    json.dumps(report, indent=2) + "\n", encoding="utf-8"
                )
            except OSError as exc:
                with self.lock:
                    self.state["archive_log_error"] = str(exc)

    def start(self, snapshot, report):
        with self.lock:
            if self.state["busy"]:
                raise ValueError("An experiment reset is already running")
            self.state = dict(
                phase="HOLDING", busy=True, reason="Handing control to native PX4 Hold"
            )
        self.quiesce.clear()
        self.released.clear()
        self.done.clear()
        self.success = False
        self.thread = threading.Thread(target=self.run, args=(snapshot, dict(report)), daemon=True)
        self.thread.start()

    def run(self, snapshot, report):
        try:
            self.backend.hold()
            scene = self.backend.inspect_scene()
            self.update("ARCHIVING", "Saving the previous map and run history")
            folder = (
                self.project
                / "logs"
                / ("experiment_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"))
            )
            if snapshot is not None:
                save_map(snapshot, folder, assess_completion(snapshot, snapshot.pose[:2]))
            else:
                folder.mkdir(parents=True)
            (folder / "run.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            self.update(
                "STOPPING",
                "Clearing the flight controller and mapping session",
                archive=str(folder),
            )
            self.quiesce.set()
            if not self.released.wait(10):
                raise RuntimeError(
                    "Navigator did not release its publishers; simulator reset aborted"
                )
            self.backend.stop_scene()
            baseline = self.backend.inspect_scene()
            if not baseline["stopped"] or baseline["scene"] != scene["scene"]:
                raise RuntimeError("Isaac did not stop in the same scene")
            if baseline["start_up"][2] < 0.98:
                raise RuntimeError("Starting drone pose is not upright; inspect scene placement")
            if not all(math.isfinite(v) for v in baseline["start_xyz"] + baseline["start_up"]):
                raise RuntimeError("Starting drone pose contains non-finite coordinates")
            # Isaac Stop restores scene placement. The saved baseline is a record,
            # never a command to override a placement edited by the user.
            baseline["source"] = "scene_placement"
            baseline_file = self.project / "logs" / "experiment_baseline.json"
            if baseline_file.exists():
                (folder / "previous_baseline.json").write_bytes(baseline_file.read_bytes())
            temporary = baseline_file.with_suffix(".tmp")
            temporary.write_text(json.dumps(baseline, indent=2) + "\n", encoding="utf-8")
            temporary.replace(baseline_file)
            self.update("RESTARTING_PX4", "Starting a fresh PX4 and DDS session", baseline=baseline)
            self.backend.service("stop", "px4")
            self.backend.service("stop", "agent")
            self.backend.service("start", "agent")
            self.backend.service("start", "px4")
            self.update("CONNECTING", "Reloading the Isaac motor/sensor bridge")
            self.backend.load_bridge()
            self.backend.play()
            self.update("SETTLING", "Waiting for fresh motor data and a stable disarmed drone")
            deadline = time.monotonic() + 45
            stable_checks = 0
            while time.monotonic() < deadline:
                try:
                    stable_checks = stable_checks + 1 if self.backend.ground_ready() else 0
                except (RuntimeError, OSError, subprocess.TimeoutExpired):
                    stable_checks = 0
                if stable_checks >= 10:
                    break
                time.sleep(0.5)
            else:
                raise RuntimeError(
                    "Drone did not settle disarmed with a fresh simulator connection"
                )
            self.success = True
            self.update(
                "READY", "Reset complete: starting position restored, map cleared, drone disarmed"
            )
        except Exception as exc:
            if self.released.is_set():
                try:
                    self.backend.stop_scene()
                except Exception:
                    pass
            self.update("FAILED", str(exc))
        finally:
            self.update(self.report()["phase"], self.report()["reason"], busy=False)
            self.done.set()
