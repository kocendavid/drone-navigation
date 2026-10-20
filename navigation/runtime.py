"""Version reporting and history handoff when Reset reloads the Python process."""

from collections import deque
import hashlib
import json
from pathlib import Path
import shutil
import time

RELOAD_EXIT_CODE = 75


class RuntimeVersion:
    def __init__(self, project):
        self.project = Path(project)
        self.loaded = self.fingerprint()
        self.disk = self.loaded
        self.next_check = 0.0

    def fingerprint(self):
        files = sorted((self.project / "navigation").glob("*.py")) + [
            self.project / name
            for name in (
                "ros2_navigate.py",
                "ros2_hover.py",
                "ros2_step.py",
                "config/navigation_settings.json",
            )
        ]
        for name in ("native_control", "native_navigation"):
            native = self.project / name
            if native.exists():
                files += sorted(native.glob("*.cpp")) + sorted(native.glob("*.hpp"))
                files += [native / "CMakeLists.txt"]
        digest = hashlib.sha256()
        for path in files:
            digest.update(str(path.relative_to(self.project)).encode())
            digest.update(path.read_bytes())
        return digest.hexdigest()[:12]

    def report(self):
        now = time.monotonic()
        if now >= self.next_check:
            try:
                self.disk = self.fingerprint()
            except OSError:
                self.disk = "unavailable"
            self.next_check = now + 2.0
        return dict(loaded=self.loaded, disk=self.disk, reload_pending=self.loaded != self.disk)


def prepare_reload(project, reset_report, history, request_id):
    """Called only after successful disarmed Reset and publisher quiescence."""
    project = Path(project)
    archive = Path(reset_report["archive"]).resolve()
    if not archive.is_relative_to((project / "logs").resolve()):
        raise ValueError("Reset archive must be inside this lab's logs")
    if reset_report["phase"] != "READY" or reset_report["busy"]:
        raise ValueError("Reload requires a completed ground reset")
    history.close()
    summary = history.report()
    if history.path:
        target = archive / "history.jsonl"
        if history.path.resolve() != target.resolve():
            shutil.copyfile(history.path, target)
        summary["log_file"] = str(target)
    context = dict(reset=reset_report, history=summary, request_id=request_id)
    path = project / "logs" / "navigator_reload.json"
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(context, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def restore_reload(project, dashboard):
    from .history import RunHistory

    project = Path(project)
    path = project / "logs" / "navigator_reload.json"
    if not path.exists():
        return None
    context = json.loads(path.read_text(encoding="utf-8"))
    summary = context["history"]
    previous = RunHistory()
    previous.closed = True
    previous.run_id = summary["run_id"]
    previous.events = deque(summary["events"], maxlen=200)
    previous.targets = deque(summary["targets"], maxlen=1000)
    previous.trail = deque(summary["trail"], maxlen=5000)
    if summary["log_file"]:
        previous.path = Path(summary["log_file"]).resolve()
        if not previous.path.is_relative_to((project / "logs").resolve()):
            raise ValueError("Archived history must be inside this lab's logs")
        with previous.path.open("rb") as stream:
            while True:
                offset = stream.tell()
                line = stream.readline()
                if not line:
                    break
                row = json.loads(line)
                if row["run_id"] != previous.run_id:
                    raise ValueError("Archived history run mismatch")
                if row["type"] == "frame":
                    previous.frame_offsets[row["id"]] = offset
                    while len(previous.frame_offsets) > 10000:
                        previous.frame_offsets.popitem(last=False)
                    previous.frames[row["id"]] = row
                    while len(previous.frames) > 80:
                        previous.frames.popitem(last=False)
    dashboard.previous_history = previous
    path.unlink()
    return context
