"""Passive timestamped lab recorder, independent of navigator/reset lifecycle."""

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from queue import Empty, Full, Queue
import socket
import subprocess
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.diagnostic_wire import PORT
from navigation.diagnostics import analyze, incident_window
from navigation.flight_profile import FlightProfile

PROJECT = Path(__file__).resolve().parents[1]


def atomic_json(path, value, *, busy_ok=False):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    try:
        temporary.replace(path)
    except PermissionError:
        if not busy_ok:
            raise
        # A Windows reader may briefly deny replacement. Keep recording raw
        # events; the next periodic status update retries without blocking UDP.
        print("Diagnostic status file busy; recording continues", file=sys.stderr, flush=True)
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, help="Optional finite read-only capture")
    parser.add_argument(
        "--floor-z", type=float, help="Mapping floor for one-second height profiles"
    )
    args = parser.parse_args()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", PORT))  # One recorder, no port sharing.
    receiver.settimeout(0.02)
    folder = (
        PROJECT / "logs" / ("timing_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"))
    )
    folder.mkdir()
    clock_anchor = dict(unix_s=time.time(), monotonic_s=time.perf_counter())
    atomic_json(folder / "wall_clock.json", clock_anchor)
    floor_z = args.floor_z
    if floor_z is None:
        floor_z = json.loads((PROJECT / "config" / "navigation_settings.json").read_text())[
            "floor_z_m"
        ]
    profile = FlightProfile(
        folder / "flight_profiles", floor_z, clock_anchor["unix_s"] - clock_anchor["monotonic_s"]
    )
    profile_error = None
    log = (folder / "events.jsonl").open("w", encoding="utf-8", buffering=65536)
    status_path = PROJECT / "logs" / "timing_diagnostics.json"
    native_session = uuid.uuid4().hex
    script = (
        "/mnt/"
        + PROJECT.drive[0].lower()
        + str(PROJECT / "tools" / "px4_timing_probe.py")[2:].replace("\\", "/")
    )
    process = subprocess.Popen(
        ["wsl.exe", "-d", "Ubuntu", "--exec", "python3", script],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        encoding="utf-8",
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    pending = Queue(maxsize=2048)
    dropped = [0]

    def read_native():
        for line in process.stdout:
            try:
                item = json.loads(line)
                item.update(
                    source="px4_native",
                    session=native_session,
                    received_wall_s=time.perf_counter(),
                    received_unix_s=time.time(),
                )
                pending.put_nowait(item)
            except (ValueError, Full):
                dropped[0] += 1

    reader = threading.Thread(target=read_native, daemon=True)
    reader.start()
    counts = Counter()
    recent = deque(maxlen=12000)
    sequence = {}
    lost = 0
    incidents = []
    last_incident = None
    last_status = 0
    began = time.perf_counter()
    latest = None
    status = {}
    try:
        while args.duration is None or time.perf_counter() - began < args.duration:
            incoming = []
            try:
                payload, peer = receiver.recvfrom(16384)
                if peer[0] == "127.0.0.1":
                    item = json.loads(payload)
                    if isinstance(item.get("kind"), str) and item.get("session"):
                        item["received_wall_s"] = time.perf_counter()
                        item["received_unix_s"] = time.time()
                        incoming.append(item)
            except socket.timeout:
                pass
            except (ValueError, TypeError):
                dropped[0] += 1
            for _ in range(100):
                try:
                    incoming.append(pending.get_nowait())
                except Empty:
                    break
            now = time.perf_counter()
            for item in incoming:
                counts[item["kind"]] += 1
                key = (item["source"], item["session"])
                if "sequence" in item:
                    number = item["sequence"]
                    if key in sequence and number > sequence[key] + 1:
                        lost += number - sequence[key] - 1
                    sequence[key] = max(number, sequence.get(key, 0))
                recent.append(item)
                log.write(json.dumps(item, allow_nan=False, separators=(",", ":")) + "\n")
                if profile_error is None:
                    try:
                        profile.observe(item)
                    except Exception as error:
                        # Profile/export failures must not interrupt raw recording.
                        profile_error = str(error)
                        print(
                            "Flight profile unavailable: " + profile_error,
                            file=sys.stderr,
                            flush=True,
                        )
                if item["kind"] in ("stop", "deadline_miss"):
                    incidents.append(dict(deadline=now + 2, event=item))
            while recent and now - recent[0]["received_wall_s"] > 20:
                recent.popleft()
            for incident in list(incidents):
                if now >= incident["deadline"]:
                    event = incident["event"]
                    window = incident_window(recent, event)
                    summary = analyze(window, event=event)
                    summary["recording_integrity"] = dict(
                        requested_before_s=10,
                        requested_after_s=2,
                        full_pre_window=began <= event["received_wall_s"] - 10,
                        recorder_drops=dropped[0],
                        sequence_gaps=lost,
                    )
                    name = (
                        "incident_"
                        + str(counts["stop"])
                        + "_"
                        + str(int(event["received_wall_s"] * 1000))
                        + ".json"
                    )
                    path = folder / name
                    atomic_json(path, dict(summary=summary, records=window))
                    last_incident = dict(
                        path=str(path), reason=event.get("reason"), summary=summary
                    )
                    last_incident["summary"] = {
                        k: v for k, v in summary.items() if k != "aligned_samples"
                    }
                    incidents.remove(incident)
            if now - last_status >= 1:
                # Continuous recent-window comparisons are computed off the
                # flight thread. Missing evidence remains explicit.
                latest = analyze(list(recent))
                latest["aligned_samples"] = latest["aligned_samples"][-5:]
                status = dict(
                    schema=1,
                    recording=True,
                    pid=os.getpid(),
                    updated_wall_s=now,
                    updated_unix_s=time.time(),
                    log_file=str(folder / "events.jsonl"),
                    stream_counts=dict(counts),
                    dropped_records=dropped[0],
                    sequence_gaps=lost,
                    native_probe_running=process.poll() is None,
                    flight_profile=dict(
                        csv=str(profile.files[-1]) if profile.files else None,
                        samples=profile.samples,
                        error=profile_error,
                    ),
                    latest=latest,
                    last_incident=last_incident,
                )
                atomic_json(status_path, status, busy_ok=True)
                log.flush()
                last_status = now
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
        process.stdin.close()  # Native probe exits on EOF; no PX4 command is stopped.
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.terminate()
        log.close()
        profile.finish()
        status.update(
            recording=False,
            updated_wall_s=time.perf_counter(),
            updated_unix_s=time.time(),
            flight_profile=dict(
                csv=str(profile.files[-1]) if profile.files else None,
                samples=profile.samples,
                error=profile_error,
            ),
        )
        atomic_json(status_path, status, busy_ok=True)


if __name__ == "__main__":
    main()
