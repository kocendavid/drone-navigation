"""Read-only native uORB snapshots. Run in WSL; never publishes flight data."""

import json
from pathlib import Path
import select
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.diagnostics import parse_listener

PX4 = "/home/finel/PX4-Autopilot/build/px4_sitl_default/bin/px4-listener"


def main():
    while True:
        if select.select([sys.stdin], [], [], 0)[0] and not sys.stdin.read(1):
            return
        began = time.monotonic()
        pid = subprocess.run(
            ["pgrep", "-x", "px4"], capture_output=True, text=True, timeout=1
        ).stdout.strip()
        try:
            if len(pid.splitlines()) != 1:
                raise ValueError("Expected one Isaac PX4 process")
            executable = str((Path("/proc") / pid / "exe").resolve())
            command = (Path("/proc") / pid / "cmdline").read_bytes().decode(errors="replace")
            if "/px4_sitl_default/bin/px4" not in executable or "rcS-isaac" not in command:
                raise ValueError("Native probe requires the Isaac SITL clock")
        except (OSError, ValueError) as error:
            print(json.dumps(dict(kind="native_probe_error", error=str(error))), flush=True)
            time.sleep(0.2)
            continue
        for topic, kind in (
            ("vehicle_local_position", "px4_position"),
            ("trajectory_setpoint", "px4_setpoint"),
        ):
            started = time.monotonic()
            try:
                result = subprocess.run(
                    [PX4, "--instance", "0", topic, "-n", "1"],
                    capture_output=True,
                    text=True,
                    timeout=1,
                )
                fields = parse_listener(result.stdout)
                if fields.get("timestamp"):
                    print(
                        json.dumps(
                            dict(
                                kind=kind,
                                px4_pid=pid,
                                fields=fields,
                                native_poll_duration_s=time.monotonic() - started,
                                native_capture_wall_s=started,
                                native_clock_domain="WSL monotonic; not Windows host monotonic",
                            ),
                            allow_nan=False,
                        ),
                        flush=True,
                    )
            except (OSError, subprocess.TimeoutExpired) as error:
                print(json.dumps(dict(kind="native_probe_error", error=str(error))), flush=True)
        time.sleep(max(0.01, 0.05 - (time.monotonic() - began)))


if __name__ == "__main__":
    main()
