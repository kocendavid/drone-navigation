"""Local authenticated client for NVIDIA's installed Isaac Python server."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket

PROJECT = Path(__file__).resolve().parents[1]
SESSION = PROJECT / "logs" / "isaac_control.json"


def execute(code, *, session=SESSION, timeout=20.0):
    config = json.loads(Path(session).read_text(encoding="utf-8"))
    if config.get("host") != "127.0.0.1" or not config.get("auth_token"):
        raise ValueError("Expected authenticated localhost session")
    request = dict(
        code=code,
        auth_token=config["auth_token"],
        context="drone_lab",
        timeout=timeout,
    )
    with socket.create_connection((config["host"], config["port"]), timeout=timeout) as connection:
        connection.sendall(json.dumps(request).encode("utf-8"))
        connection.shutdown(socket.SHUT_WR)
        chunks = []
        size = 0
        while data := connection.recv(65536):
            size += len(data)
            if size > 8 * 1024 * 1024:
                raise ValueError("Isaac response exceeds 8 MiB")
            chunks.append(data)
    result = json.loads(b"".join(chunks).decode("utf-8"))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--status", action="store_true", help="Read scene/timeline/bridge state")
    operation.add_argument("--run", type=Path, help="Run a local .py script inside Isaac")
    operation.add_argument("--code", help="Execute explicit Python source inside Isaac")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    if not 0 < args.timeout <= 60:
        parser.error("timeout must be between 0 and 60 seconds")
    if args.status:
        code = (
            "import builtins, json, omni.usd, omni.timeline\n"
            "_stage = omni.usd.get_context().get_stage()\n"
            "_timeline = omni.timeline.get_timeline_interface()\n"
            "_bridge = getattr(builtins, '_px4_isaac_bridge', None)\n"
            "print(json.dumps(dict(scene=_stage.GetRootLayer().identifier if _stage else None, "
            "playing=_timeline.is_playing(), stopped=_timeline.is_stopped(), "
            "bridge_loaded=_bridge is not None and not _bridge.closed)))"
        )
    elif args.run:
        path = args.run.resolve(strict=True)
        if path.suffix.lower() != ".py":
            parser.error("--run requires a .py file")
        code = "import runpy\nrunpy.run_path(" + repr(path.as_posix()) + ")\nNone"
    else:
        code = args.code
    try:
        result = execute(code, timeout=args.timeout)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Isaac connection failed: {error}\n")
    audit = PROJECT / "logs" / "lab_control_audit.jsonl"
    audit.parent.mkdir(exist_ok=True)
    with audit.open("a", encoding="utf-8") as stream:
        stream.write(
            json.dumps(
                dict(
                    recorded_utc=datetime.now(timezone.utc).isoformat(),
                    code=code,
                    status=result.get("status"),
                )
            )
            + "\n"
        )
    print(json.dumps(result, indent=2))
    return 0 if result.get("status") == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
