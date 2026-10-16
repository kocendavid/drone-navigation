"""Operator bootstrap: authenticated localhost access to Isaac's Python server.

Changes only the execution-server extension and its settings. No scene, physics,
timeline or PX4 changes. Run once per Isaac session from its Script Editor.
"""

import asyncio
import json
from pathlib import Path
import secrets

import carb.settings
import omni.kit.app

PROJECT = Path(__file__).resolve().parents[1]
SESSION = PROJECT / "logs" / "isaac_control.json"
EXTENSION = "isaacsim.code_editor.python_server"
PREFIX = "/exts/" + EXTENSION
manager = omni.kit.app.get_app().get_extension_manager()
settings = carb.settings.get_settings()

if not manager.is_extension_enabled(EXTENSION):
    settings.set(PREFIX + "/host", "127.0.0.1")
    settings.set(PREFIX + "/port", 8226)
    settings.set(PREFIX + "/require_auth", True)
    settings.set(PREFIX + "/auth_token", secrets.token_urlsafe(32))
    settings.set(PREFIX + "/carb_logs", False)
    manager.set_extension_enabled_immediate(EXTENSION, True)

from isaacsim.code_editor.python_server import get_server_status, ServerState


async def record_ready():
    for _ in range(100):
        status = get_server_status()
        if status is not None and status.state == ServerState.RUNNING:
            assert (
                status.authentication_required
            ), "Existing Python server must require authentication"
            assert status.bound_endpoints, "Python server has no listener"
            assert all(
                endpoint.host == "127.0.0.1" for endpoint in status.bound_endpoints
            ), "Expected localhost-only listener"
            token = settings.get(PREFIX + "/auth_token")
            assert token, "Authentication token missing"
            endpoint = status.bound_endpoints[0]
            SESSION.parent.mkdir(exist_ok=True)
            SESSION.write_text(
                json.dumps(dict(host=endpoint.host, port=endpoint.port, auth_token=token), indent=2)
                + "\n",
                encoding="utf-8",
            )
            print(
                f"[Lab control] READY: authenticated localhost:{endpoint.port}. "
                "Scene, physics, timeline and PX4 unchanged."
            )
            return
        if status is not None and status.last_error:
            raise RuntimeError(status.last_error)
        await asyncio.sleep(0.1)
    raise RuntimeError("Python server did not become ready within 10 seconds")


task = asyncio.ensure_future(record_ready())


def report_failure(completed):
    if not completed.cancelled() and completed.exception() is not None:
        print("[Lab control] ERROR:", completed.exception())


task.add_done_callback(report_failure)
print("[Lab control] Starting local connection; waiting for READY. Token stays in ignored logs/.")
