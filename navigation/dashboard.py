"""Local map/route UI. HTTP commands enter an operator queue, never PX4 directly."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from queue import Full
import secrets
import threading
import time
import numpy as np
from .planning import plan


def validate_destination(point):
    """Use the same finite XY schema for route previews and queued goals."""
    if (
        not isinstance(point, list)
        or len(point) != 2
        or not all(type(value) in (float, int) and math.isfinite(value) for value in point)
    ):
        raise ValueError("Choose a finite destination")
    return [float(value) for value in point]


class Dashboard:
    def __init__(self, feed, commands=None, port=8787):
        self.feed = feed
        self.commands = commands
        self.token = secrets.token_urlsafe(24)
        self.report = dict(
            phase="OBSERVING", reason="Map preview; flight controls disabled", ready=False
        )
        self.preview = []
        self.preview_goal = None
        self.note = "Click an observed destination to preview a route"
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def respond(self, status, payload, kind="application/json"):
                body = (
                    json.dumps(payload, allow_nan=False).encode()
                    if kind == "application/json"
                    else payload.encode()
                )
                self.send_response(status)
                self.send_header("Content-Type", kind + "; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Frame-Options", "DENY")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/":
                    html = (Path(__file__).parents[1] / "config" / "navigation.html").read_text(
                        encoding="utf-8"
                    )
                    self.respond(200, html.replace("__TOKEN__", owner.token), "text/html")
                elif self.path == "/api/state":
                    self.respond(200, owner.state())
                else:
                    self.respond(404, dict(error="Not found"))

            def do_POST(self):
                origin = self.headers.get("Origin")
                if self.headers.get("X-Navigation-Token") != owner.token or (
                    origin and origin != owner.url
                ):
                    self.respond(403, dict(error="Use this navigator window"))
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 1024:
                        raise ValueError("Invalid command size")
                    if self.headers.get("Content-Type") != "application/json":
                        raise ValueError("JSON command required")
                    item = json.loads(self.rfile.read(length))
                    if not isinstance(item, dict):
                        raise ValueError("Command must be a JSON object")
                    if self.path == "/api/preview":
                        point, route = owner.plan_destination(item.get("point"))
                        owner.preview = route
                        owner.preview_goal = point
                        owner.note = f"Route preview: {sum(math.dist(a,b) for a,b in zip(owner.preview,owner.preview[1:])):.1f} m"
                        self.respond(200, dict(path=owner.preview, goal=owner.preview_goal))
                    elif self.path == "/api/command":
                        if owner.commands is None:
                            raise ValueError("This is a map preview; flight controls are disabled")
                        action = item.get("action")
                        if action not in ("goal", "hold", "resume", "explore"):
                            raise ValueError("Unknown command")
                        if action == "goal":
                            # Require a reachable route before queueing operator intent.
                            item["point"], _ = owner.plan_destination(item.get("point"))
                        owner.commands.put_nowait(item)
                        self.respond(202, dict(queued=action))
                    else:
                        self.respond(404, dict(error="Not found"))
                except (ValueError, KeyError, TypeError, Full) as exc:
                    owner.note = str(exc)
                    self.respond(400, dict(error=str(exc)))

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def plan_destination(self, point):
        point = validate_destination(point)
        snap, failure = self.feed.get()
        if failure or snap is None or not snap.fresh(time.monotonic()):
            raise ValueError(failure or "Waiting for fresh map")
        return point, plan(snap, snap.pose[:2], point)

    def state(self):
        snap, failure = self.feed.get()
        now = time.monotonic()
        result = dict(
            **self.report,
            control_enabled=self.commands is not None,
            preview=self.preview,
            preview_goal=self.preview_goal,
            note=self.note,
            map_fresh=False,
            map_reason=failure or "Waiting for map",
        )
        if snap is not None:
            result.update(
                map_fresh=not failure and snap.fresh(now),
                map_reason=failure or snap.reason,
                size=snap.model.grid.shape[0],
                origin=snap.model.origin.tolist(),
                resolution=snap.model.resolution,
                grid="".join(np.array(["?", ".", "#", "b"])[snap.model.grid.ravel()].tolist()),
                safe="".join(np.where(snap.safe.ravel(), "1", "0").tolist()),
                pose=snap.pose,
                scans=snap.scans,
                observed_cells=int(((snap.model.grid == 1) | (snap.model.grid == 2)).sum()),
            )
        return result

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
