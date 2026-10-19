"""Local map/route UI. HTTP commands enter an operator queue, never PX4 directly."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit
import json
import math
from pathlib import Path
from queue import Full
import secrets
import threading
import time
import numpy as np
from .planning import plan
from .history import RunHistory, CommandReceipts
from .centering import routing_geometry
from .diagnostics import read_status


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
    def __init__(self, feed, commands=None, port=8787, history=None):
        self.feed = feed
        self.commands = commands
        self.reset = None
        self.flight_controls = None
        self.flight_available = commands is not None
        self.history = history or RunHistory()
        self.previous_history = None
        self.receipts = CommandReceipts()
        self.logged_receipts = {}
        self.receipt_lock = threading.Lock()
        self.reset_request_id = None
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
                elif self.path.startswith("/api/timing/download"):
                    status = read_status(Path(__file__).parents[1])
                    value = (
                        status.get("log_file")
                        if "recording=1" in self.path
                        else (status.get("last_incident") or {}).get("path")
                    )
                    path = Path(value).resolve() if value else None
                    if (
                        path is None
                        or not path.is_relative_to((Path(__file__).parents[1] / "logs").resolve())
                        or not path.is_file()
                    ):
                        self.respond(404, dict(error="No timing evidence recorded yet"))
                        return
                    size = path.stat().st_size
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "application/x-ndjson" if path.suffix == ".jsonl" else "application/json",
                    )
                    self.send_header(
                        "Content-Disposition", 'attachment; filename="' + path.name + '"'
                    )
                    self.send_header("Content-Length", str(size))
                    self.end_headers()
                    with path.open("rb") as stream:
                        while size:
                            data = stream.read(min(65536, size))
                            if not data:
                                break
                            self.wfile.write(data)
                            size -= len(data)
                elif self.path == "/api/home-route":
                    try:
                        self.respond(200, owner.plan_home())
                    except (ValueError, TypeError) as exc:
                        self.respond(400, dict(error=str(exc)))
                elif self.path.startswith("/api/command-status?id="):
                    query = parse_qs(urlsplit(self.path).query)
                    command = next(
                        (
                            c
                            for c in owner.receipts.report()
                            if c["id"] == query.get("id", [None])[0]
                        ),
                        None,
                    )
                    self.respond(
                        200 if command else 404,
                        dict(command=command) if command else dict(error="Unknown request"),
                    )
                elif self.path.startswith("/api/frame?id="):
                    try:
                        query = parse_qs(urlsplit(self.path).query)
                        history = (
                            owner.previous_history
                            if query.get("previous") == ["1"]
                            else owner.history
                        )
                        frame = history.frame(int(query["id"][0])) if history else None
                    except (ValueError, KeyError):
                        frame = None
                    self.respond(
                        200 if frame else 404,
                        frame or dict(error="Frame is archived; download the full log"),
                    )
                elif self.path in ("/api/history/download", "/api/history/download?previous=1"):
                    history = (
                        owner.previous_history
                        if self.path.endswith("?previous=1")
                        else owner.history
                    )
                    if history is None or history.path is None:
                        self.respond(404, dict(error="No recorded file for this run"))
                        return
                    history.flush()
                    with history.path.open("rb") as stream:
                        stream.seek(0, 2)
                        size = stream.tell()
                        stream.seek(0)
                        self.send_response(200)
                        self.send_header("Content-Type", "application/x-ndjson")
                        self.send_header(
                            "Content-Disposition", 'attachment; filename="experiment-history.jsonl"'
                        )
                        self.send_header("Content-Length", str(size))
                        self.end_headers()
                        while size:
                            data = stream.read(min(size, 65536))
                            if not data:
                                break
                            self.wfile.write(data)
                            size -= len(data)
                else:
                    self.respond(404, dict(error="Not found"))

            def do_POST(self):
                request_id = None
                origin = self.headers.get("Origin")
                wrong_origin = origin and origin != owner.url
                stale_token = self.headers.get("X-Navigation-Token") != owner.token
                if stale_token or wrong_origin:
                    # Consume a small rejected body so Windows does not reset
                    # the TCP connection before the client receives the 403.
                    try:
                        length = int(self.headers.get("Content-Length", "0"))
                        if 0 <= length <= 1024:
                            self.rfile.read(length)
                    except (ValueError, OSError):
                        pass
                    self.respond(
                        403,
                        dict(
                            code="origin_mismatch" if wrong_origin else "session_expired",
                            error=(
                                "Request came from another page origin; open " + owner.url
                                if wrong_origin
                                else "Navigator restarted; refresh the page session"
                            ),
                        ),
                    )
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
                        receipt, is_new = owner.receipts.begin(item)
                        request_id = receipt["id"]
                        if not is_new:
                            self.respond(
                                202,
                                dict(
                                    queued=action,
                                    command=owner.receipts.update(
                                        request_id, receipt["status"], receipt["reason"]
                                    ),
                                ),
                            )
                            return
                        if action == "restart":
                            if owner.reset is None:
                                raise ValueError("Experiment reset is unavailable in this viewer")
                            snap, _ = owner.feed.get()
                            if owner.flight_controls is not None:
                                owner.flight_controls.cancel("Experiment reset requested")
                            owner.reset.start(
                                snap, dict(owner.report, history=owner.history.report())
                            )
                            owner.reset_request_id = request_id
                            owner.preview = []
                            owner.preview_goal = None
                            command = owner.acknowledge_command(
                                request_id, "running", "Reset started; handing control to PX4 Hold"
                            )
                            self.respond(202, dict(queued=action, command=command))
                            return
                        if action not in (
                            "goal",
                            "home",
                            "hold",
                            "resume",
                            "explore",
                            "mission",
                            "takeoff",
                            "offboard",
                            "land",
                        ):
                            raise ValueError("Unknown command")
                        if (
                            action in ("explore", "mission")
                            and type(item.get("continuous", True)) is not bool
                        ):
                            raise ValueError("Continuous exploration must be true or false")
                        if not owner.flight_available or (
                            owner.reset is not None and owner.reset.report()["busy"]
                        ):
                            raise ValueError("Wait for the experiment reset to finish")
                        flight = owner.flight_controls
                        if action in ("takeoff", "offboard", "land") or (
                            action == "hold" and flight is not None and flight.report()["busy"]
                        ):
                            if flight is None:
                                raise ValueError("Flight controls are unavailable in this viewer")
                            if action in ("hold", "land"):
                                flight.allowed("native_hold" if action == "hold" else action)
                                flight.cancel("Operator selected " + action)
                            flight.start("native_hold" if action == "hold" else action, request_id)
                            command = next(
                                c for c in owner.receipts.report() if c["id"] == request_id
                            )
                            self.respond(202, dict(queued=action, command=command))
                            return
                        if action == "goal":
                            # Route search belongs to the controller's worker. HTTP
                            # acceptance must not wait for A* or block operator feedback.
                            item["point"] = validate_destination(item.get("point"))
                        item["request_id"] = request_id
                        item["_deadline"] = receipt["deadline"]
                        command = owner.acknowledge_command(
                            request_id, "queued", "Queued; awaiting the flight controller"
                        )
                        owner.commands.put_nowait(item)
                        self.respond(202, dict(queued=action, command=command))
                    else:
                        self.respond(404, dict(error="Not found"))
                except (ValueError, KeyError, TypeError, Full) as exc:
                    owner.note = str(exc) or "Command queue is full; wait for the pending request"
                    command = owner.acknowledge_command(request_id, "rejected", owner.note)
                    self.respond(400, dict(error=owner.note, command=command))

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def acknowledge_command(self, request_id, status, reason):
        command = self.receipts.update(request_id, status, reason)
        self.log_receipt(command)
        return command

    def log_receipt(self, command):
        if command:
            with self.receipt_lock:
                version = command["status"], command["reason"]
                if self.logged_receipts.get(command["id"]) == version:
                    return
                self.logged_receipts[command["id"]] = version
                if len(self.logged_receipts) > 200:
                    self.logged_receipts.pop(next(iter(self.logged_receipts)))
            self.history.event("command", command["reason"], command=command)

    def observe(self, report, snap, now):
        self.report = report
        if self.flight_controls is not None:
            self.flight_controls.observe(report)
        self.history.observe(report, snap, now)

    def plan_destination(self, point):
        point = validate_destination(point)
        snap, failure = self.feed.get()
        if failure or snap is None or not snap.fresh(time.monotonic()):
            raise ValueError(failure or "Waiting for fresh map")
        return point, plan(snap, snap.pose[:2], point)

    def plan_home(self):
        """Read-only demonstration of the same planner used by Return to start."""
        report = self.report
        run_id = self.history.run_id
        if report.get("home_xy") is None:
            raise ValueError("Start point not captured yet: first establish a stable mapped hover")
        if self.reset is not None and self.reset.report()["busy"]:
            raise ValueError("Wait for the experiment reset to finish")
        point, path = self.plan_destination(report["home_xy"])
        return dict(
            path=path,
            goal=point,
            route_length_m=sum(math.dist(a, b) for a, b in zip(path, path[1:])),
            straight_line_m=math.dist(path[0], point),
            planning_policy="prefer_known_space_centers_and_clearance",
            run_id=run_id,
            note="Preview only. Return replans from the current pose/map and finishes hovering.",
        )

    def state(self):
        snap, failure = self.feed.get()
        now = time.monotonic()
        if self.reset is not None and self.reset_request_id:
            reset = self.reset.report()
            status = (
                "completed"
                if reset["phase"] == "READY"
                else "failed" if reset["phase"] == "FAILED" else "running"
            )
            self.acknowledge_command(
                self.reset_request_id,
                status,
                reset.get("reason", "Reset " + reset["phase"].lower()),
            )
        commands = self.receipts.report()
        for command in commands:
            self.log_receipt(command)
        result = dict(
            **self.report,
            control_enabled=self.commands is not None
            and self.flight_available
            and not (self.reset is not None and self.reset.report()["busy"]),
            experiment_reset=self.reset.report() if self.reset is not None else None,
            flight_controls=self.flight_controls.report() if self.flight_controls else None,
            preview=self.preview,
            preview_goal=self.preview_goal,
            note=self.note,
            map_fresh=False,
            map_reason=failure or "Waiting for map",
            history=self.history.report(),
            previous_history=self.previous_history.report() if self.previous_history else None,
            commands=commands,
            timing_diagnostics=read_status(Path(__file__).parents[1]),
            map_assessment=(
                "Provisional LiDAR observations; enclosure passes the fixed-height completion rule"
                if (self.report.get("mission", {}).get("completion") or {}).get("complete")
                else "Provisional LiDAR observations; enclosure is not yet verified"
            ),
        )
        if snap is not None:
            _, centerline, _ = routing_geometry(snap)
            result.update(
                map_fresh=not failure and snap.fresh(now),
                map_reason=failure or snap.reason,
                size=snap.model.grid.shape[1],
                width=snap.model.grid.shape[1],
                height=snap.model.grid.shape[0],
                origin=snap.model.origin.tolist(),
                resolution=snap.model.resolution,
                grid="".join(np.array(["?", ".", "#", "b"])[snap.model.grid.ravel()].tolist()),
                confirmed_walls="".join(
                    np.where(snap.model.confirmed_walls.ravel(), "1", "0").tolist()
                ),
                wall_policy=snap.model.wall_policy(),
                safe="".join(np.where(snap.safe.ravel(), "1", "0").tolist()),
                centerline="".join(np.where(centerline.ravel(), "1", "0").tolist()),
                planning_policy="prefer_known_space_centers_and_clearance",
                pose=snap.pose if snap.valid else None,
                hit_endpoints_xy=np.round(snap.hits, 3).tolist() if snap.valid else [],
                scans=snap.scans,
                observed_cells=int(((snap.model.grid == 1) | (snap.model.grid == 2)).sum()),
            )
        return result

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.history.close()
