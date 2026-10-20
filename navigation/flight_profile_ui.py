"""Local read-only graph viewer, independent of the flight-control process."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import csv
import io
import json
import math
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit


class ProfileServer:
    def __init__(self, folder, port=8788):
        root = Path(folder).resolve()
        stopped = threading.Event()
        cache_lock = threading.Lock()
        cached_signature, cached_rows = None, []

        def current():
            try:
                path = root / "status.json"
                status = json.loads(path.read_text(encoding="utf-8"))
                status["updated_unix_s"] = status.get("updated_unix_s", path.stat().st_mtime)
                return status
            except (OSError, ValueError, TypeError):
                return {}

        def asset(status, view):
            value = (
                status.get("csv")
                if view == "csv"
                else next(
                    (
                        p
                        for p in status.get("graphs", [])
                        if (Path(p).stem.endswith("_navigation")) == (view == "navigation")
                    ),
                    None,
                )
            )
            if not isinstance(value, str):
                return None
            path = Path(value).resolve()
            if (
                path.is_relative_to(root)
                and path.is_file()
                and path.suffix == (".csv" if view == "csv" else ".png")
            ):
                return path
            return None

        def snapshot():
            nonlocal cached_signature, cached_rows
            status = current()
            path = asset(status, "csv")
            with cache_lock:
                try:
                    stat = path.stat() if path else None
                    signature = (str(path), stat.st_size, stat.st_mtime_ns) if stat else None
                    if signature != cached_signature:
                        rows = []
                        if path:
                            # Read a fixed snapshot; ignore an unfinished append at EOF.
                            with path.open("rb") as stream:
                                content = stream.read(stat.st_size)
                            content = content[: content.rfind(b"\n") + 1].decode("utf-8")
                            for row in csv.DictReader(io.StringIO(content)):
                                values = {}
                                for key, value in row.items():
                                    if key in ("phase", "reason", "speed_limit"):
                                        values[key] = value
                                    elif value in ("True", "False"):
                                        values[key] = value == "True"
                                    else:
                                        try:
                                            number = float(value)
                                            values[key] = number if math.isfinite(number) else None
                                        except (ValueError, TypeError):
                                            values[key] = None
                                if values.get("simulation_s") is not None:
                                    rows.append(values)
                        cached_signature, cached_rows = signature, rows
                except (OSError, UnicodeError, csv.Error):
                    # Keep the previous snapshot while an atomic replacement is in progress.
                    pass
                rows = cached_rows
                version = str(cached_signature)
            live = (
                bool(status.get("recording"))
                and 0 <= time.time() - status.get("updated_unix_s", 0) <= 20
            )
            return dict(
                available=bool(rows),
                navigation_available=any(
                    r.get("phase") in ("NAVIGATING", "PLANNING", "SETTLING") for r in rows
                ),
                csv_available=path is not None,
                live=live,
                updated_unix_s=status.get("updated_unix_s"),
                simulation_s=rows[-1]["simulation_s"] if rows else None,
                run=str(path) if path else None,
                version=version,
                samples=rows,
            )

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def send_asset_headers(self, status, mime, size, filename=None):
                self.send_response(status)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(size))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header(
                    "Content-Security-Policy",
                    "frame-ancestors 'self' http://127.0.0.1:8787 http://localhost:8787",
                )
                if filename:
                    self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
                self.end_headers()

            def payload(self, status, value, mime="application/json; charset=utf-8"):
                body = (json.dumps(value) if isinstance(value, dict) else value).encode()
                self.send_asset_headers(status, mime, len(body))
                self.wfile.write(body)

            def do_GET(self):
                request = urlsplit(self.path)
                status = current()
                if request.path == "/":
                    html = (Path(__file__).parents[1] / "config" / "flight_profile.html").read_text(
                        encoding="utf-8"
                    )
                    self.payload(200, html, "text/html; charset=utf-8")
                elif request.path == "/api/profile":
                    data = snapshot()
                    data.pop("samples")
                    self.payload(200, data)
                elif request.path == "/api/samples":
                    self.payload(200, snapshot())
                elif request.path == "/api/events":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Accel-Buffering", "no")
                    self.end_headers()
                    previous = None
                    try:
                        while not stopped.is_set():
                            data = snapshot()
                            version = (data["version"], data["live"], data["updated_unix_s"])
                            if version != previous:
                                body = json.dumps(data, allow_nan=False)
                                self.wfile.write(f"event: profile\ndata: {body}\n\n".encode())
                                self.wfile.flush()
                                previous = version
                            stopped.wait(0.2)
                    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                        pass
                elif request.path in ("/vendor/uPlot.iife.min.js", "/vendor/uPlot.min.css"):
                    path = (
                        Path(__file__).parents[1]
                        / "config"
                        / "vendor"
                        / "uplot"
                        / Path(request.path).name
                    )
                    mime = "text/css" if path.suffix == ".css" else "text/javascript"
                    self.payload(200, path.read_text(encoding="utf-8"), mime + "; charset=utf-8")
                elif request.path in ("/plot", "/samples.csv"):
                    view = (
                        "csv"
                        if request.path == "/samples.csv"
                        else parse_qs(request.query).get("view", ["navigation"])[0]
                    )
                    path = asset(status, view) if view in ("csv", "navigation", "full") else None
                    if path is None:
                        self.payload(404, dict(error="Waiting for recorded graph data"))
                        return
                    # Open one snapshot, including when a live graph is atomically replaced.
                    with path.open("rb") as stream:
                        stream.seek(0, 2)
                        size = stream.tell()
                        stream.seek(0)
                        self.send_asset_headers(
                            200,
                            "text/csv; charset=utf-8" if view == "csv" else "image/png",
                            size,
                            path.name if view == "csv" else None,
                        )
                        while size:
                            data = stream.read(min(size, 65536))
                            if not data:
                                break
                            self.wfile.write(data)
                            size -= len(data)
                else:
                    self.payload(404, dict(error="Not found"))

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.stopped = stopped
        self.server.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.stopped.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
