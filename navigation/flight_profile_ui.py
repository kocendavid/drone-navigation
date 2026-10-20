"""Local read-only graph viewer, independent of the flight-control process."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import csv
import json
import math
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit


class ProfileTail:
    """Parse only complete appended CSV records; keep partial records for retry."""

    def __init__(self):
        self.signature = None
        self.identity = None
        self.rows = []
        self.headers = None
        self.offset = 0
        self.anchor = b""
        self.epoch = 0
        self.navigation_available = False
        self.bytes_read = 0

    def reset(self, identity):
        self.identity = identity
        self.rows = []
        self.headers = None
        self.offset = 0
        self.anchor = b""
        self.epoch += 1
        self.navigation_available = False

    @staticmethod
    def values(headers, fields):
        result = {}
        for index, key in enumerate(headers):
            value = fields[index] if index < len(fields) else None
            if key in ("phase", "reason", "speed_limit"):
                result[key] = value
            elif value in ("True", "False"):
                result[key] = value == "True"
            else:
                try:
                    number = float(value)
                    result[key] = number if math.isfinite(number) else None
                except (ValueError, TypeError):
                    result[key] = None
        return result

    def update(self, path):
        if path is None:
            if self.identity is not None:
                self.reset(None)
                self.signature = None
            return
        stat = path.stat()
        signature = (str(path), stat.st_size, stat.st_mtime_ns)
        if signature == self.signature:
            return
        identity = (str(path), stat.st_dev, stat.st_ino)
        with path.open("rb") as stream:
            replaced = identity != self.identity or stat.st_size < self.offset
            if not replaced and self.signature is not None:
                replaced = stat.st_size <= self.signature[1]
                if not replaced and self.anchor:
                    stream.seek(self.offset - len(self.anchor))
                    anchor = stream.read(len(self.anchor))
                    self.bytes_read += len(anchor)
                    replaced = anchor != self.anchor
            if replaced:
                self.reset(identity)
            begin = self.offset
            stream.seek(begin)
            content = stream.read(max(0, stat.st_size - begin))
            self.bytes_read += len(content)
            lines = content.splitlines(keepends=True)
            if lines and not lines[-1].endswith(b"\n"):
                lines.pop()
            ends, position = [], begin
            for line in lines:
                position += len(line)
                ends.append(position)
            reader = csv.reader((line.decode("utf-8") for line in lines), strict=True)
            try:
                for fields in reader:
                    if self.headers is None:
                        self.headers = fields
                    elif fields:
                        row = self.values(self.headers, fields)
                        if row.get("simulation_s") is not None:
                            self.rows.append(row)
                            self.navigation_available |= row.get("phase") in (
                                "NAVIGATING",
                                "PLANNING",
                                "SETTLING",
                            )
                    self.offset = ends[reader.line_num - 1]
            except csv.Error:
                # A quoted multi-line record can end mid-write. Retry only
                # that unfinished record when the file grows again.
                pass
            stream.seek(max(0, self.offset - 64))
            self.anchor = stream.read(min(64, self.offset))
            self.bytes_read += len(self.anchor)
        self.signature = signature


class ProfileServer:
    def __init__(self, folder, port=8788, status_path=None):
        root = Path(folder).resolve()
        stopped = threading.Event()
        cache_lock = threading.Lock()
        cache = ProfileTail()
        status_path = Path(status_path) if status_path else root / "status.json"

        def current():
            try:
                path = status_path
                status = json.loads(path.read_text(encoding="utf-8"))
                status["updated_unix_s"] = status.get("updated_unix_s", path.stat().st_mtime)
                if "flight_profile" in status:
                    status = dict(
                        status["flight_profile"],
                        recording=status.get("recording", False),
                        updated_unix_s=status["updated_unix_s"],
                    )
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

        def snapshot(cursor=None, include_samples=True):
            status = current()
            path = asset(status, "csv")
            with cache_lock:
                try:
                    cache.update(path)
                except (OSError, UnicodeError, csv.Error):
                    pass  # Keep the last complete records while a writer replaces the file.
                count = len(cache.rows)
                reset = cursor is None or cursor[0] != cache.epoch or cursor[1] > count
                offset = 0 if reset else cursor[1]
                latest = cache.rows[-1] if count else {}
                live = bool(status.get("recording")) and (
                    0 <= time.time() - status.get("updated_unix_s", 0) <= 20
                )
                return dict(
                    available=bool(count),
                    navigation_available=cache.navigation_available,
                    csv_available=path is not None,
                    live=live,
                    updated_unix_s=status.get("updated_unix_s"),
                    simulation_s=latest.get("simulation_s"),
                    timestamp_unix_s=latest.get("timestamp_unix_s"),
                    run=str(path) if path else None,
                    version=f"{cache.epoch}:{count}",
                    epoch=cache.epoch,
                    reset=reset,
                    offset=offset,
                    total_samples=count,
                    samples=list(cache.rows[offset:]) if include_samples else [],
                    error=status.get("error"),
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
                    data = snapshot(include_samples=False)
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
                    previous, cursor = None, None
                    try:
                        while not stopped.is_set():
                            data = snapshot(cursor)
                            version = (data["version"], data["live"], data["updated_unix_s"])
                            if version != previous:
                                body = json.dumps(data, allow_nan=False)
                                self.wfile.write(f"event: profile\ndata: {body}\n\n".encode())
                                self.wfile.flush()
                                previous = version
                                cursor = (data["epoch"], data["total_samples"])
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

        self.profile_cache = cache
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
