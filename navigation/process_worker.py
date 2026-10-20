"""Private subprocess RPC. Only relay threads serialize or wait for replies.

Workers run below normal priority on Windows and limit numerical thread pools.
No ROS flight publishers, command clocks or control state cross this boundary.
"""

import importlib
import os
from pathlib import Path
import pickle
import struct
import subprocess
import sys
import threading

from .background import LatestWorker


def send(stream, value):
    payload = pickle.dumps(value, protocol=5)
    stream.write(struct.pack("!Q", len(payload)))
    stream.write(payload)
    stream.flush()


def receive(stream):
    def read(size):
        chunks = bytearray()
        while len(chunks) < size:
            part = stream.read(size - len(chunks))
            if not part:
                raise EOFError("Background process closed its pipe")
            chunks.extend(part)
        return chunks

    size = struct.unpack("!Q", read(8))[0]
    if size > 64 * 1024 * 1024:
        raise ValueError("Background message exceeds 64 MiB")
    return pickle.loads(read(size))  # Private child pipe, never network input.


class ProcessService:
    def __init__(self, name, factory, *arguments):
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(str(path) for path in sys.path if path)
        for key in (
            "OPENBLAS_NUM_THREADS",
            "OMP_NUM_THREADS",
            "MKL_NUM_THREADS",
            "NUMEXPR_NUM_THREADS",
        ):
            env[key] = "1"
        flags = (
            subprocess.CREATE_NO_WINDOW | subprocess.BELOW_NORMAL_PRIORITY_CLASS
            if os.name == "nt"
            else 0
        )
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", "navigation.process_worker", name, factory],
            cwd=Path(__file__).resolve().parents[1],
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            creationflags=flags,
        )
        self.arguments = arguments
        self.initialized = False
        self.closed = False
        self.lock = threading.Lock()

    def call(self, value):
        """Blocking RPC: call only from a background/HTTP thread."""
        with self.lock:
            if self.closed:
                raise RuntimeError("Background service is closed")
            try:
                if not self.initialized:
                    send(self.process.stdin, self.arguments)
                    self.initialized = True
                send(self.process.stdin, value)
                result, error = receive(self.process.stdout)
            except (OSError, EOFError) as exc:
                raise RuntimeError(
                    f"Background process failed (exit {self.process.poll()}): {exc}"
                ) from exc
            if error:
                raise RuntimeError(error)
            return result

    def close(self):
        # Terminate first: an unresponsive job must not strand a relay thread.
        self.closed = True
        if self.process.poll() is None:
            self.process.terminate()
        self.process.wait(timeout=3)
        with self.lock:
            self.process.stdin.close()
            self.process.stdout.close()


class ProcessLatestWorker(LatestWorker):
    def __init__(self, name, factory, *arguments):
        self.service = ProcessService(name, factory, *arguments)
        super().__init__(name, self.service.call)

    def latest(self):
        if not self.stopping and self.service.process.poll() is not None:
            return None, f"Background process exited: {self.service.process.returncode}"
        return super().latest()

    def close(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify_all()
        self.service.close()
        super().close()


def main():
    from .diagnostic_wire import DiagnosticEmitter
    import time

    name, factory = sys.argv[1:]
    # stdout belongs exclusively to the framed protocol, including imports.
    output = os.fdopen(os.dup(sys.stdout.fileno()), "wb")
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sys.stdout = sys.stderr
    module, attribute = factory.split(":")
    handler = getattr(importlib.import_module(module), attribute)(*receive(sys.stdin.buffer))
    diagnostics = DiagnosticEmitter(name + "_process")
    try:
        while True:
            try:
                value = receive(sys.stdin.buffer)
            except EOFError:
                return
            wall, cpu = time.perf_counter(), time.process_time()
            job_id = diagnostics.sequence + 1
            diagnostics.emit("background_started", pid=os.getpid(), job_id=job_id)
            try:
                result, error = handler(value), None
            except Exception as exc:
                result, error = None, f"{type(exc).__name__}: {exc}"
            diagnostics.emit(
                "background_work",
                pid=os.getpid(),
                job_id=job_id,
                duration_s=time.perf_counter() - wall,
                cpu_s=time.process_time() - cpu,
                error=error,
            )
            send(output, (result, error))
    finally:
        diagnostics.close()
        output.close()
        close = getattr(handler, "close", None)
        if close:
            close()


if __name__ == "__main__":
    main()
