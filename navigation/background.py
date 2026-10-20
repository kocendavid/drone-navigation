"""Bounded background work: one running job and one newest pending input."""

import threading
import time

from .diagnostic_wire import DiagnosticEmitter


class LatestWorker:
    def __init__(self, name, handler):
        self.handler = handler
        self.condition = threading.Condition()
        self.pending = None
        self.last_key = None
        self.result = None
        self.failure = None
        self.stopping = False
        self.superseded = 0
        self.diagnostics = DiagnosticEmitter(name)
        self.thread = threading.Thread(target=self._run, name=name, daemon=True)
        self.thread.start()

    def submit(self, key, value):
        with self.condition:
            if self.stopping or key == self.last_key:
                return
            self.last_key = key
            self.superseded += int(self.pending is not None)
            self.pending = (key, value)
            self.condition.notify()

    def latest(self):
        with self.condition:
            return self.result, self.failure

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                    if self.stopping:
                        return
                    key, value = self.pending
                    self.pending = None
                began = time.perf_counter()
                try:
                    result = self.handler(value)
                except Exception as exc:
                    with self.condition:
                        self.failure = str(exc)
                        self.condition.notify_all()
                    self.diagnostics.emit("background_error", reason=str(exc))
                    continue
                self.diagnostics.emit(
                    "background_work",
                    duration_s=time.perf_counter() - began,
                    superseded=self.superseded,
                )
                with self.condition:
                    if not self.stopping:
                        self.result = (key, result)
                        self.failure = None
                        self.condition.notify_all()
        finally:
            self.diagnostics.close()

    def close(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify_all()
        self.thread.join(timeout=2)
