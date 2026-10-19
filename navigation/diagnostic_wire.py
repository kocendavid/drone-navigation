"""Best-effort localhost observations; failures never affect flight control."""

import json
import socket
import time
import uuid

PORT = 18788


class DiagnosticEmitter:
    def __init__(self, source, port=PORT):
        self.source = source
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.dropped = 0
        self.peer = ("127.0.0.1", port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def emit(self, kind, **fields):
        self.sequence += 1
        try:
            record = dict(
                kind=kind,
                source=self.source,
                session=self.session,
                sequence=self.sequence,
                emitted_wall_s=time.perf_counter(),
                host_clock=time.get_clock_info("perf_counter").implementation,
                producer_monotonic_clock=time.get_clock_info("monotonic").implementation,
                **fields,
            )
            payload = json.dumps(record, allow_nan=False, separators=(",", ":")).encode()
            if len(payload) > 16000:
                raise ValueError("Diagnostic record too large")
            self.socket.sendto(payload, self.peer)
        except (OSError, ValueError, TypeError):
            self.dropped += 1

    def close(self):
        self.socket.close()
