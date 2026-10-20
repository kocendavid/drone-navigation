"""Real subprocess fixtures; never imported by production control."""

import os
import sys
import time


class Probe:
    def __call__(self, request):
        action, value = request
        if action == "crash":
            os._exit(23)
        if action == "error":
            raise ValueError("injected child failure")
        if action == "busy":
            previous = sys.getswitchinterval()
            sys.setswitchinterval(1.0)
            until = time.perf_counter() + value
            count = 0
            try:
                while time.perf_counter() < until:
                    count = (count * 1664525 + 1013904223) & 0xFFFFFFFF
            finally:
                sys.setswitchinterval(previous)
            return os.getpid(), count
        return os.getpid(), value
