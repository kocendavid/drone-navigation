"""Per-callback wall and CPU timings, including every early return."""

import time


class ControlTiming:
    def __init__(self, deadline_ns=100_000_000):
        self.started_ns = self.last_ns = time.perf_counter_ns()
        self.started_cpu_ns = self.last_cpu_ns = time.thread_time_ns()
        self.deadline_ns = deadline_ns
        self.stages = {}
        self.submission = None
        self.outcome = "no_command"
        self.callback_gap_s = None
        self.previous_command_published = False

    def mark(self, name):
        now, cpu = time.perf_counter_ns(), time.thread_time_ns()
        previous = self.stages.setdefault(name, dict(wall_s=0.0, cpu_s=0.0))
        previous["wall_s"] += (now - self.last_ns) / 1e9
        previous["cpu_s"] += (cpu - self.last_cpu_ns) / 1e9
        self.last_ns, self.last_cpu_ns = now, cpu

    def report(self):
        wall = (time.perf_counter_ns() - self.started_ns) / 1e9
        cpu = (time.thread_time_ns() - self.started_cpu_ns) / 1e9
        submission = self.submission
        cause = None
        if submission and submission.get("reason") == "stale_command":
            cause = "stale_command"
        elif submission and submission.get("completion_age_s", 0) > self.deadline_ns / 1e9:
            cause = "native_publication_overrun"
        elif (
            self.previous_command_published
            and self.callback_gap_s is not None
            and self.callback_gap_s > self.deadline_ns / 1e9
        ):
            cause = "callback_arrival_late"
        return dict(
            started_qpc_ns=self.started_ns,
            deadline_s=self.deadline_ns / 1e9,
            duration_s=wall,
            callback_gap_s=self.callback_gap_s,
            scheduling_delay_s=(
                max(0.0, self.callback_gap_s - 0.05) if self.callback_gap_s is not None else None
            ),
            cpu_s=cpu,
            off_cpu_s=max(0.0, wall - cpu),
            stages={k: dict(v) for k, v in self.stages.items()},
            slowest_stage=max(self.stages, key=lambda k: self.stages[k]["wall_s"], default=None),
            outcome=self.outcome,
            submission=submission,
            deadline_missed=cause is not None,
            deadline_cause=cause,
        )
