"""Frontier geometry and ledger mutation stay off the flight callback."""

import threading
import time

from .process_worker import ProcessLatestWorker
from .frontier_ledger import FrontierLedger
from .mission import assess_completion
from .prepared_route import geometry_key


class FrontierAnalyzer:
    def __init__(self):
        self.ledger = FrontierLedger()
        self.choice_version = 0

    def __call__(self, value):
        from .exploration import frontiers

        snapshot, choice, version = value
        if choice is not None and version != self.choice_version:
            self.ledger.apply_choice(choice)
            self.choice_version = version
        changes = self.ledger.update(snapshot, frontiers(snapshot.model.grid))
        return dict(
            valid=snapshot.valid,
            changes=changes,
            report=self.ledger.report(),
            catalog=self.ledger.snapshot(),
            active_id=self.ledger.active_id,
            completion=assess_completion(snapshot, snapshot.pose[:2]) if snapshot.valid else None,
        )


class AsyncFrontiers:
    def __init__(self, worker_factory=None):
        self.lock = threading.Lock()
        self.choice = None
        self.choice_version = 0
        self.consumed = None
        self.worker = (
            worker_factory("frontier_analysis", FrontierAnalyzer())
            if worker_factory
            else ProcessLatestWorker(
                "frontier_analysis", "navigation.async_frontiers:FrontierAnalyzer"
            )
        )
        self.empty = FrontierLedger().report()

    def observe_map(self, snapshot):
        with self.lock:
            choice, version = self.choice, self.choice_version
        # Repeat the versioned choice: replacing a pending map cannot lose it.
        self.worker.submit(snapshot.stamp, (snapshot, choice, version))
        latest, failure = self.worker.latest()
        if failure:
            raise ValueError("Frontier analysis failed: " + failure)
        if latest is None or latest[0] == self.consumed:
            return None
        self.consumed = latest[0]
        return latest[1]["changes"]

    def _view(self):
        latest, _ = self.worker.latest()
        return latest[1] if latest else None

    @property
    def available(self):
        view = self._view()
        return view is not None and view["valid"]

    def planning_view(self, snapshot, now=None):
        """Pair a fresh completed catalog with the exact map that produced it.

        Requiring the newest stamp starves planning when analysis is slower
        than map publication. Flight still validates plans against its newest
        map; completion evidence below deliberately retains its exact stamp.
        """
        latest, failure, source = self.worker.latest_with_input()
        if self.worker.stopping or failure or latest is None or not latest[1]["valid"]:
            return None
        analyzed = source[0]
        now = time.monotonic() if now is None else now
        if not (
            latest[0] == analyzed.stamp
            and snapshot.fresh(now)
            and analyzed.fresh(now)
            and analyzed.stamp <= snapshot.stamp
            and analyzed.scans <= snapshot.scans
            and geometry_key(analyzed.model) == geometry_key(snapshot.model)
        ):
            return None
        return analyzed, [dict(row) for row in latest[1]["catalog"]], latest[1]["active_id"]

    def ready_for(self, snapshot, now=None):
        return self.planning_view(snapshot, now) is not None

    def completion_for(self, snapshot):
        latest, failure = self.worker.latest()
        if failure or not latest or latest[0] != snapshot.stamp or not snapshot.valid:
            return None
        return latest[1]["completion"]

    @property
    def active_id(self):
        view = self._view()
        return view["active_id"] if view else None

    def snapshot(self):
        view = self._view()
        return [dict(row) for row in view["catalog"]] if view else []

    def report(self):
        view = self._view()
        return view["report"] if view else self.empty

    def apply_choice(self, choice):
        with self.lock:
            self.choice = choice
            self.choice_version += 1

    def close(self):
        self.worker.close()
