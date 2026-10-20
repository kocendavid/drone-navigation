"""Frontier geometry and ledger mutation stay off the flight callback."""

import threading

from .background import LatestWorker
from .frontier_ledger import FrontierLedger
from .mission import assess_completion


class AsyncFrontiers:
    def __init__(self):
        self.ledger = FrontierLedger()  # Owned only by the worker.
        self.lock = threading.Lock()
        self.choice = None
        self.consumed = None
        self.worker = LatestWorker("frontier_analysis", self._analyze)
        self.empty = self.ledger.report()

    def _analyze(self, snapshot):
        from .exploration import frontiers

        with self.lock:
            choice, self.choice = self.choice, None
        if choice is not None:
            self.ledger.apply_choice(choice)
        changes = self.ledger.update(snapshot, frontiers(snapshot.model.grid))
        return dict(
            valid=snapshot.valid,
            changes=changes,
            report=self.ledger.report(),
            catalog=self.ledger.snapshot(),
            active_id=self.ledger.active_id,
            completion=assess_completion(snapshot, snapshot.pose[:2]) if snapshot.valid else None,
        )

    def observe_map(self, snapshot):
        self.worker.submit(snapshot.stamp, snapshot)
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

    def ready_for(self, snapshot):
        latest, failure = self.worker.latest()
        return bool(not failure and latest and latest[0] == snapshot.stamp and latest[1]["valid"])

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

    def close(self):
        self.worker.close()
