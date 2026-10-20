"""Planning may use fresh completed analysis without waiting for the newest map."""

from dataclasses import replace
import threading
import unittest

from navigation.async_frontiers import AsyncFrontiers
from navigation.background import LatestWorker
from navigation.mapping import RayMap
from test_frontier_ledger import maze


class AsyncFrontierTests(unittest.TestCase):
    def test_maps_outpacing_analysis_still_provide_a_coherent_planning_view(self):
        entered, release, later_entered, later_release = (threading.Event() for _ in range(4))

        def factory(name, handler):
            def analyze(value):
                if value[0].stamp == 1:
                    entered.set()
                    release.wait(3)
                else:
                    later_entered.set()
                    later_release.wait(3)
                return handler(value)

            return LatestWorker(name, analyze)

        ledger = AsyncFrontiers(worker_factory=factory)
        original = maze()
        try:
            ledger.observe_map(original)
            self.assertTrue(entered.wait(1))
            for index in range(2, 13):
                newest = replace(original, stamp=index, scans=index)
                ledger.observe_map(newest)
            release.set()
            self.assertTrue(later_entered.wait(2))
            with ledger.worker.condition:
                self.assertTrue(ledger.worker.condition.wait_for(lambda: ledger.worker.result, 2))
            view = ledger.planning_view(newest, now=original.generated + 0.1)
            self.assertIsNotNone(view)
            source, catalog, active_id = view
            self.assertIs(source, original)
            self.assertEqual(catalog, ledger.worker.latest()[0][1]["catalog"])
            self.assertEqual(active_id, ledger.worker.latest()[0][1]["active_id"])
            self.assertTrue(ledger.ready_for(newest, now=original.generated + 0.1))
            # Lagging analysis cannot claim that the current map is complete.
            self.assertIsNone(ledger.completion_for(newest))
            self.assertGreater(ledger.worker.superseded, 0)
        finally:
            release.set()
            later_release.set()
            ledger.close()

    def test_planning_rejects_expired_invalid_reset_and_incompatible_sources(self):
        ledger = AsyncFrontiers(worker_factory=LatestWorker)
        self.addCleanup(ledger.close)
        source = maze()
        ledger.observe_map(source)
        with ledger.worker.condition:
            self.assertTrue(ledger.worker.condition.wait_for(lambda: ledger.worker.result, 2))
        now = source.generated + 0.1
        current = replace(source, stamp=2, scans=2)
        self.assertTrue(ledger.ready_for(current, now))
        for invalid in (
            replace(current, valid=False),
            replace(current, stamp=0),
            replace(current, scans=0),
            replace(current, model=RayMap()),
            replace(current, cloud_wall=now - 1.1),
        ):
            with self.subTest(snapshot=invalid.reason, stamp=invalid.stamp):
                self.assertIsNone(ledger.planning_view(invalid, now))
        # Even a freshly published current map cannot freshen old analysis.
        later = source.generated + 0.9
        current = replace(current, generated=later, cloud_wall=later, clock_wall=later)
        self.assertIsNone(ledger.planning_view(current, later))
        with ledger.worker.condition:
            ledger.worker.failure = "analysis failed"
        self.assertIsNone(ledger.planning_view(source, now))
        ledger.close()
        self.assertIsNone(ledger.planning_view(source, now))
