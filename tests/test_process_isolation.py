"""Exercise the actual process boundary under CPU stalls and terminal failures."""

import os
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

from navigation.process_worker import ProcessLatestWorker, ProcessService


def probe_service():
    paths = str(Path(__file__).parent) + os.pathsep + os.environ.get("PYTHONPATH", "")
    with patch.dict(os.environ, PYTHONPATH=paths):
        return ProcessService("test_cpu", "process_jobs:Probe")


class ProcessIsolationTests(unittest.TestCase):
    def test_map_geometry_and_prepared_routes_cross_the_real_process_boundary(self):
        import numpy as np
        from navigation.settings import NavigationSettings
        from navigation.prepared_route import PreparedRoute
        from test_frontier_ledger import maze

        settings = NavigationSettings(
            map_width_cells=20, map_height_cells=20, map_origin_x_m=0, map_origin_y_m=0
        )
        geometry = ProcessService("test_geometry", "navigation.map_geometry:MapGeometry", settings)
        planning = ProcessService("test_planning", "navigation.planning_jobs:PlanningJobs")
        self.addCleanup(geometry.close)
        self.addCleanup(planning.close)
        model, clearance, body, safe, center_distance, centerline = geometry.call(
            dict(
                width=20,
                height=20,
                grid="." * 400,
                confirmed_walls="0" * 400,
            )
        )
        self.assertEqual(model.grid.shape, (20, 20))
        expected = model.geometry()
        for actual, reference in zip((clearance, body, safe), expected):
            np.testing.assert_allclose(actual, reference)
        self.assertEqual(center_distance.shape, model.grid.shape)
        self.assertEqual(centerline.shape, model.grid.shape)
        snap = maze()
        token, route, error, choice = planning.call(
            (
                "plan",
                dict(
                    token=17,
                    snap=snap,
                    start=[3.1, 3.1],
                    goal=[10.1, 3.1],
                    settings=NavigationSettings(),
                ),
            )
        )
        self.assertEqual(token, 17)
        self.assertIsNone(error)
        self.assertIsNone(choice)
        self.assertIsInstance(route, PreparedRoute)
        self.assertTrue(route.safe(snap))
        self.assertTrue(route.with_start([3.2, 3.1]).safe(snap))
        self.assertEqual(len(route.limits), len(route))

    def test_frontier_state_persists_in_child_and_child_loss_revokes_results(self):
        from navigation.async_frontiers import AsyncFrontiers
        from navigation.exploration import choose_frontier
        from test_frontier_ledger import maze

        ledger = AsyncFrontiers()
        self.addCleanup(ledger.close)
        snap = maze()
        ledger.observe_map(snap)
        with ledger.worker.condition:
            self.assertTrue(ledger.worker.condition.wait_for(lambda: ledger.worker.result, 5))
        # This checks persistence across a cold child start. Freshness has its
        # own regressions; process/import startup is not simulated sensor age.
        self.assertTrue(ledger.ready_for(snap, now=snap.generated))
        choice = choose_frontier(snap, snap.pose[:2], catalog=ledger.snapshot())
        ledger.apply_choice(choice)
        snap.stamp += 1
        ledger.observe_map(snap)
        with ledger.worker.condition:
            self.assertTrue(
                ledger.worker.condition.wait_for(lambda: ledger.worker.result[0] == snap.stamp, 3)
            )
        self.assertEqual(ledger.active_id, choice.frontier_id)
        ledger.worker.service.process.terminate()
        ledger.worker.service.process.wait(3)
        self.assertFalse(ledger.ready_for(snap))
        with self.assertRaisesRegex(ValueError, "Background process exited"):
            ledger.observe_map(snap)

    def test_cpu_work_holds_its_own_gil_and_does_not_block_parent(self):
        service = probe_service()
        self.addCleanup(service.close)
        child_pid, _ = service.call(("echo", None))
        self.assertNotEqual(child_pid, os.getpid())
        completed = threading.Event()

        def run():
            service.call(("busy", 0.5))
            completed.set()

        thread = threading.Thread(target=run)
        thread.start()
        # Parent performs useful Python work throughout a child job that keeps
        # the child GIL for 500 ms (five entire command leases).
        until = time.perf_counter() + 0.25
        iterations = 0
        while time.perf_counter() < until:
            iterations += 1
        self.assertFalse(completed.is_set())
        self.assertGreater(iterations, 100)
        thread.join(3)
        self.assertFalse(thread.is_alive())

    def test_latest_input_is_bounded_and_submit_does_not_serialize_or_wait(self):
        paths = str(Path(__file__).parent) + os.pathsep + os.environ.get("PYTHONPATH", "")
        with patch.dict(os.environ, PYTHONPATH=paths):
            worker = ProcessLatestWorker("test_latest", "process_jobs:Probe")
        self.addCleanup(worker.close)
        # Warm the real child before timing the bounded submission path.
        worker.service.call(("echo", None))
        worker.submit(1, ("busy", 0.35))
        with worker.condition:
            self.assertTrue(worker.condition.wait_for(lambda: worker.pending is None, 1))
        began = time.perf_counter()
        for index in range(2, 100):
            worker.submit(index, ("echo", index))
        self.assertLess(time.perf_counter() - began, 0.1)
        with worker.condition:
            self.assertTrue(
                worker.condition.wait_for(lambda: worker.result and worker.result[0] == 99, 3)
            )
        self.assertEqual(worker.result[1][1], 99)
        self.assertGreater(worker.superseded, 0)

    def test_child_errors_and_death_are_visible_and_close_interrupts_a_job(self):
        service = probe_service()
        self.addCleanup(service.close)
        with self.assertRaisesRegex(RuntimeError, "injected child failure"):
            service.call(("error", None))
        self.assertEqual(service.call(("echo", "alive"))[1], "alive")
        with self.assertRaisesRegex(RuntimeError, "Background process failed"):
            service.call(("crash", None))
        service.close()

        service = probe_service()
        self.addCleanup(service.close)
        service.call(("echo", None))
        errors = []

        def wait_for_job():
            try:
                service.call(("busy", 30))
            except RuntimeError as exc:
                errors.append(str(exc))

        thread = threading.Thread(target=wait_for_job)
        thread.start()
        service.close()
        thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertTrue(errors)
        self.assertIsNotNone(service.process.poll())
