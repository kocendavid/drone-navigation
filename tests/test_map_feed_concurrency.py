"""Clock progress under stalled geometry, bounded backlog and terminal failures."""

import json
from queue import Queue
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from navigation.centering import center_geometry
from navigation.feed import MapFeed
from navigation.settings import NavigationSettings
from navigation.timing import TrajectoryClock


class MapFeedConcurrencyTests(unittest.TestCase):
    def setUp(self):
        self.lines = Queue()
        self.feed = feed = MapFeed.__new__(MapFeed)
        feed.settings = NavigationSettings(
            map_width_cells=20,
            map_height_cells=20,
            map_origin_x_m=0,
            map_origin_y_m=0,
            map_resolution_m=0.2,
        )
        from navigation.map_geometry import MapGeometry

        feed.geometry = SimpleNamespace(call=MapGeometry(feed.settings))
        feed.lock = threading.Lock()
        feed.map_condition = threading.Condition(feed.lock)
        feed.pending_map = None
        feed.stopping = False
        feed.maps_superseded = 0
        feed.trajectory_clock = TrajectoryClock()
        feed.session = "concurrency-test"
        feed.snapshot = feed.failure = feed.stamp = None
        feed.confirmed = 0
        feed.process = SimpleNamespace(
            stdout=SimpleNamespace(readline=lambda limit: self.lines.get(timeout=3)),
            poll=lambda: None,
        )
        self.clock_read = threading.Event()
        feed.diagnostics = SimpleNamespace(
            emit=lambda kind, **fields: (
                self.clock_read.set()
                if kind == "feed_clock" and fields["sim_ns"] == 350_000_000
                else None
            )
        )
        self.entered = threading.Event()
        self.release = threading.Event()
        self.now = patch("navigation.feed.time.monotonic", return_value=100.0)
        self.wall_clock = self.now.start()
        self.addCleanup(self.now.stop)
        self.threads = []
        self.addCleanup(self.cleanup_threads)

    def cleanup_threads(self):
        self.release.set()
        self.feed._fail("Test cleanup")
        self.lines.put("")
        for thread in self.threads:
            thread.join(timeout=2)
            self.assertFalse(thread.is_alive(), "Map feed thread did not terminate")

    def start_threads(self):
        for target in (self.feed._process_maps, self.feed._read):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self.threads.append(thread)

    def send(self, **fields):
        self.lines.put(
            json.dumps(
                dict(
                    session=self.feed.session,
                    frame="world",
                    generated_monotonic_s=self.wall_clock.return_value,
                    **fields,
                )
            )
            + "\n"
        )

    def send_map(self, scans):
        self.send(
            width=20,
            height=20,
            resolution=0.2,
            origin=[0, 0],
            grid="." * 400,
            scans=scans,
            fresh=True,
            stamp_ns=scans * 100_000_000,
            pose=[1, 1, 0, 1],
            hits=np.zeros((120, 2)).tolist(),
            cloud_age_s=0.1,
            clock_age_s=0.1,
            reason="Live map",
        )

    def slow_geometry(self, *args):
        self.entered.set()
        if not self.release.wait(2):
            raise TimeoutError("Test did not release geometry")
        return center_geometry(*args)

    def send_clocks(self):
        for i in range(8):
            self.send(type="clock", clock_ns=i * 50_000_000)
        self.assertTrue(self.clock_read.wait(1), "Geometry blocked clock delivery")

    def test_stalled_geometry_does_not_block_clock_and_only_latest_pending_map_is_built(self):
        with patch("navigation.map_geometry.center_geometry", side_effect=self.slow_geometry):
            self.start_threads()
            self.send_map(1)
            self.assertTrue(self.entered.wait(1))
            # Hold geometry across more than the 250 ms freshness limit.
            self.wall_clock.return_value = 100.35
            for scans in (2, 3, 4):
                self.send_map(scans)
            self.send_clocks()
            # Clock is usable before the geometry operation is allowed to finish.
            self.assertTrue(self.feed.trajectory_clock.fresh(100.35))
            self.assertAlmostEqual(self.feed.trajectory_clock.consume(100.35), 0.2)
            self.assertIsNone(self.feed.snapshot)
            with self.feed.map_condition:
                self.assertEqual(self.feed.pending_map["scans"], 4)
                self.assertEqual(self.feed.maps_superseded, 2)
            self.release.set()
            with self.feed.map_condition:
                self.assertTrue(
                    self.feed.map_condition.wait_for(
                        lambda: self.feed.snapshot is not None and self.feed.snapshot.scans == 4,
                        timeout=1,
                    )
                )
            # Skipping publications does not manufacture three confirmation scans.
            self.assertFalse(self.feed.snapshot.valid)
            self.send_map(5)
            with self.feed.map_condition:
                self.assertTrue(
                    self.feed.map_condition.wait_for(
                        lambda: self.feed.snapshot.scans == 5, timeout=1
                    )
                )
            self.assertTrue(self.feed.snapshot.valid)
            self.assertFalse(self.feed.snapshot.fresh(101.25))
            self.assertFalse(self.feed.trajectory_clock.fresh(100.601))

    def test_eof_during_geometry_latches_failure_and_rejects_late_result(self):
        with patch("navigation.map_geometry.center_geometry", side_effect=self.slow_geometry):
            self.start_threads()
            self.send_map(1)
            self.assertTrue(self.entered.wait(1))
            self.send_clocks()
            self.lines.put("")
            self.threads[1].join(timeout=1)
            self.assertFalse(self.threads[1].is_alive())
            self.assertFalse(self.feed.trajectory_clock.fresh(100))
            self.release.set()
            self.threads[0].join(timeout=1)
            self.assertIsNone(self.feed.snapshot)
            self.assertEqual(self.feed.get()[1], "Map worker output closed")

    def test_geometry_exception_revokes_clock_and_cannot_be_cleared_by_reader(self):
        def fail_geometry(*args):
            self.entered.set()
            self.release.wait(2)
            raise RuntimeError("Synthetic geometry failure")

        with patch("navigation.map_geometry.center_geometry", side_effect=fail_geometry):
            self.start_threads()
            self.send_map(1)
            self.assertTrue(self.entered.wait(1))
            self.send_clocks()
            self.assertTrue(self.feed.trajectory_clock.fresh(100))
            self.release.set()
            self.threads[0].join(timeout=1)
            self.send(type="clock", clock_ns=400_000_000)
            self.threads[1].join(timeout=1)
            self.assertFalse(self.threads[1].is_alive())
            self.assertFalse(self.feed.trajectory_clock.fresh(100))
            self.assertIsNone(self.feed.snapshot)
            self.assertIn("Synthetic geometry failure", self.feed.failure)


if __name__ == "__main__":
    unittest.main()
