"""Native searches preserve Python reachability, costs and completion evidence."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from navigation.mapping import RayMap
from navigation.mission import assess_completion, assess_completion_python
from navigation.planning import astar, astar_python, path_cost, segment_safe
from navigation.native_planning import NativePlanning, native_planning, ROOT


@unittest.skipUnless(
    (ROOT / ".tools/native_navigation/current_build.json").exists(), "Build native navigation first"
)
class NativePlanningTests(unittest.TestCase):
    def test_large_maps_have_no_benchmark_fixture_size_limit(self):
        model = RayMap(size=400)
        model.grid[:] = 1
        self.assertEqual(len(astar(model.grid == 1, (0, 0), (399, 399))), 400)
        evidence = assess_completion(SimpleNamespace(model=model), model.world((200, 200)))
        self.assertTrue(evidence.touches_map_edge)
        self.assertEqual(evidence.component_cells, 160000)

    def test_dispatch_uses_cpp_and_existing_broken_builds_are_not_silently_ignored(self):
        self.assertIsInstance(native_planning(), NativePlanning)
        with patch("navigation.planning.astar_python", side_effect=AssertionError("Python search")):
            self.assertEqual(astar(np.ones((3, 3), bool), (0, 0), (2, 2)), [(0, 0), (1, 1), (2, 2)])
        model = RayMap(size=4)
        model.grid[:] = 1
        snap = SimpleNamespace(model=model)
        with patch(
            "navigation.mission.assess_completion_python",
            side_effect=AssertionError("Python flood fill"),
        ):
            self.assertTrue(assess_completion(snap, model.world((1, 1))).touches_map_edge)
        with patch("navigation.native_planning.source_hash", return_value="stale"):
            with self.assertRaisesRegex(RuntimeError, "stale"):
                NativePlanning()

    def test_weighted_search_matches_reference_on_random_obstacles_and_costs(self):
        rng = np.random.default_rng(20261020)
        for _ in range(100):
            mask = rng.random((19, 23)) > 0.28
            mask[1, 1] = mask[17, 21] = True
            cost = 1 + rng.random(mask.shape) * 8
            try:
                expected = astar_python(mask, (1, 1), (17, 21), cost)
            except ValueError:
                with self.assertRaises(ValueError):
                    astar(mask, (1, 1), (17, 21), cost)
                continue
            actual = astar(mask, (1, 1), (17, 21), cost)
            self.assertAlmostEqual(path_cost(actual, cost), path_cost(expected, cost), places=9)
            self.assertTrue(all(segment_safe(mask, a, b) for a, b in zip(actual, actual[1:])))

    def test_corner_cutting_invalid_costs_and_invalid_endpoints_are_rejected(self):
        mask = np.array([[True, False], [False, True]])
        with self.assertRaises(ValueError):
            astar(mask, (0, 0), (1, 1))
        for start, goal in (((-1, 0), (1, 1)), ((0, 0), (2, 1)), ((0, 1), (1, 1))):
            with self.assertRaises(ValueError):
                astar(mask, start, goal)
        mask[:] = True
        for cost in (np.zeros((2, 2)), np.full((2, 2), np.nan), np.ones((3, 3))):
            with self.assertRaises(ValueError):
                astar(mask, (0, 0), (1, 1), cost)

    def test_completion_matches_reference_across_unknown_prior_edges_and_components(self):
        rng = np.random.default_rng(491)
        model = RayMap(size=(17, 21), origin=(-2, -3))
        snap = SimpleNamespace(model=model)
        for _ in range(100):
            model.grid = rng.integers(0, 4, size=model.grid.shape, dtype=np.uint8)
            point = model.world((int(rng.integers(17)), int(rng.integers(21))))
            self.assertEqual(assess_completion(snap, point), assess_completion_python(snap, point))
        for label in (0, 1, 2, 3):
            model.grid[:] = label
            self.assertEqual(
                assess_completion(snap, model.world((1, 1))),
                assess_completion_python(snap, model.world((1, 1))),
            )
        self.assertEqual(
            assess_completion(snap, [100, 100]), assess_completion_python(snap, [100, 100])
        )

    def test_missing_build_keeps_offline_reference_available(self):
        try:
            with (
                tempfile.TemporaryDirectory() as folder,
                patch("navigation.native_planning.ROOT", Path(folder)),
            ):
                native_planning.cache_clear()
                self.assertIsNone(native_planning())
                self.assertEqual(astar(np.ones((2, 2), bool), (0, 0), (1, 1)), [(0, 0), (1, 1)])
        finally:
            native_planning.cache_clear()
