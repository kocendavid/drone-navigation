"""Persistent frontier lifecycle and nearest-route choice through occluded rooms."""

import math
import time
import unittest
from unittest.mock import patch
import numpy as np
from navigation.frontier_ledger import FrontierLedger
from navigation.exploration import frontiers, choose_frontier, ExplorationSession
from navigation.mapping import RayMap, FREE, OCCUPIED, UNKNOWN
from navigation.feed import MapSnapshot


def maze():
    model = RayMap(resolution=0.2, size=(50, 80), origin=(0, 0), body_radius=0.2, clearance=0.6)
    model.grid[:] = OCCUPIED
    model.grid[10:20, 10:65] = FREE
    model.grid[25:35, 10:65] = FREE
    model.grid[10:35, 55:65] = FREE  # The only way around the separating wall.
    model.grid[25:35, 5:10] = UNKNOWN  # Screen-near, flight-far lower frontier.
    model.grid[5:10, 35:45] = UNKNOWN  # Screen-far, flight-near upper frontier.
    clearance, body, safe = model.geometry()
    now = time.monotonic()
    return MapSnapshot(
        model=model,
        clearance=clearance,
        body_free=body,
        safe=safe,
        valid=True,
        stamp=1,
        pose=[3.1, 3.1, 0, 0],
        generated=now,
        cloud_wall=now,
        clock_wall=now,
        reason="Live",
        scans=1,
        hits=np.empty((0, 2)),
    )


class FrontierLedgerTests(unittest.TestCase):
    def test_active_frontier_does_not_plan_alternatives_that_cannot_win(self):
        import navigation.exploration as exploration

        snap = maze()
        ledger = FrontierLedger()
        ledger.update(snap, frontiers(snap.model.grid))
        active = max(ledger.records, key=lambda key: ledger.records[key]["boundary_xy"][1])
        first = choose_frontier(snap, snap.pose[:2], catalog=ledger.snapshot(), active_id=active)
        with patch.object(exploration, "plan", wraps=exploration.plan) as planner:
            kept = choose_frontier(
                snap,
                snap.pose[:2],
                catalog=ledger.snapshot(),
                active_id=active,
                current_goal=first.goal,
            )
        self.assertEqual(kept.goal, first.goal)
        self.assertEqual(planner.call_count, 1)
        self.assertEqual([row["id"] for row in kept.frontier_assessments], [active])

    def test_unusable_active_frontier_still_falls_back_to_other_work(self):
        snap = maze()
        ledger = FrontierLedger()
        ledger.update(snap, frontiers(snap.model.grid))
        active = max(ledger.records, key=lambda key: ledger.records[key]["boundary_xy"][1])
        for cell in ledger.records[active]["cells"]:
            snap.model.grid[cell] = OCCUPIED
        snap.clearance, snap.body_free, snap.safe = snap.model.geometry()
        choice = choose_frontier(snap, snap.pose[:2], catalog=ledger.snapshot(), active_id=active)
        self.assertNotEqual(choice.frontier_id, active)

    def test_small_boundaries_are_filtered_not_resolved_and_can_grow_eligible(self):
        snap = maze()
        ledger = FrontierLedger()
        mask = np.zeros_like(snap.model.grid, bool)
        mask[12, 15:22] = True  # 1.4 m.
        ledger.update(snap, mask)
        row = next(iter(ledger.records.values()))
        self.assertEqual(row["status"], "filtered")
        self.assertEqual(ledger.report()["resolved"], 0)
        self.assertEqual(ledger.report()["pending"], 1)
        snap.stamp += 1
        mask[12, 22] = True  # 1.6 m; same frontier, newly eligible.
        ledger.update(snap, mask)
        self.assertEqual(row["status"], "pending")
        self.assertAlmostEqual(row["span_m"], 1.6)

    def test_selected_viewpoint_is_near_boundary_middle_not_nearest_endpoint(self):
        snap = maze()
        choice = choose_frontier(snap, snap.pose[:2])
        ledger = FrontierLedger()
        ledger.update(snap, frontiers(snap.model.grid))
        row = ledger.records[choice.frontier_id]
        axis = np.asarray(row["axis_xy"])
        self.assertLess(abs((np.asarray(choice.goal) - row["midpoint_xy"]) @ axis), 0.7)

    def test_wall_detour_beats_pixel_proximity_and_distances_are_actual_planned_lengths(self):
        snap = maze()
        start = snap.pose[:2]
        choice = choose_frontier(snap, start)
        self.assertLess(choice.goal[1], 4)
        self.assertEqual(len(choice.frontier_assessments), 2)
        other = next(row for row in choice.frontier_assessments if row["id"] != choice.frontier_id)
        self.assertLess(math.dist(start, other["viewpoint_xy"]), math.dist(start, choice.goal))
        self.assertGreater(other["route_length_m"], choice.route_length_m + 10)
        self.assertAlmostEqual(
            choice.route_length_m,
            sum(math.dist(a, b) for a, b in zip(choice.path, choice.path[1:])),
        )

    def test_active_frontier_stays_selected_and_a_useful_goal_does_not_jump(self):
        snap = maze()
        ledger = FrontierLedger()
        ledger.update(snap, frontiers(snap.model.grid))
        far_id = max(ledger.records, key=lambda key: ledger.records[key]["boundary_xy"][1])
        choice = choose_frontier(snap, snap.pose[:2], catalog=ledger.snapshot(), active_id=far_id)
        self.assertEqual(choice.frontier_id, far_id)
        retained = choose_frontier(
            snap,
            snap.pose[:2],
            catalog=ledger.snapshot(),
            active_id=far_id,
            current_goal=choice.goal,
        )
        self.assertEqual(retained.goal, choice.goal)
        self.assertIn("Continue active", retained.selection_reason)

    def test_arrival_does_not_resolve_frontier_only_three_new_valid_scans_do(self):
        snap = maze()
        session = ExplorationSession()
        session.start(snap)
        ledger = session.frontier_ledger
        choice = choose_frontier(snap, snap.pose[:2], catalog=ledger.snapshot())
        ledger.apply_choice(choice)
        key = choice.frontier_id
        session.arrived(snap, choice.goal)
        self.assertEqual(ledger.records[key]["status"], "active")
        snap.model.grid[snap.model.grid == UNKNOWN] = OCCUPIED
        session.observe(snap, 1)  # Same scan cannot change lifecycle.
        self.assertEqual(ledger.records[key]["missing_scans"], 0)
        for i in range(3):
            snap.stamp += 1
            session.observe(snap, 2 + i)
            self.assertEqual(ledger.records[key]["status"] == "resolved", i == 2)
        self.assertIsNone(ledger.active_id)
        self.assertEqual(ledger.report()["pending"], 0)
        self.assertEqual(ledger.report()["resolved"], 2)

    def test_invalid_maps_cannot_resolve_unfinished_boundaries(self):
        snap = maze()
        ledger = FrontierLedger()
        mask = frontiers(snap.model.grid)
        ledger.update(snap, mask)
        snap.valid = False
        for _ in range(4):
            snap.stamp += 1
            ledger.update(snap, np.zeros_like(mask))
        self.assertEqual(ledger.report()["resolved"], 0)

    def test_frontier_merge_preserves_work_instead_of_marking_it_explored(self):
        snap = maze()
        ledger = FrontierLedger()
        mask = np.zeros_like(snap.model.grid, bool)
        mask[10, 10:13] = True
        mask[10, 15:18] = True
        ledger.update(snap, mask)
        first = set(ledger.records)
        snap.stamp += 1
        mask[10, 10:18] = True
        ledger.update(snap, mask)
        merged = [r for r in ledger.records.values() if r["status"] == "merged"]
        self.assertEqual(len(merged), 1)
        self.assertIn(merged[0]["merged_into"], first)
        self.assertEqual(ledger.report()["resolved"], 0)
        self.assertEqual(ledger.report()["pending"], 1)

    def test_hold_and_explore_preserve_catalog_and_visited_viewpoints_reset_creates_empty(self):
        snap = maze()
        session = ExplorationSession()
        session.start(snap)
        ids = set(session.frontier_ledger.records)
        session.arrived(snap, [8, 3])
        session.cancel("Hold")
        session.start(snap)
        self.assertEqual(set(session.frontier_ledger.records), ids)
        self.assertIn([8, 3], session.visited)
        self.assertEqual(ExplorationSession().frontier_ledger.report()["frontiers"], [])
