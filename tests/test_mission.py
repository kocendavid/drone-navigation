"""Completion proof, map persistence and landing confirmation without flight."""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from navigation.mapping import RayMap, FREE, OCCUPIED, UNKNOWN, START_AREA
from navigation.mission import ExploreMission, assess_completion, save_map


def enclosed():
    model = RayMap(size=20, resolution=0.1, origin=(0, 0))
    model.grid[3:17, 3:17] = FREE
    model.grid[3:17, 3] = model.grid[3:17, 16] = OCCUPIED
    model.grid[3, 3:17] = model.grid[16, 3:17] = OCCUPIED
    return SimpleNamespace(model=model, stamp=1)


class CompletionTests(unittest.TestCase):
    def test_enclosure_allows_unknown_outside_observed_walls(self):
        snap = enclosed()
        evidence = assess_completion(snap, [1, 1])
        self.assertTrue(evidence.complete)
        self.assertGreater(int((snap.model.grid == UNKNOWN).sum()), 0)
        self.assertEqual(evidence.frontier_cells, 0)

    def test_small_unknown_hole_prevents_false_completion(self):
        snap = enclosed()
        snap.model.grid[8, 8] = UNKNOWN
        evidence = assess_completion(snap, [1, 1])
        self.assertFalse(evidence.complete)
        self.assertEqual(evidence.frontier_cells, 1)

    def test_known_space_at_grid_edge_is_not_an_enclosed_environment(self):
        snap = enclosed()
        snap.model.grid[:] = FREE
        evidence = assess_completion(snap, [1, 1])
        self.assertFalse(evidence.complete)
        self.assertTrue(evidence.touches_map_edge)

    def test_assumed_start_area_cannot_prove_observations(self):
        snap = enclosed()
        snap.model.grid[snap.model.grid == FREE] = START_AREA
        self.assertFalse(assess_completion(snap, [1, 1]).complete)

    def test_unknown_doorway_prevents_claiming_hidden_room_is_explored(self):
        snap = enclosed()
        snap.model.grid[3, 8:11] = UNKNOWN
        self.assertFalse(assess_completion(snap, [1, 1]).complete)

    def test_saved_map_keeps_unknown_prior_and_correct_image_y_direction(self):
        snap = enclosed()
        snap.model.grid[10, 10] = START_AREA
        evidence = assess_completion(snap, [1, 1])
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(save_map(snap, Path(directory) / "mission", evidence))
            raw = json.loads((folder / "map.json").read_text())
            self.assertEqual(raw["grid"][10][10], START_AREA)
            image = (folder / "map.pgm").read_bytes().split(b"\n", 3)[3]
            pixels = np.frombuffer(image, dtype=np.uint8).reshape(20, 20)
            self.assertEqual(pixels[19 - 10, 10], 205)
            self.assertEqual(pixels[19 - 4, 4], 254)
            self.assertEqual(pixels[19 - 3, 3], 0)
            self.assertIn("origin: [0.0, 0.0, 0.0]", (folder / "map.yaml").read_text())


class MissionTests(unittest.TestCase):
    def ready_to_land(self):
        mission = ExploreMission()
        mission.start()
        evidence = assess_completion(enclosed(), [1, 1])
        mission.confirm(evidence, 1, 0.0)
        mission.confirm(evidence, 2, 0.5)
        mission.confirm(evidence, 3, 1.0)
        mission.request_landing("saved-map-folder", 10.0)
        return mission

    def test_repeated_scan_and_short_confirmation_period_do_not_land(self):
        mission = ExploreMission()
        mission.start()
        evidence = assess_completion(enclosed(), [1, 1])
        self.assertFalse(mission.confirm(evidence, 1, 0.0))
        self.assertFalse(mission.confirm(evidence, 1, 1.0))
        self.assertFalse(mission.confirm(evidence, 2, 0.1))
        self.assertFalse(mission.confirm(evidence, 3, 0.2))
        self.assertTrue(mission.confirm(evidence, 4, 1.0))

    def test_unknown_remaining_or_cancel_never_requests_landing(self):
        mission = ExploreMission()
        mission.start()
        snap = enclosed()
        snap.model.grid[8, 8] = UNKNOWN
        self.assertFalse(mission.confirm(assess_completion(snap, [1, 1]), 1, 0))
        self.assertEqual(mission.phase, "INCOMPLETE")
        self.assertFalse(mission.landing_due(20))
        mission = self.ready_to_land()
        mission.cancel("Operator hold")
        self.assertFalse(mission.landing_due(10))

    def test_acknowledgement_alone_does_not_prove_landed(self):
        mission = self.ready_to_land()
        mission.acknowledge_land(0)
        self.assertTrue(mission.land_acknowledged)
        self.assertEqual(mission.phase, "LAND_REQUESTED")
        self.assertFalse(
            mission.observe_landing(
                fresh=True, land_mode=False, armed=True, landed=False, landed_fresh=True
            )
        )

    def test_landed_requires_observed_land_mode_and_fresh_disarmed_landed_state(self):
        mission = self.ready_to_land()
        self.assertFalse(
            mission.observe_landing(
                fresh=True,
                land_mode=False,
                armed=False,
                disarmed=True,
                landed=True,
                landed_fresh=True,
            )
        )
        mission.observe_landing(
            fresh=True, land_mode=True, armed=True, landed=False, landed_fresh=True
        )
        mission.observe_landing(
            fresh=True, land_mode=True, armed=False, disarmed=True, landed=True, landed_fresh=False
        )
        self.assertEqual(mission.phase, "LANDING")
        mission.observe_landing(
            fresh=True, land_mode=True, armed=False, disarmed=True, landed=True, landed_fresh=True
        )
        self.assertEqual(mission.phase, "LANDED")

    def test_rejected_or_unconfirmed_land_does_not_report_success(self):
        mission = self.ready_to_land()
        mission.acknowledge_land(2)
        self.assertEqual(mission.phase, "INCOMPLETE")
        self.assertFalse(mission.landing_due(10))
        mission = self.ready_to_land()
        self.assertTrue(mission.landing_due(10))
        self.assertFalse(mission.landing_due(10.5))
        self.assertTrue(mission.landing_due(12))
        self.assertTrue(mission.landing_due(14))
        self.assertFalse(mission.landing_due(16))
        self.assertFalse(mission.landing_due(21))
        self.assertEqual(mission.phase, "INCOMPLETE")


if __name__ == "__main__":
    unittest.main()
