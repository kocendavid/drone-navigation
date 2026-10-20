"""Exploration completion evidence and acknowledged landing lifecycle."""

from collections import deque
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np

from .mapping import FREE, OCCUPIED, START_AREA, UNKNOWN


@dataclass(frozen=True)
class CompletionEvidence:
    complete: bool
    reason: str
    component_cells: int
    frontier_cells: int
    touches_map_edge: bool


def assess_completion(snapshot, start_xy):
    from .native_planning import native_planning

    native = native_planning()
    if native is not None:
        return native.completion(snapshot, start_xy)
    return assess_completion_python(snapshot, start_xy)


def assess_completion_python(snapshot, start_xy):
    """Require an enclosed observed component, not exhaustion of a heuristic.

    This concerns the connected fixed-height space visible from this mission.
    Sealed rooms behind observed walls and other heights are outside this claim.
    The footprint prior remains an explicit assumption, never new coverage.
    """
    model = snapshot.model
    grid = model.grid
    start = model.cell(start_xy)
    known = (grid == FREE) | (grid == START_AREA)
    if not model.inside(start) or not known[start]:
        return CompletionEvidence(False, "Current position is not in known space", 0, 0, False)
    component = np.zeros(grid.shape, dtype=bool)
    queue = deque([start])
    component[start] = True
    boundary_unknown = set()
    edge = False
    while queue:
        y, x = queue.popleft()
        if y in (0, grid.shape[0] - 1) or x in (0, grid.shape[1] - 1):
            edge = True
        for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            other = (y + dy, x + dx)
            if not model.inside(other):
                continue
            if grid[other] == UNKNOWN:
                boundary_unknown.add(other)
            elif known[other] and not component[other]:
                component[other] = True
                queue.append(other)
    count = int(component.sum())
    unresolved = len(boundary_unknown)
    observed = bool(((grid == FREE) & component).any())
    complete = observed and not unresolved and not edge
    reason = (
        "Connected observed space is enclosed; no boundary to unknown remains"
        if complete
        else (
            "Known space reaches the map edge; enclosure is unverified"
            if edge
            else (
                f"{unresolved} unknown boundary cells remain; exploration is incomplete"
                if unresolved
                else "Only the assumed starting footprint is known"
            )
        )
    )
    return CompletionEvidence(complete, reason, count, unresolved, edge)


def save_map(snapshot, folder, evidence):
    """Export raw evidence plus a ROS-compatible PGM/YAML occupancy map."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    grid = snapshot.model.grid
    height, width = grid.shape
    pixels = np.array([205, 254, 0, 205], dtype=np.uint8)[grid]
    (folder / "map.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + np.flipud(pixels).tobytes()
    )
    x, y = map(float, snapshot.model.origin)
    (folder / "map.yaml").write_text(
        "image: map.pgm\nmode: trinary\n"
        f"resolution: {snapshot.model.resolution}\norigin: [{x}, {y}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    (folder / "map.json").write_text(
        json.dumps(
            dict(
                frame="world",
                stamp_ns=snapshot.stamp,
                width=width,
                height=height,
                resolution=snapshot.model.resolution,
                origin=[x, y],
                grid=grid.tolist(),
                confirmed_walls=snapshot.model.confirmed_walls.tolist(),
                wall_policy=snapshot.model.wall_policy(),
                cell_labels=dict(unknown=UNKNOWN, free=FREE, occupied=OCCUPIED, assumed=START_AREA),
                completion=asdict(evidence),
                scope="Connected fixed-height observed space; initial footprint assumed clear",
            ),
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return str(folder)


class ExploreMission:
    def __init__(self):
        self.enabled = False
        self.phase = "IDLE"
        self.reason = "Start Explore and land from a stable Offboard hover"
        self.evidence = None
        self.confirmations = 0
        self.first_confirmation_s = None
        self.last_stamp = None
        self.saved_map = None
        self.request_started = self.last_request = None
        self.land_attempts = 0
        self.land_acknowledged = False

    def start(self):
        self.__init__()
        self.enabled = True
        self.phase = "EXPLORING"
        self.reason = "Exploring; landing requires repeated completion evidence"

    def cancel(self, reason):
        self.enabled = False
        self.phase = "CANCELLED"
        self.reason = reason

    def incomplete(self, reason):
        self.enabled = False
        self.phase = "INCOMPLETE"
        self.reason = reason

    def confirm(self, evidence, stamp, simulated_s):
        self.evidence = evidence
        if not evidence.complete:
            self.incomplete(evidence.reason)
            return False
        self.phase = "VERIFYING"
        self.reason = (
            "Confirming enclosure with three fresh scans over at least one simulated second"
        )
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        if self.first_confirmation_s is None:
            self.first_confirmation_s = simulated_s
        self.confirmations += 1
        if self.confirmations >= 3 and simulated_s - self.first_confirmation_s >= 1.0:
            self.phase = "SAVING"
            self.reason = "Completion verified; saving map before landing"
            return True
        return False

    def reset_confirmation(self):
        self.confirmations = 0
        self.first_confirmation_s = self.last_stamp = None

    def request_landing(self, saved_map, now):
        if not self.enabled or self.phase != "SAVING" or not self.evidence.complete:
            raise ValueError("Landing requires confirmed completion and saved map")
        self.saved_map = saved_map
        self.phase = "LAND_REQUESTED"
        self.reason = "Requesting PX4 landing at current position"
        self.request_started = now

    def acknowledge_land(self, result):
        if self.phase != "LAND_REQUESTED":
            return
        if result == 0:
            self.land_acknowledged = True
            self.reason = "Landing acknowledged; waiting for PX4 Land mode"
        elif result != 5:
            self.incomplete(f"PX4 rejected landing command (result {result}); holding")

    def observe_landing(self, *, fresh, land_mode, armed, landed, landed_fresh, disarmed=False):
        if self.phase not in ("LAND_REQUESTED", "LANDING") or not fresh:
            return False
        if self.phase == "LANDING" and not land_mode and armed:
            self.cancel("PX4 left Land mode; mission landing cancelled")
            return False
        if land_mode:
            self.phase = "LANDING"
            self.reason = "PX4 controls descent; waiting for landed and disarmed reports"
        if self.phase == "LANDING" and disarmed and landed and landed_fresh:
            self.enabled = False
            self.phase = "LANDED"
            self.reason = "Exploration complete; map saved; PX4 reports landed and disarmed"
        return self.phase in ("LANDING", "LANDED")

    def landing_due(self, now):
        if not self.enabled or self.phase != "LAND_REQUESTED":
            return False
        if now - self.request_started > 10.0:
            self.incomplete("PX4 did not enter Land mode within 10 seconds; holding")
            return False
        if self.land_attempts < 3 and (self.last_request is None or now - self.last_request >= 2.0):
            self.land_attempts += 1
            self.last_request = now
            return True
        return False

    def report(self):
        return dict(
            enabled=self.enabled,
            phase=self.phase,
            reason=self.reason,
            completion=asdict(self.evidence) if self.evidence is not None else None,
            confirmed_scans=self.confirmations,
            saved_map=self.saved_map,
            land_attempts=self.land_attempts,
            land_acknowledged=self.land_acknowledged,
            landing_policy="current_position",
        )
