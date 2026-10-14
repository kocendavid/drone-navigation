"""Choose observed viewpoints near free/unknown frontiers; never route into unknown."""

from dataclasses import dataclass
import math
import numpy as np
from .mapping import FREE, UNKNOWN, OCCUPIED
from .planning import plan, reachable_mask


def frontiers(grid):
    """Observed free cells sharing an edge with unknown, excluding map exterior."""
    unknown = grid == UNKNOWN
    adjacent = np.zeros(grid.shape, dtype=bool)
    adjacent[1:] |= unknown[:-1]
    adjacent[:-1] |= unknown[1:]
    adjacent[:, 1:] |= unknown[:, :-1]
    adjacent[:, :-1] |= unknown[:, 1:]
    return (grid == FREE) & adjacent


def visible_unknown(grid, cell, resolution, radius=3.0):
    """Estimated new area in sparse rays, stopping at known occupied cells.

    Unknown is an information opportunity, not a claim that a future ray can
    pass through it. The actual map still requires real sensor observations.
    """
    angles = np.linspace(0, 2 * math.pi, 48, endpoint=False)
    distances = np.arange(0.0, radius / resolution, 1.0)
    rays = np.rint(
        np.array(cell)[None, None, :]
        + np.stack((np.sin(angles), np.cos(angles)), axis=1)[:, None, :] * distances[None, :, None]
    ).astype(int)
    seen = set()
    h, w = grid.shape
    for ray in rays:
        for y, x in ray:
            if not 0 <= y < h or not 0 <= x < w or grid[y, x] == OCCUPIED:
                break
            if grid[y, x] == UNKNOWN:
                seen.add((int(y), int(x)))
    return len(seen) * resolution**2


@dataclass
class ExplorationChoice:
    goal: list
    path: list
    expected_new_area_m2: float
    route_length_m: float
    frontier_cells: int


def choose_frontier(snapshot, start_xy, visited=()):
    model = snapshot.model
    edges = frontiers(model.grid)
    if not edges.any():
        raise ValueError("No observed free/unknown frontier remains")
    # Stand off so the complete vehicle footprint remains in observed space.
    near = np.zeros(edges.shape, dtype=bool)
    radius = int(math.ceil(0.9 / model.resolution))
    h, w = edges.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if math.hypot(dy, dx) * model.resolution > 0.9:
                continue
            ya, yb = max(0, dy), min(h, h + dy)
            xa, xb = max(0, dx), min(w, w + dx)
            near[ya:yb, xa:xb] |= edges[ya - dy : yb - dy, xa - dx : xb - dx]
    reachable = reachable_mask(snapshot, start_xy)
    candidates = np.argwhere(reachable & near)
    candidates = candidates[(candidates[:, 0] % 3 == 0) & (candidates[:, 1] % 3 == 0)]
    ranked = []
    for cell in candidates:
        goal = model.world(cell)
        distance = math.dist(goal, start_xy)
        if distance < 0.65 or any(math.dist(goal, old) < 0.5 for old in visited):
            continue
        ranked.append((distance, cell, goal))
    ranked.sort(key=lambda item: item[0])
    # Bound selection work outside the heartbeat. Route checks determine
    # reachability; Euclidean distance is only an initial ranking heuristic.
    scored = []
    for distance, cell, goal in ranked[:160]:
        gain = visible_unknown(model.grid, cell, model.resolution)
        if gain >= 0.12:
            scored.append((gain / (0.5 + distance), gain, goal))
    scored.sort(key=lambda item: item[0], reverse=True)
    best = None
    best_score = -math.inf
    for _, gain, goal in scored[:24]:
        try:
            path = plan(snapshot, start_xy, goal)
        except ValueError:
            continue
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        score = gain / (0.5 + length)
        if score > best_score:
            best_score = score
            best = ExplorationChoice(goal.tolist(), path, gain, length, int(edges.sum()))
    if best is None:
        raise ValueError("No useful reachable frontier viewpoint; unknown space may remain")
    return best


class ExplorationSession:
    """Explicit operator-started, bounded session with progress/stall accounting."""

    def __init__(self, max_goals=20):
        self.max_goals = max_goals
        self.enabled = False
        self.phase = "IDLE"
        self.reason = "Explore is operator started"
        self.visited = []
        self.reached = 0
        self.baseline = 0
        self.last_cells = 0
        self.stalls = 0
        self.area_gain_m2 = 0.0
        self.expected_gain_m2 = 0.0
        self.frontier_count = 0
        self.wait_stamp = None

    def start(self, snapshot):
        self.enabled = True
        self.phase = "SELECTING"
        self.reason = "Choosing a reachable frontier viewpoint"
        self.visited = []
        self.reached = 0
        self.stalls = 0
        self.area_gain_m2 = 0.0
        self.wait_stamp = None
        self.baseline = self.last_cells = int(
            ((snapshot.model.grid == FREE) | (snapshot.model.grid == OCCUPIED)).sum()
        )

    def pause(self, reason):
        if self.enabled:
            self.phase = "PAUSED"
            self.reason = reason

    def cancel(self, reason):
        self.enabled = False
        self.phase = "STOPPED"
        self.reason = reason
        self.wait_stamp = None

    def arrived(self, snapshot, goal):
        self.visited.append(list(goal))
        self.reached += 1
        self.phase = "SCANNING"
        self.reason = "Waiting for a new scan at this viewpoint"
        self.wait_stamp = snapshot.stamp

    def scanned(self, snapshot):
        if self.phase != "SCANNING" or snapshot.stamp <= self.wait_stamp:
            return False
        cells = int(((snapshot.model.grid == FREE) | (snapshot.model.grid == OCCUPIED)).sum())
        self.area_gain_m2 = (cells - self.baseline) * snapshot.model.resolution**2
        self.stalls = self.stalls + 1 if cells - self.last_cells < 8 else 0
        self.last_cells = cells
        self.wait_stamp = None
        if self.reached >= self.max_goals or self.stalls >= 3:
            self.enabled = False
            self.phase = "FINISHED"
            self.reason = (
                "Viewpoint limit reached"
                if self.reached >= self.max_goals
                else "Three viewpoints without meaningful new observations"
            )
            return False
        self.phase = "SELECTING"
        self.reason = "Choosing the next frontier viewpoint"
        return True
