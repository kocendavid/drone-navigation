"""Choose observed viewpoints near free/unknown frontiers; never route into unknown."""

from dataclasses import dataclass
import math
import numpy as np
from .mapping import FREE, UNKNOWN, OCCUPIED
from .planning import plan, route_distances
from .frontier_ledger import FrontierLedger, boundary_geometry, MINIMUM_SPAN_M


def frontiers(grid):
    """Observed free cells sharing an edge with unknown, excluding map exterior."""
    unknown = grid == UNKNOWN
    adjacent = np.zeros(grid.shape, dtype=bool)
    adjacent[1:] |= unknown[:-1]
    adjacent[:-1] |= unknown[1:]
    adjacent[:, 1:] |= unknown[:, :-1]
    adjacent[:, :-1] |= unknown[:, 1:]
    return (grid == FREE) & adjacent


def visible_unknown_cells(grid, cell, resolution, radius=3.0):
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
    return seen


def visible_unknown(grid, cell, resolution, radius=3.0):
    return len(visible_unknown_cells(grid, cell, resolution, radius)) * resolution**2


def wall_endings(grid, resolution, confirmed_walls):
    """Conservative occlusion-corner clues from ends of confirmed wall segments.

    Fit a local line in either orientation. Require a one-sided wall at least
    0.6 m long, nearby free space and unknown beyond its tip. These are viewpoint
    hints, never evidence that the hidden space is physically traversable.
    """
    occupied = (grid == OCCUPIED) & confirmed_walls
    adjacent_free = np.zeros(grid.shape, dtype=bool)
    adjacent_unknown = np.zeros(grid.shape, dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            ya, yb = max(0, dy), min(grid.shape[0], grid.shape[0] + dy)
            xa, xb = max(0, dx), min(grid.shape[1], grid.shape[1] + dx)
            neighbour = grid[ya - dy : yb - dy, xa - dx : xb - dx]
            adjacent_free[ya:yb, xa:xb] |= neighbour == FREE
            adjacent_unknown[ya:yb, xa:xb] |= neighbour == UNKNOWN
    radius = max(2, int(math.ceil(0.8 / resolution)))
    tips = []
    for cell in np.argwhere(occupied & adjacent_free & adjacent_unknown):
        low = np.maximum(0, cell - radius)
        high = np.minimum(grid.shape, cell + radius + 1)
        points = np.argwhere(occupied[low[0] : high[0], low[1] : high[1]]) + low - cell
        points = points[np.linalg.norm(points, axis=1) * resolution <= 0.8] * resolution
        if len(points) < 4:
            continue
        eigenvalues, axes = np.linalg.eigh(np.cov(points.T))
        if eigenvalues[-1] < 3 * max(eigenvalues[0], resolution**2 / 12):
            continue
        direction = axes[:, -1]
        projection = points @ direction
        if projection.min() < -0.25 and projection.max() > 0.25:
            continue  # Middle of a wall, rather than its observed end.
        if np.ptp(projection) < 0.6:
            continue
        if points.mean(axis=0) @ direction > 0:
            direction = -direction
        # A clear doorway can already be observed beyond the wall tip while
        # the room beside its far face is still occluded. Check both flanks.
        perpendicular = np.array([-direction[1], direction[0]])
        offsets = np.array(
            [
                direction * forward + perpendicular * sideways
                for forward in (-0.8, -0.4, 0.0, 0.4, 0.8)
                for sideways in (-0.8, 0.0, 0.8)
            ]
        )
        probes = np.rint(cell + offsets / resolution).astype(int)
        valid = (probes >= 0).all(axis=1) & (probes < grid.shape).all(axis=1)
        if not valid.all() or np.count_nonzero(grid[tuple(probes.T)] == UNKNOWN) < 2:
            continue
        if any(np.linalg.norm(cell - old[0]) * resolution < 0.8 for old in tips):
            continue
        tips.append((cell.copy(), direction, float(np.ptp(projection))))
    return tips


@dataclass
class ExplorationChoice:
    goal: list
    path: list
    expected_new_area_m2: float
    route_length_m: float
    frontier_cells: int
    history_distance_m: float | None = None
    selection_reason: str = "Reachable frontier viewpoint"
    corner_xy: list | None = None
    hidden_area_m2: float = 0.0
    wall_endings_xy: list | None = None
    frontier_id: int | None = None
    frontier_assessments: list | None = None


def wall_occluded_cells(grid, start, cells):
    """Which proposed unknown cells have a known wall on their current sight line?

    Chunk the ray sampling to keep selection memory bounded on the maze map.
    This estimates sensing opportunity, not collision clearance for a route.
    """
    result = set()
    points = np.array(sorted(cells), dtype=int).reshape(-1, 2)
    for chunk in np.array_split(points, max(1, math.ceil(len(points) / 256))):
        if not len(chunk):
            continue
        delta = chunk - start
        steps = np.maximum(1, np.abs(delta).max(axis=1))
        sample = np.arange(int(steps.max()) + 1)
        rays = np.rint(
            start + delta[:, None, :] * np.minimum(sample / steps[:, None], 1)[..., None]
        ).astype(int)
        blocked = (grid[rays[:, :, 0], rays[:, :, 1]] == OCCUPIED).any(axis=1)
        result.update(map(tuple, chunk[blocked]))
    return result


def choose_frontier(
    snapshot, start_xy, visited=(), trail=(), *, catalog=None, active_id=None, current_goal=None
):
    """Nearest reachable frontier by actual center-guided route length.

    Cluster IDs come from the session ledger. Visiting a viewpoint does not
    resolve a boundary. Keep a useful active frontier before choosing the next.
    """
    model = snapshot.model
    edges = frontiers(model.grid)
    if not edges.any():
        raise ValueError("No observed free/unknown frontier remains")
    if catalog is None:
        ledger = FrontierLedger()
        ledger.update(snapshot, edges)
        catalog = ledger.snapshot()
    distances = route_distances(snapshot, start_xy)
    radius = int(math.ceil(0.9 / model.resolution))
    offsets = [
        (dy, dx, dy * dy + dx * dx)
        for dy in range(-radius, radius + 1)
        for dx in range(-radius, radius + 1)
        if math.hypot(dy, dx) * model.resolution <= 0.9
    ]
    owners = {}
    for region in catalog:
        if boundary_geometry(region["cells"], model.resolution)[0] < MINIMUM_SPAN_M:
            continue
        for y, x in region["cells"]:
            for dy, dx, squared in offsets:
                cell = (y + dy, x + dx)
                if (
                    cell[0] % 3
                    or cell[1] % 3
                    or not model.inside(cell)
                    or not snapshot.safe[cell]
                    or not math.isfinite(distances[cell])
                ):
                    continue
                value = (squared, region["id"])
                if value < owners.get(cell, (math.inf, math.inf)):
                    owners[cell] = value
    candidates = {row["id"]: [] for row in catalog}
    history = np.asarray(trail if len(trail) else visited, dtype=float).reshape(-1, 2)
    for cell, (_, key) in owners.items():
        goal = model.world(cell)
        if math.dist(goal, start_xy) < 0.65 or any(math.dist(goal, old) < 0.5 for old in visited):
            continue
        candidates[key].append((float(distances[cell]), cell, goal))
    assessments = []
    options = []
    for region in catalog:
        key = region["id"]
        span, midpoint, axis = boundary_geometry(region["cells"], model.resolution)
        if span < MINIMUM_SPAN_M:
            assessments.append(
                dict(
                    id=key,
                    status="filtered",
                    span_m=span,
                    route_length_m=None,
                    viewpoint_xy=None,
                    reason=f"Boundary span {span:.2f} m below {MINIMUM_SPAN_M:.1f} m; unresolved",
                )
            )
            continue
        # Aim near the middle of this boundary, not whichever endpoint is
        # nearest the drone. Frontier selection still compares flight lengths.
        ranked = sorted(
            candidates[key],
            key=lambda r: (
                abs(float((np.asarray(r[1]) - midpoint) @ axis)),
                -snapshot.clearance[r[1]],
                r[0],
            ),
        )
        trials = []
        # Retain a useful destination within the active boundary instead of
        # repeatedly moving its goal closer as the drone approaches it.
        if key == active_id and current_goal is not None:
            cell = model.cell(current_goal)
            if (
                model.inside(cell)
                and snapshot.safe[cell]
                and any(math.dist(cell, edge) * model.resolution <= 1.2 for edge in region["cells"])
                and not any(math.dist(current_goal, old) < 0.5 for old in visited)
            ):
                trials.append((np.array(current_goal), True))
        trials.extend((goal, False) for _, _, goal in ranked[:12])
        best = None
        useful = 0
        for goal, retained in trials:
            cell = model.cell(goal)
            gain = visible_unknown(model.grid, cell, model.resolution)
            if gain < 0.12:
                continue
            useful += 1
            try:
                path = plan(snapshot, start_xy, goal)
            except ValueError:
                continue
            length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
            separation = (
                float(np.linalg.norm(history - goal, axis=1).min()) if len(history) else None
            )
            row = dict(
                id=key, goal=goal, path=path, length=length, gain=gain, separation=separation
            )
            if retained:
                best = row
                break
            center_error = abs(float((np.asarray(cell) - midpoint) @ axis)) * model.resolution
            row["center_error"] = center_error
            if best is None or (center_error, length) < (best["center_error"], best["length"]):
                best = row
            if useful >= 3:
                break
        if best is None:
            assessments.append(
                dict(
                    id=key,
                    status="deferred" if ranked else "unreachable",
                    route_length_m=None,
                    viewpoint_xy=None,
                    reason="No useful unvisited reachable viewpoint; boundary remains unresolved",
                )
            )
        else:
            options.append(best)
            assessments.append(
                dict(
                    id=key,
                    status="pending",
                    route_length_m=best["length"],
                    viewpoint_xy=best["goal"].tolist(),
                    reason="Reachable route through observed free space",
                )
            )
    if not options:
        raise ValueError(
            f"No useful reachable frontier with span at least {MINIMUM_SPAN_M:.1f} m; unknown space may remain"
        )
    selected = min(options, key=lambda r: (r["id"] != active_id, r["length"], -r["gain"], r["id"]))
    tips = wall_endings(model.grid, model.resolution, model.confirmed_walls)
    corner = None
    hidden = 0.0
    seen = visible_unknown_cells(model.grid, model.cell(selected["goal"]), model.resolution)
    if tips:
        occluded = wall_occluded_cells(model.grid, np.array(model.cell(start_xy)), seen)
        for tip, _, _ in tips:
            if np.linalg.norm(np.array(model.cell(selected["goal"])) - tip) * model.resolution > 3:
                continue
            area = (
                sum(
                    np.linalg.norm(np.array(cell) - tip) * model.resolution <= 3
                    for cell in occluded
                )
                * model.resolution**2
            )
            if area >= 0.12 and area > hidden:
                corner, hidden = model.world(tip).tolist(), float(area)
    retained = selected["id"] == active_id
    return ExplorationChoice(
        selected["goal"].tolist(),
        selected["path"],
        selected["gain"],
        selected["length"],
        int(edges.sum()),
        selected["separation"],
        f"{'Continue active' if retained else 'Nearest reachable'} frontier F{selected['id']}: "
        f"{selected['length']:.1f} m planned flight route; estimated gain {selected['gain']:.2f} m²",
        corner,
        hidden,
        [model.world(tip).tolist() for tip, _, _ in tips],
        selected["id"],
        assessments,
    )


class ExplorationSession:
    """Explicit operator-started, bounded session with progress/stall accounting."""

    def __init__(self, max_goals=20):
        self.max_goals = max_goals
        self.enabled = False
        self.phase = "IDLE"
        self.reason = "Explore is operator started"
        self.visited = []
        self.trail = []
        self.reached = 0
        self.baseline = 0
        self.last_cells = 0
        self.stalls = 0
        self.area_gain_m2 = 0.0
        self.expected_gain_m2 = 0.0
        self.frontier_count = 0
        self.selection_reason = "No viewpoint chosen yet"
        self.wait_stamp = None
        self.continuous = False
        self.reconsiderations = 0
        self.target_switches = 0
        self.current_target_gain_m2 = None
        self.distance_flown_m = 0.0
        self.measurement_pose = None
        self.started_s = None
        self.elapsed_s = 0.0
        self.reconsider_reason = "Waiting for exploration"
        self.corner_xy = None
        self.hidden_area_m2 = 0.0
        self.wall_endings_xy = []
        self.frontier_ledger = FrontierLedger()

    def start(self, snapshot, *, continuous=False, simulation_s=0.0):
        # Re-starting Explore in the same map retains unresolved work and IDs.
        self._observe_frontiers(snapshot)
        self.enabled = True
        self.phase = "SELECTING"
        self.reason = "Choosing a reachable frontier viewpoint"
        # Viewpoint memory belongs to this experiment, including Explore after Hold.
        if not self.trail:
            self.trail = [list(snapshot.pose[:2])]
        self.reached = 0
        self.stalls = 0
        self.area_gain_m2 = 0.0
        self.wait_stamp = None
        self.baseline = self.last_cells = int(
            ((snapshot.model.grid == FREE) | (snapshot.model.grid == OCCUPIED)).sum()
        )
        self.continuous = continuous
        self.reconsiderations = self.target_switches = 0
        self.current_target_gain_m2 = None
        self.corner_xy = None
        self.hidden_area_m2 = 0.0
        self.wall_endings_xy = []
        self.distance_flown_m = 0.0
        self.measurement_pose = list(snapshot.pose[:2])
        self.started_s = simulation_s
        self.elapsed_s = 0.0
        self.reconsider_reason = (
            "Targets update during travel" if continuous else "Targets update after arrival"
        )

    def _observe_frontiers(self, snapshot):
        if hasattr(self.frontier_ledger, "observe_map"):
            return self.frontier_ledger.observe_map(snapshot)
        return self.frontier_ledger.update(snapshot, frontiers(snapshot.model.grid))

    def observe(self, snapshot, simulation_s):
        changes = self._observe_frontiers(snapshot)
        if self.enabled:
            cells = int(((snapshot.model.grid == FREE) | (snapshot.model.grid == OCCUPIED)).sum())
            self.area_gain_m2 = (cells - self.baseline) * snapshot.model.resolution**2
            if self.started_s is not None:
                self.elapsed_s = max(0.0, simulation_s - self.started_s)
        return changes

    def pause(self, reason):
        if self.enabled:
            self.phase = "PAUSED"
            self.reason = reason

    def record_pose(self, xy):
        if self.enabled and self.measurement_pose is not None:
            distance = math.dist(xy, self.measurement_pose)
            if distance >= 0.02:
                self.distance_flown_m += distance
                self.measurement_pose = list(map(float, xy))
        if self.enabled and (not self.trail or math.dist(xy, self.trail[-1]) >= 0.2):
            self.trail.append(list(map(float, xy)))

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
