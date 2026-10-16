"""Plan through observed space with body clearance and a conservative start escape."""

import heapq
from collections import deque
import math
import numpy as np


NEIGHBOURS = [(dy, dx, math.hypot(dx, dy)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]


def line_cells(a, b):
    dy, dx = b[0] - a[0], b[1] - a[1]
    n = max(abs(dx), abs(dy)) * 2 + 1
    points = np.rint(np.linspace(a, b, n)).astype(int)
    return list(dict.fromkeys(map(tuple, points.tolist())))


def segment_safe(mask, a, b):
    previous = None
    for y, x in line_cells(a, b):
        if not 0 <= y < mask.shape[0] or not 0 <= x < mask.shape[1] or not mask[y, x]:
            return False
        if previous is not None and y != previous[0] and x != previous[1]:
            if not mask[y, previous[1]] or not mask[previous[0], x]:
                return False
        previous = (y, x)
    return True


def astar(mask, start, goal):
    if not mask[start] or not mask[goal]:
        raise ValueError("Start or destination is outside the traversable map")
    queue = [(math.dist(start, goal), 0.0, start)]
    cost = {start: 0.0}
    parent = {}
    while queue:
        _, g, cell = heapq.heappop(queue)
        if g != cost.get(cell):
            continue
        if cell == goal:
            path = [cell]
            while cell in parent:
                cell = parent[cell]
                path.append(cell)
            return path[::-1]
        for dy, dx, distance in NEIGHBOURS:
            nxt = (cell[0] + dy, cell[1] + dx)
            if not 0 <= nxt[0] < mask.shape[0] or not 0 <= nxt[1] < mask.shape[1] or not mask[nxt]:
                continue
            if dy and dx and (not mask[cell[0] + dy, cell[1]] or not mask[cell[0], cell[1] + dx]):
                continue
            new = g + distance
            if new < cost.get(nxt, math.inf):
                cost[nxt] = new
                parent[nxt] = cell
                heapq.heappush(queue, (new + math.dist(nxt, goal), new, nxt))
    raise ValueError("No route through currently observed free space")


def escape_start(snapshot, start):
    """Find the same monotone observed-space escape used by all route searches."""
    model = snapshot.model
    escape = [start]
    if not snapshot.safe[start]:
        # A hover can begin closer to a wall than the preferred navigation
        # margin. Escape only through observed body-free cells, with obstacle
        # clearance nondecreasing; never tunnel through an inflated obstacle.
        if not snapshot.body_free[start]:
            raise ValueError("Current vehicle footprint is not sufficiently observed")
        queue = [(0.0, start)]
        costs = {start: 0.0}
        parent = {}
        exit_cell = None
        while queue:
            g, cell = heapq.heappop(queue)
            if g != costs[cell]:
                continue
            if snapshot.safe[cell]:
                exit_cell = cell
                break
            for dy, dx, d in NEIGHBOURS:
                nxt = (cell[0] + dy, cell[1] + dx)
                if not model.inside(nxt) or not snapshot.body_free[nxt]:
                    continue
                if (
                    dy
                    and dx
                    and (
                        not snapshot.body_free[cell[0] + dy, cell[1]]
                        or not snapshot.body_free[cell[0], cell[1] + dx]
                    )
                ):
                    continue
                if snapshot.clearance[nxt] + 0.02 < snapshot.clearance[cell]:
                    continue
                new = g + d
                if new * model.resolution <= 2.0 and new < costs.get(nxt, math.inf):
                    costs[nxt] = new
                    parent[nxt] = cell
                    heapq.heappush(queue, (new, nxt))
        if exit_cell is None:
            raise ValueError("No observed route away from the current obstacle margin")
        escape = [exit_cell]
        cell = exit_cell
        while cell in parent:
            cell = parent[cell]
            escape.append(cell)
        escape.reverse()
    return escape


def reachable_mask(snapshot, start_xy):
    """Connected preferred-clearance cells, with identical diagonal constraints."""
    start = snapshot.model.cell(start_xy)
    if not snapshot.model.inside(start):
        raise ValueError("Vehicle is outside the map")
    seed = escape_start(snapshot, start)[-1]
    connected = np.zeros(snapshot.safe.shape, dtype=bool)
    connected[seed] = True
    queue = deque([seed])
    mask = snapshot.safe
    while queue:
        cell = queue.popleft()
        for dy, dx, _ in NEIGHBOURS:
            nxt = (cell[0] + dy, cell[1] + dx)
            if not snapshot.model.inside(nxt) or connected[nxt] or not mask[nxt]:
                continue
            if dy and dx and (not mask[cell[0] + dy, cell[1]] or not mask[cell[0], cell[1] + dx]):
                continue
            connected[nxt] = True
            queue.append(nxt)
    return connected


def route_distances(snapshot, start_xy):
    """Shortest grid travel distance through known safe space from the vehicle."""
    model = snapshot.model
    start = model.cell(start_xy)
    if not model.inside(start):
        raise ValueError("Current position is outside the map")
    escape = escape_start(snapshot, start)
    distances = np.full(model.grid.shape, np.inf)
    initial = sum(math.dist(a, b) for a, b in zip(escape, escape[1:])) * model.resolution
    distances[escape[-1]] = initial
    queue = [(initial, escape[-1])]
    while queue:
        cost, cell = heapq.heappop(queue)
        if cost != distances[cell]:
            continue
        for dy, dx, step in NEIGHBOURS:
            other = (cell[0] + dy, cell[1] + dx)
            if not model.inside(other) or not snapshot.safe[other]:
                continue
            if (
                dy
                and dx
                and (
                    not snapshot.safe[cell[0] + dy, cell[1]]
                    or not snapshot.safe[cell[0], cell[1] + dx]
                )
            ):
                continue
            new = cost + step * model.resolution
            if new < distances[other]:
                distances[other] = new
                heapq.heappush(queue, (new, other))
    return distances


def plan(snapshot, start_xy, goal_xy):
    model = snapshot.model
    start, goal = model.cell(start_xy), model.cell(goal_xy)
    if not model.inside(start) or not model.inside(goal):
        raise ValueError("Destination is outside the map")
    if not snapshot.safe[goal]:
        raise ValueError("Choose an observed destination outside the obstacle margin")
    escape = escape_start(snapshot, start)
    start = escape[-1]
    route = astar(snapshot.safe, start, goal)
    # Shortcuts must preserve inflated obstacles and unknown-space exclusions.
    smooth = [route[0]]
    i = 0
    while i < len(route) - 1:
        j = len(route) - 1
        while j > i + 1 and not segment_safe(snapshot.safe, route[i], route[j]):
            j -= 1
        smooth.append(route[j])
        i = j
    cells = escape[:-1] + smooth
    world = [model.world(cell).tolist() for cell in cells]
    world[0] = list(map(float, start_xy))
    if len(world) == 1:
        world.append(list(map(float, goal_xy)))
    else:
        world[-1] = list(map(float, goal_xy))
    if not all(
        segment_safe(snapshot.body_free, model.cell(a), model.cell(b))
        for a, b in zip(world, world[1:])
    ):
        raise ValueError("Route shortcut crosses insufficiently observed space")
    return world
