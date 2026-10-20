"""Prepare route footprints in planning workers; audit current masks with NumPy."""

import math
import numpy as np

from .curves import speed_profile, corner_limit, LATERAL_ACCELERATION_SHARE
from .planning import line_cells, segment_safe
from .braking import swept_space_safe


def geometry_key(model):
    return model.grid.shape, model.resolution, tuple(model.origin)


def footprint(model, a, b):
    cells = line_cells(model.cell(a), model.cell(b))
    expanded = list(cells)
    for previous, current in zip(cells, cells[1:]):
        if current[0] != previous[0] and current[1] != previous[1]:
            expanded.extend(((current[0], previous[1]), (previous[0], current[1])))
    return np.asarray(expanded, dtype=np.int32).reshape(-1, 2)


class CommittedPrefix:
    """Capture old-route progress so an asynchronous tail cannot turn us back."""

    def __init__(self, follower, horizon_s):
        self.first_index = follower.index
        self.points = [follower.target.tolist()]
        remaining = max(0.5, float(np.linalg.norm(follower.velocity)) * horizon_s)
        self.length = 0.0
        for index in range(follower.index, len(follower.path)):
            start, end = np.asarray(self.points[-1]), follower.path[index]
            vector = end - start
            length = float(np.linalg.norm(vector))
            self.join_index = index
            self.direction = vector / length if length > 1e-8 else np.zeros(2)
            travel = min(remaining, length)
            self.points.append((start + self.direction * travel).tolist())
            self.length += travel
            remaining -= travel
            if remaining <= 1e-8:
                break
        self.anchor = np.asarray(self.points[-1])

    def rebase(self, path, follower):
        """Trim traversed samples; the generation token certifies route identity."""
        if (
            follower.index < self.first_index
            or follower.index > self.join_index
            or follower.index == self.join_index
            and float((follower.target - self.anchor) @ self.direction) > 1e-8
        ):
            raise ValueError("Vehicle passed committed prefix; evaluating a fresh route")
        return path.with_start(follower.target, first_segment=follower.index - self.first_index)


class PreparedRoute(list):
    def __init__(self, path, model, settings):
        super().__init__([list(map(float, point)) for point in path])
        self.model = model
        self.geometry = geometry_key(model)
        self.settings = settings
        self.segments = [footprint(model, a, b) for a, b in zip(self, self[1:])]
        self._index_cells()
        self.limits = speed_profile(
            [np.asarray(p) for p in self], settings.acceleration_m_s2, settings.cruise_m_s
        )
        self.recenter_improves = None
        self.commitment = None

    def _index_cells(self):
        self.offsets = np.concatenate(([0], np.cumsum([len(c) for c in self.segments])))
        self.cells = np.concatenate(self.segments) if self.segments else np.empty((0, 2), int)

    def with_start(self, start, first_segment=0):
        result = PreparedRoute.__new__(PreparedRoute)
        list.__init__(result, [list(map(float, start)), *self[first_segment + 1 :]])
        result.model, result.geometry, result.settings = self.model, self.geometry, self.settings
        result.segments = [
            footprint(self.model, start, self[first_segment + 1]),
            *self.segments[first_segment + 1 :],
        ]
        result._index_cells()
        # Only the first bend and its upstream braking bound change. The tail
        # was prepared in the planner; this equals a full profile rebuild.
        result.limits = self.limits[first_segment:].copy()
        path = [np.asarray(p) for p in result[:3]]
        cfg = self.settings
        if len(result) > 2:
            result.limits[1] = min(
                corner_limit(path, 1, cfg.acceleration_m_s2, cfg.cruise_m_s),
                math.sqrt(
                    result.limits[2] ** 2
                    + 2
                    * LATERAL_ACCELERATION_SHARE
                    * cfg.acceleration_m_s2
                    * np.linalg.norm(path[2] - path[1])
                ),
            )
        result.limits[0] = min(
            cfg.cruise_m_s,
            math.sqrt(
                result.limits[1] ** 2
                + 2
                * LATERAL_ACCELERATION_SHARE
                * cfg.acceleration_m_s2
                * np.linalg.norm(path[1] - path[0])
            ),
        )
        result.recenter_improves = self.recenter_improves
        result.commitment = None
        return result

    def safe(self, snapshot, first_segment=0):
        if geometry_key(snapshot.model) != self.geometry:
            return False
        cells = self.cells[self.offsets[first_segment] :]
        if not len(cells):
            return True
        y, x = cells.T
        h, w = snapshot.body_free.shape
        return bool(
            np.all((y >= 0) & (y < h) & (x >= 0) & (x < w)) and np.all(snapshot.body_free[y, x])
        )


def route_safe(snapshot, path):
    if isinstance(path, PreparedRoute):
        return path.safe(snapshot)
    return all(
        segment_safe(snapshot.body_free, snapshot.model.cell(a), snapshot.model.cell(b))
        for a, b in zip(path, path[1:])
    )


def remaining_safe(follower, snapshot, pose):
    envelope = getattr(follower, "braking_envelope", None)
    if envelope is not None:
        # The onward route is provisional until the vehicle has stopped. Do not
        # audit a fictitious shortcut from the braking reference to that route.
        return swept_space_safe(snapshot, envelope)
    route = getattr(follower, "prepared_route", None)
    if route is None:
        return route_safe(snapshot, follower.remaining_path(pose))
    if not 1 <= follower.index < len(route):
        return False
    # The moving connector is short; the unchanged suffix uses precomputed cells.
    return (
        route.safe(snapshot, follower.index)
        and segment_safe(
            snapshot.body_free, snapshot.model.cell(pose), snapshot.model.cell(follower.target)
        )
        and segment_safe(
            snapshot.body_free,
            snapshot.model.cell(follower.target),
            snapshot.model.cell(follower.path[follower.index]),
        )
    )
