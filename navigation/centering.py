"""Known-space center clues and bounded clearance costs for route planning."""

import numpy as np
from .mapping import distance_field, FREE, START_AREA


def center_geometry(model, wall_clearance, safe):
    """Distance ridges inside known space; unknown is a boundary, never a wall hit.

    The resulting center is provisional: it moves as more space is observed.
    It is not the centroid of a room, which could lie inside an obstacle.
    """
    known = (model.grid == FREE) | (model.grid == START_AREA)
    clearance = np.minimum(wall_clearance, distance_field(~known, model.resolution))
    padded = np.pad(clearance, 1, constant_values=0)
    h, w = clearance.shape
    ridge = np.zeros(clearance.shape, dtype=bool)
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = padded[1 + dy : 1 + dy + h, 1 + dx : 1 + dx + w]
        b = padded[1 - dy : 1 - dy + h, 1 - dx : 1 - dx + w]
        ridge |= (
            (clearance >= a - 1e-9)
            & (clearance >= b - 1e-9)
            & (clearance > np.minimum(a, b) + 1e-9)
        )
    return clearance, ridge & safe


def routing_geometry(snapshot):
    clearance = getattr(snapshot, "center_clearance", None)
    centers = getattr(snapshot, "centerline", None)
    if clearance is None or centers is None:
        clearance, centers = center_geometry(snapshot.model, snapshot.clearance, snapshot.safe)
    model = snapshot.model
    scale = model.route_clearance_scale
    cost = 1.0 + model.route_clearance_weight * (scale / (clearance + scale)) ** 2
    if centers.any():
        # The displayed ridges must influence the route, not merely annotate it.
        # Exterior cells are not centerline sites. Retain wall/unknown clearance
        # costs so a nearby narrow ridge cannot beat a much wider safe passage.
        to_center = distance_field(centers, model.resolution, exterior_blocked=False)
        cost += model.route_clearance_weight * (to_center / (clearance + scale)) ** 2
    return clearance, centers, cost
