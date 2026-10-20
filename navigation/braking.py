"""Stopping geometry for a moving route handover; no ROS or flight commands."""

import math
import numpy as np


def stopping_envelope(reference, velocity, pose, measured_velocity, deceleration, reaction_s):
    """Reserve both stopping rays and the space between them.

    The reaction reserve applies to the physical and commanded motion. This is
    a planning model, not a measured guarantee of the aircraft's stopping power.
    """

    def endpoint(start, v):
        return np.asarray(start) + v * (reaction_s + np.linalg.norm(v) / (2 * deceleration))

    return np.array(
        [reference, endpoint(reference, velocity), pose, endpoint(pose, measured_velocity)],
        dtype=float,
    )


def swept_space_safe(snapshot, points):
    """Conservatively rasterize the convex stopping envelope into body-free space.

    Test cell squares, including polygon interiors and diagonal boundaries, not
    just the two endpoints. The existing mask already includes vehicle radius.
    Unknown cells and out-of-map geometry cannot certify a stop.
    """
    points = np.asarray(points, dtype=float)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        return False
    model = snapshot.model
    points = (points - np.asarray(model.origin)) / model.resolution
    h, w = snapshot.body_free.shape
    if np.any(points < 0) or np.any(points >= [w, h]):
        return False
    # Monotone-chain convex hull; only four input points.
    ordered = sorted(set(map(tuple, points.tolist())))

    def cross(a, b, c):
        u, v = np.subtract(b, a), np.subtract(c, a)
        return u[0] * v[1] - u[1] * v[0]

    halves = []
    for sequence in (ordered, ordered[::-1]):
        half = []
        for point in sequence:
            while len(half) > 1 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.append(half[:-1])
    hull = np.asarray(halves[0] + halves[1] or ordered)
    low = np.floor(points.min(axis=0) - 1).astype(int)
    high = np.floor(points.max(axis=0) + 1).astype(int)
    x, y = np.meshgrid(np.arange(low[0], high[0] + 1), np.arange(low[1], high[1] + 1))
    centers = np.stack((x + 0.5, y + 0.5), axis=-1)
    radius = math.sqrt(0.5)  # Cell half diagonal: conservative square intersection.
    if len(hull) <= 2:
        a, b = hull[0], hull[-1]
        edge = b - a
        fraction = np.clip(((centers - a) @ edge) / max(float(edge @ edge), 1e-20), 0, 1)
        touched = np.linalg.norm(centers - a - fraction[..., None] * edge, axis=-1) <= radius
    else:
        touched = np.ones(x.shape, dtype=bool)
        for a, b in zip(hull, np.roll(hull, -1, axis=0)):
            edge = b - a
            delta = centers - a
            touched &= edge[0] * delta[..., 1] - edge[1] * delta[
                ..., 0
            ] >= -radius * np.linalg.norm(edge)
    xx, yy = x[touched], y[touched]
    return bool(
        np.all((xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)) and np.all(snapshot.body_free[yy, xx])
    )
