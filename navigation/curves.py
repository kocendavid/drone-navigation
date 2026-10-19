"""Tangent corner rounding, accepted only in observed, clearance-safe space."""

import math
import numpy as np


def rounded_route(points, acceptable, maximum_cut=1.5, spacing=0.08):
    points = [np.asarray(p, dtype=float) for p in points]
    result = [points[0]]
    for a, b, c in zip(points, points[1:], points[2:]):
        incoming, outgoing = b - a, c - b
        before, after = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        if min(before, after) < 1e-6:
            continue
        u, v = incoming / before, outgoing / after
        cosine = float(u @ v)
        if cosine > 0.99999 or cosine < -0.95:
            result.append(b)
            continue
        cut = min(maximum_cut, 0.4 * before, 0.4 * after)
        accepted = None
        while cut >= 0.08:
            entry, exit = b - u * cut, b + v * cut
            count = max(6, math.ceil(2 * cut / spacing))
            t = np.linspace(0, 1, count + 1)[:, None]
            curve = (1 - t) ** 2 * entry + 2 * t * (1 - t) * b + t**2 * exit
            candidate = [result[-1], *curve, c]
            if acceptable(candidate, [result[-1], b, c]):
                accepted = curve
                break
            cut *= 0.5
        result.extend(accepted if accepted is not None else [b])
    result.append(points[-1])
    return [
        p.tolist()
        for i, p in enumerate(result)
        if i == 0 or np.linalg.norm(p - result[i - 1]) > 1e-7
    ]


def speed_profile(path, acceleration, cruise):
    """Curvature bound plus backward braking pass; final and sharp bends stop.

    Reserve acceleration for tangent speed changes as well as lateral motion.
    Dense rounded samples are traversed continuously, not treated as stops.
    """
    limits = np.full(len(path), cruise, dtype=float)
    limits[-1] = 0.0
    for i in range(1, len(path) - 1):
        a, b = path[i] - path[i - 1], path[i + 1] - path[i]
        x, y = np.linalg.norm(a), np.linalg.norm(b)
        if min(x, y) < 1e-8:
            limits[i] = 0
            continue
        angle = math.acos(float(np.clip((a @ b) / (x * y), -1, 1)))
        if angle > math.radians(12):
            limits[i] = 0.0  # An unrounded corner has no finite turning radius.
        elif angle > 1e-5:
            curvature = 2 * math.sin(angle) / max(np.linalg.norm(a + b), 1e-8)
            limits[i] = min(cruise, math.sqrt(0.6 * acceleration / curvature))
    for i in range(len(path) - 2, -1, -1):
        distance = np.linalg.norm(path[i + 1] - path[i])
        limits[i] = min(
            limits[i], math.sqrt(limits[i + 1] ** 2 + 2 * 0.6 * acceleration * distance)
        )
    return limits
