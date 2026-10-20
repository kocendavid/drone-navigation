"""Tangent corner rounding, accepted only in observed, clearance-safe space."""

import math
import numpy as np


CORNER_DEVIATION_M = 0.02
LATERAL_ACCELERATION_SHARE = 0.6


def rounded_route(points, acceptable, maximum_cut=3.0, spacing=0.08, minimum_radius=None):
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
        desired_cut = maximum_cut
        if minimum_radius is not None:
            # Peak curvature of this quadratic is sin(theta/2)/(cut*cos(theta/2)^2).
            desired_cut = max(
                desired_cut,
                minimum_radius * math.sqrt((1 - cosine) / 2) / max((1 + cosine) / 2, 1e-8),
            )
        cut = min(desired_cut, 0.4 * before, 0.4 * after)
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
    """Continuous curvature bound plus backward braking pass; final goal stops.

    Reserve acceleration for tangent speed changes as well as lateral motion.
    Dense rounded samples are traversed continuously, not treated as stops.
    """
    limits = np.full(len(path), cruise, dtype=float)
    limits[-1] = 0.0
    for i in range(1, len(path) - 1):
        limits[i] = corner_limit(path, i, acceleration, cruise)
    for i in range(len(path) - 2, -1, -1):
        distance = np.linalg.norm(path[i + 1] - path[i])
        limits[i] = min(
            limits[i],
            math.sqrt(
                limits[i + 1] ** 2 + 2 * LATERAL_ACCELERATION_SHARE * acceleration * distance
            ),
        )
    return limits


def corner_geometry(path, i):
    """Turn angle and effective curvature, including a 2 cm corner-blend budget.

    A sparse polyline vertex has no physical radius. Bound the radius of an
    implicit tangent fillet by its permitted deviation from the vertex. This
    avoids treating a widely sampled sharp corner as a broad circular arc.
    """
    a, b = path[i] - path[i - 1], path[i + 1] - path[i]
    x, y = np.linalg.norm(a), np.linalg.norm(b)
    if min(x, y) < 1e-8:
        return math.pi, math.inf
    angle = math.acos(float(np.clip((a @ b) / (x * y), -1, 1)))
    if angle > 1e-5:
        curvature = 2 * math.sin(angle) / max(np.linalg.norm(a + b), 1e-8)
        blend_curvature = (1 / max(math.cos(angle / 2), 1e-12) - 1) / CORNER_DEVIATION_M
        return angle, max(curvature, blend_curvature)
    return angle, 0.0


def corner_limit(path, i, acceleration, cruise):
    """Continuous speed bound from lateral acceleration and corner geometry."""
    _, curvature = corner_geometry(path, i)
    return (
        min(cruise, math.sqrt(LATERAL_ACCELERATION_SHARE * acceleration / curvature))
        if curvature > 0
        else cruise
    )


def flight_metrics(path, acceleration, cruise):
    """Offline rest-to-rest estimate; excludes live tracking and obstacle stops."""
    points = np.asarray(path, dtype=float)
    limits = speed_profile(points, acceleration, cruise)
    limits[0] = 0.0
    tangent_acceleration = LATERAL_ACCELERATION_SHARE * acceleration
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    for i, distance in enumerate(distances):
        limits[i + 1] = min(
            limits[i + 1], math.sqrt(limits[i] ** 2 + 2 * tangent_acceleration * distance)
        )
    seconds = 0.0
    for i, distance in enumerate(distances):
        a, b = limits[i : i + 2]
        peak = min(cruise, math.sqrt(tangent_acceleration * distance + (a * a + b * b) / 2))
        ramp_distance = max(0.0, (2 * peak * peak - a * a - b * b) / (2 * tangent_acceleration))
        seconds += (2 * peak - a - b) / tangent_acceleration
        seconds += max(0.0, distance - ramp_distance) / max(peak, 1e-8)
    variation = float(np.abs(np.diff(limits)).sum())
    return dict(
        seconds=seconds,
        speed_variation_m_s=variation,
        length_m=float(distances.sum()),
        speeds=limits,
    )


def tangent_tail(prefix, tail, acceptable, minimum_radius, spacing=0.08):
    """Blend forward from an immutable prefix; reject bends without safe room.

    Only the tail changes, so asynchronous prefix rebasing remains exact.
    Every candidate is audited by the caller against the current safe mask.
    """
    points = np.asarray(tail, dtype=float)
    anchor = points[0]
    incoming = next(
        (
            anchor - np.asarray(p)
            for p in reversed(prefix[:-1])
            if np.linalg.norm(anchor - p) > 1e-6
        ),
        None,
    )
    if incoming is None or len(points) < 2:
        return tail
    incoming /= np.linalg.norm(incoming)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(lengths.sum())
    distance = min(total, max(1.0, 2 * minimum_radius))
    while distance >= 0.16:
        remaining = distance
        for i, length in enumerate(lengths):
            if length > 1e-8 and remaining <= length + 1e-8:
                outgoing = (points[i + 1] - points[i]) / length
                end = points[i] + outgoing * min(length, remaining)
                handle = min(distance / 3, float(np.linalg.norm(end - anchor)) / 2)
                t = np.linspace(0, 1, max(8, math.ceil(2 * distance / spacing)) + 1)[:, None]
                curve = (
                    (1 - t) ** 3 * anchor
                    + 3 * (1 - t) ** 2 * t * (anchor + incoming * handle)
                    + 3 * (1 - t) * t**2 * (end - outgoing * handle)
                    + t**3 * end
                )
                candidate = [p.tolist() for p in curve]
                first = i + 2 if np.linalg.norm(points[i + 1] - end) < 1e-7 else i + 1
                candidate.extend(p.tolist() for p in points[first:])
                if handle > 0.04 and acceptable(candidate, tail):
                    return candidate
                break
            remaining -= length
        distance *= 0.5
    return tail
