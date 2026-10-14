"""Measured forward surfaces for a flat-floor, stationary LiDAR experiment."""

from dataclasses import asdict, dataclass
import math
import numpy as np


def estimate_flat_floor(world_xyz):
    """Find a broad horizontal plane in the lower half of observed returns.

    Used only for the stationary flat-room experiment. Refuse narrow surfaces,
    slopes or weak support rather than treating arbitrary minimum Z as floor.
    """
    points = np.asarray(world_xyz, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 500:
        raise ValueError("Too few points for stationary floor calibration")
    lower = points[points[:, 2] <= np.percentile(points[:, 2], 50)]
    bins = np.floor(lower[:, 2] / 0.01).astype(np.int64)
    values, counts = np.unique(bins, return_counts=True)
    center = (values[np.argmax(counts)] + 0.5) * 0.01
    candidate = lower[np.abs(lower[:, 2] - center) <= 0.02]
    fraction = len(candidate) / len(points)
    spans = np.ptp(candidate[:, :2], axis=0)
    if fraction < 0.15 or min(spans) < 1.0:
        raise ValueError("No broad floor plane with sufficient support")
    design = np.column_stack((candidate[:, 0], candidate[:, 1], np.ones(len(candidate))))
    slope_x, slope_y, intercept = np.linalg.lstsq(design, candidate[:, 2], rcond=None)[0]
    residual = np.abs(candidate[:, 2] - design @ [slope_x, slope_y, intercept])
    if (
        math.hypot(slope_x, slope_y) > math.tan(math.radians(0.5))
        or np.percentile(residual, 95) > 0.015
    ):
        raise ValueError("Candidate floor is sloped or not sufficiently flat")
    height = float(np.median(candidate[:, 2]))
    return {
        "height_world_m": height,
        "support_fraction": fraction,
        "span_xy_m": spans.tolist(),
        "residual_p95_m": float(np.percentile(residual, 95)),
    }


@dataclass(frozen=True)
class ClearanceLimits:
    hold_m: float = 2.0
    min_range_m: float = 1.0
    max_range_m: float = 30.0
    forward_min_m: float = 0.2
    forward_max_m: float = 15.0
    half_width_m: float = 0.5
    body_z_min_m: float = -0.3
    body_z_max_m: float = 0.5
    floor_z_world_m: float = 0.0
    floor_margin_m: float = 0.12
    voxel_m: float = 0.05

    def __post_init__(self):
        if not all(math.isfinite(v) for v in asdict(self).values()):
            raise ValueError("All clearance limits must be finite")
        if not (
            0 <= self.min_range_m < self.max_range_m
            and 0 <= self.forward_min_m < self.hold_m < self.forward_max_m
            and self.body_z_min_m < self.body_z_max_m
            and min(self.half_width_m, self.floor_margin_m, self.voxel_m) > 0
        ):
            raise ValueError("Invalid clearance limits")


def forward_surfaces(
    sensor_xyz, body_rotation, body_translation, world_rotation, world_translation, limits
):
    """Transform, remove low floor returns, select a body corridor, then voxelize.

    CLEAR means the closest observed corridor surface is beyond hold_m. No
    returns means UNKNOWN. This does not infer free volume from missing rays.
    Distance uses the minimum original X, before downsampling, so a voxel
    representative cannot hide a point inside the hold threshold.
    """
    xyz = np.asarray(sensor_xyz, dtype=float).reshape(-1, 3)
    finite = xyz[np.isfinite(xyz).all(axis=1)]
    with np.errstate(over="ignore", invalid="ignore"):
        ranges = np.linalg.norm(finite, axis=1)
    usable = finite[
        np.isfinite(ranges) & (ranges >= limits.min_range_m) & (ranges <= limits.max_range_m)
    ]
    body = usable @ np.asarray(body_rotation).T + np.asarray(body_translation)
    world = usable @ np.asarray(world_rotation).T + np.asarray(world_translation)
    above_floor = world[:, 2] >= limits.floor_z_world_m + limits.floor_margin_m
    corridor = (
        above_floor
        & (body[:, 0] >= limits.forward_min_m)
        & (body[:, 0] <= limits.forward_max_m)
        & (np.abs(body[:, 1]) <= limits.half_width_m)
        & (body[:, 2] >= limits.body_z_min_m)
        & (body[:, 2] <= limits.body_z_max_m)
    )
    selected = body[corridor]
    nearest = selected[np.argmin(selected[:, 0])] if len(selected) else None
    if len(selected):
        # Keep the nearest original sample per 5 cm voxel, rather than a centroid.
        ordered = selected[np.argsort(selected[:, 0], kind="stable")]
        cells = np.floor(ordered / limits.voxel_m).astype(np.int64)
        _, first = np.unique(cells, axis=0, return_index=True)
        reduced = ordered[np.sort(first)]
    else:
        reduced = np.empty((0, 3), dtype=float)
    distance = float(nearest[0]) if nearest is not None else None
    state = "UNKNOWN" if distance is None else ("HOLD" if distance <= limits.hold_m else "CLEAR")
    metrics = {
        "state": state,
        "reason": (
            "No observed forward surfaces"
            if distance is None
            else "Closest observed forward surface"
        ),
        "forward_distance_m": distance,
        "nearest_body_xyz_m": nearest.tolist() if nearest is not None else None,
        "input_points": len(xyz),
        "invalid_points": len(xyz) - len(finite),
        "range_valid_points": len(usable),
        "floor_excluded_points": int((~above_floor).sum()),
        "corridor_points": len(selected),
        "voxel_points": len(reduced),
        "observed_world_min_z_m": float(world[:, 2].min()) if len(world) else None,
    }
    return metrics, reduced


def health_issue(
    *,
    now_wall,
    arrival_wall,
    clock_progress_wall,
    clock_ns,
    cloud_ns,
    stale_after=1.0,
    max_clock_skew=0.5,
):
    if arrival_wall is None or now_wall - arrival_wall > stale_after:
        return "Cloud missing or stale"
    if clock_progress_wall is None or now_wall - clock_progress_wall > stale_after:
        return "Simulation clock missing or stopped"
    if cloud_ns is None or cloud_ns <= 0:
        return "Cloud acquisition timestamp must be positive"
    if clock_ns is None or abs(clock_ns - cloud_ns) / 1e9 > max_clock_skew:
        return "Cloud and simulation clock disagree"
    return None
