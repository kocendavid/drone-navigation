"""Conservative motion budget for promoting LiDAR hits to persistent walls."""

import math
import numpy as np


# The existing Example_Rotary sensor is configured to one full scan per tick.
SCAN_RATE_HZ = 10.0
SCAN_PERIOD_S = 1.0 / SCAN_RATE_HZ


def confirmation_quality(poses, resolution, mapping_range, *, exposure_s=SCAN_PERIOD_S):
    """Bound raw endpoint travel across sampled sensor poses, in metres.

    Snapshot sweeps require one valid capture-time pose, regardless of motion
    between ticks. Rolling sources retain the conservative motion gate;
    unknown exposure cannot certify a scan.
    Sum intermediate rolling motion so matching endpoints cannot imply rest.
    """
    translation = rotation = 0.0
    try:
        if (
            exposure_s is None
            or not math.isfinite(exposure_s)
            or exposure_s < 0
            or len(poses) != (1 if exposure_s == 0 else 5)
            or not (resolution > 0 and mapping_range > 0)
        ):
            raise ValueError("Incomplete scan-period transform history")
        for matrix, position in poses:
            matrix, position = np.asarray(matrix), np.asarray(position)
            if (
                matrix.shape != (3, 3)
                or position.shape != (3,)
                or not np.isfinite(matrix).all()
                or not np.isfinite(position).all()
                or not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-5)
            ):
                raise ValueError("Invalid scan-period transform")
        if exposure_s == 0:
            return dict(
                allowed=True,
                reason="Instantaneous sweep with capture-time transform",
                exposure_s=0.0,
                motion_bound_m=0.0,
                budget_m=resolution / 2,
            )
        for (a, x), (b, y) in zip(poses, poses[1:]):
            translation += float(np.linalg.norm(np.asarray(y) - x))
            cosine = (float(np.trace(np.asarray(a).T @ b)) - 1.0) / 2
            rotation += math.acos(float(np.clip(cosine, -1, 1)))
        bound = translation + mapping_range * rotation
        allowed = math.isfinite(bound) and bound <= resolution / 2
        return dict(
            allowed=allowed,
            reason="Low-motion scan" if allowed else "Scan motion exceeds half a map cell",
            motion_bound_m=bound,
            budget_m=resolution / 2,
            translation_m=translation,
            rotation_deg=math.degrees(rotation),
            exposure_s=exposure_s,
        )
    except (ValueError, TypeError):
        return dict(allowed=False, reason="Incomplete or invalid scan-period transform history")
