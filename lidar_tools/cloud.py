"""Decode XYZ without assuming a packed, little-endian PointCloud2 layout."""

import numpy as np


def xyz_array(cloud):
    """Return H x W x 3 XYZ, honoring row padding, offsets and byte order."""
    if cloud.width == 0 or cloud.height == 0:
        return np.empty((0, 3), dtype=float)
    if cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError("Invalid point_step or row_step")
    if len(cloud.data) != cloud.height * cloud.row_step:
        raise ValueError("Data length does not match height * row_step")
    fields = {f.name: f for f in cloud.fields}
    endian = ">" if cloud.is_bigendian else "<"
    axes = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError(f"{name} must be one FLOAT32 or FLOAT64 field")
        dtype = np.dtype(endian + ("f4" if field.datatype == 7 else "f8"))
        if field.offset < 0 or field.offset + dtype.itemsize > cloud.point_step:
            raise ValueError(f"{name} offset extends beyond point_step")
        axes.append(
            np.ndarray(
                (cloud.height, cloud.width),
                dtype=dtype,
                buffer=cloud.data,
                offset=field.offset,
                strides=(cloud.row_step, cloud.point_step),
            )
        )
    return np.stack(axes, axis=-1).reshape(-1, 3)


def cloud_summary(cloud, min_range, max_range):
    xyz = xyz_array(cloud)
    finite = xyz[np.isfinite(xyz).all(axis=1)]
    with np.errstate(over="ignore", invalid="ignore"):
        ranges = np.linalg.norm(finite.astype(float), axis=1)
    valid_ranges = ranges[np.isfinite(ranges)]
    usable = finite[(ranges >= min_range) & (ranges <= max_range)]
    return {
        "points": int(len(xyz)),
        "finite_points": int(len(finite)),
        "invalid_points": int(len(xyz) - len(finite)),
        "in_range_points": int(len(usable)),
        "nonfinite_range_points": int(len(ranges) - len(valid_ranges)),
        "range_min_m": float(valid_ranges.min()) if len(valid_ranges) else None,
        "range_max_m": float(valid_ranges.max()) if len(valid_ranges) else None,
        "xyz_min_m": usable.min(axis=0).astype(float).tolist() if len(usable) else None,
        "xyz_max_m": usable.max(axis=0).astype(float).tolist() if len(usable) else None,
    }
