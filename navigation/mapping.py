"""A small 2-D ray map: unknown stays unknown until a valid ray observes it."""

import math
import numpy as np

UNKNOWN, FREE, OCCUPIED, START_AREA = 0, 1, 2, 3


def distance_field(blocked, resolution):
    """Exact Euclidean distance to blocked cell centres; exterior is blocked.

    Separable lower-envelope squared distance transform, avoiding a SciPy
    dependency. Subtract a half cell diagonal to bound distance to cell area.
    """
    mask = np.pad(np.asarray(blocked, dtype=bool), 1, constant_values=True)
    field = np.where(mask, 0.0, 1e9)

    def transform(f):
        n = len(f)
        v = np.zeros(n, dtype=int)
        z = np.empty(n + 1)
        out = np.empty(n)
        k = 0
        z[0] = -np.inf
        z[1] = np.inf
        for q in range(1, n):
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * (q - v[k]))
            while s <= z[k]:
                k -= 1
                s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * (q - v[k]))
            k += 1
            v[k] = q
            z[k] = s
            z[k + 1] = np.inf
        k = 0
        for q in range(n):
            while z[k + 1] < q:
                k += 1
            out[q] = (q - v[k]) ** 2 + f[v[k]]
        return out

    for y in range(field.shape[0]):
        field[y] = transform(field[y])
    for x in range(field.shape[1]):
        field[:, x] = transform(field[:, x])
    return np.maximum(0.0, np.sqrt(field[1:-1, 1:-1]) * resolution - resolution / math.sqrt(2))


class RayMap:
    def __init__(
        self, resolution=0.1, size=120, origin=(-6.0, -6.0), body_radius=0.3, clearance=1.1
    ):
        self.resolution = resolution
        self.origin = np.array(origin, dtype=float)
        self.grid = np.zeros((size, size), dtype=np.uint8)
        self.body_radius = body_radius
        self.clearance = clearance
        self.scans = 0

    def cell(self, xy):
        ix, iy = np.floor((np.array(xy) - self.origin) / self.resolution).astype(int)
        return int(iy), int(ix)

    def world(self, cell):
        y, x = cell
        return self.origin + (np.array([x, y]) + 0.5) * self.resolution

    def inside(self, cell):
        return 0 <= cell[0] < self.grid.shape[0] and 0 <= cell[1] < self.grid.shape[1]

    def update(
        self, sensor_xyz, world_rotation, world_translation, body_xy, body_world_z, near_range=0.5
    ):
        xyz = np.asarray(sensor_xyz, dtype=float).reshape(-1, 3)
        finite = np.isfinite(xyz).all(axis=1)
        xyz = xyz[finite]
        ranges = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(ranges >= max(near_range, 0.35)) & (ranges <= 12.0)]
        world = xyz @ np.asarray(world_rotation).T + world_translation
        slab = (world[:, 2] >= body_world_z - 0.3) & (world[:, 2] <= body_world_z + 0.4)
        world = world[slab]
        delta = world[:, :2] - np.array(world_translation)[:2]
        radial = np.linalg.norm(delta, axis=1)
        valid = radial >= 0.35
        world, delta, radial = world[valid], delta[valid], radial[valid]
        if not len(world):
            raise ValueError("No valid fixed-height obstacle rays")
        bins = (
            np.floor((np.arctan2(delta[:, 1], delta[:, 0]) + math.pi) * 720 / (2 * math.pi)).astype(
                int
            )
            % 720
        )
        order = np.lexsort((radial, bins))
        _, first = np.unique(bins[order], return_index=True)
        selected = order[first]
        hits = world[selected, :2]
        delta = delta[selected]
        radial = radial[selected]
        if len(hits) < 120:
            raise ValueError("Too little angular ray coverage for room mapping")
        # Sample at half-cell intervals. Only valid return rays clear space;
        # missing beams and space behind their endpoint are never cleared.
        steps = np.arange(max(0.35, near_range), float(radial.max()), self.resolution / 2)
        positions = (
            np.array(world_translation)[:2]
            + delta[:, None, :] / radial[:, None, None] * steps[None, :, None]
        )
        before = steps[None, :] < radial[:, None] - self.resolution / 2
        cells = np.floor((positions - self.origin) / self.resolution).astype(int)
        h, w = self.grid.shape
        good = (
            before
            & (cells[:, :, 0] >= 0)
            & (cells[:, :, 0] < w)
            & (cells[:, :, 1] >= 0)
            & (cells[:, :, 1] < h)
        )
        self.grid[cells[:, :, 1][good], cells[:, :, 0][good]] = FREE
        endpoints = np.floor((hits - self.origin) / self.resolution).astype(int)
        good = (
            (endpoints[:, 0] >= 0)
            & (endpoints[:, 0] < w)
            & (endpoints[:, 1] >= 0)
            & (endpoints[:, 1] < h)
        )
        self.grid[endpoints[good, 1], endpoints[good, 0]] = OCCUPIED
        # The current physical vehicle footprint is our known start volume.
        # Never erase an observed occupied cell with this footprint seed.
        yy, xx = np.indices(self.grid.shape)
        centres = self.origin + (np.stack((xx, yy), axis=-1) + 0.5) * self.resolution
        radius = 0.6 if self.scans == 0 else 0.35
        footprint = np.linalg.norm(centres - np.array(body_xy), axis=-1) <= radius
        self.grid[footprint & (self.grid == UNKNOWN)] = START_AREA
        self.scans += 1
        return hits

    def geometry(self):
        clearance = distance_field(self.grid == OCCUPIED, self.resolution)
        known = (self.grid == FREE) | (self.grid == START_AREA)
        observed = distance_field(~known, self.resolution)
        body_free = known & (observed >= self.body_radius)
        safe = body_free & (clearance >= self.clearance)
        return clearance, body_free, safe
