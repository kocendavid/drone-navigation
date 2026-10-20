"""Checked C ABI for native planning searches; no flight-control authority."""

import ctypes as C
from functools import lru_cache
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class NativePlanning:
    def __init__(self):
        folder = ROOT / ".tools" / "native_navigation"
        manifest = json.loads((folder / "current_build.json").read_text())
        name = manifest["library"]
        if Path(name).name != name or not name.startswith(
            ("isaac_navigation_", "libisaac_navigation_")
        ):
            raise RuntimeError("Invalid native planning library path")
        self.dll = C.CDLL(str(folder / name))
        self.dll.bk_source_hash.restype = C.c_char_p
        self.dll.bk_abi_version.restype = C.c_int
        self.source_hash = source_hash()
        if self.dll.bk_abi_version() != 1 or self.dll.bk_source_hash().decode() != self.source_hash:
            raise RuntimeError(
                "Native planning build is stale; run tools/build_native_navigation.cmd"
            )
        byte = C.POINTER(C.c_uint8)
        integer = C.POINTER(C.c_int)
        self.dll.bk_astar.argtypes = [
            byte,
            C.POINTER(C.c_double),
            C.c_int,
            C.c_int,
            C.c_int,
            C.c_int,
            integer,
        ]
        self.dll.bk_astar.restype = C.c_int
        self.dll.bk_completion.argtypes = [byte, C.c_int, C.c_int, C.c_int, integer]
        self.dll.bk_completion.restype = C.c_int

    def astar(self, mask, start, goal, traversal_cost=None):
        mask = np.ascontiguousarray(mask, dtype=np.uint8)
        h, w = mask.shape
        if not all(0 <= point[0] < h and 0 <= point[1] < w for point in (start, goal)):
            raise ValueError("Start or destination is outside the traversable map")
        cost = (
            np.ascontiguousarray(traversal_cost, dtype=np.float64)
            if traversal_cost is not None
            else None
        )
        if cost is not None and cost.shape != mask.shape:
            raise ValueError("Traversal cost shape mismatch")
        output = np.empty(mask.size, dtype=np.int32)
        count = self.dll.bk_astar(
            mask.ctypes.data_as(C.POINTER(C.c_uint8)),
            cost.ctypes.data_as(C.POINTER(C.c_double)) if cost is not None else None,
            h,
            w,
            start[0] * w + start[1],
            goal[0] * w + goal[1],
            output.ctypes.data_as(C.POINTER(C.c_int)),
        )
        if count == -1:
            raise ValueError("No route through currently observed free space")
        if count == -2:
            raise ValueError("Invalid traversable map or traversal costs")
        if count < 0:
            raise RuntimeError("Native search allocation failed")
        return [(int(cell) // w, int(cell) % w) for cell in output[:count]]

    def completion(self, snapshot, start_xy):
        from .mission import CompletionEvidence

        grid = np.ascontiguousarray(snapshot.model.grid, dtype=np.uint8)
        start = snapshot.model.cell(start_xy)
        out = (C.c_int * 5)()
        h, w = grid.shape
        index = start[0] * w + start[1] if snapshot.model.inside(start) else -1
        result = self.dll.bk_completion(grid.ctypes.data_as(C.POINTER(C.c_uint8)), h, w, index, out)
        if result:
            raise RuntimeError("Native completion allocation failed")
        complete, count, unresolved, edge, valid = out
        reason = (
            "Current position is not in known space"
            if not valid
            else (
                "Connected observed space is enclosed; no boundary to unknown remains"
                if complete
                else (
                    "Known space reaches the map edge; enclosure is unverified"
                    if edge
                    else (
                        f"{unresolved} unknown boundary cells remain; exploration is incomplete"
                        if unresolved
                        else "Only the assumed starting footprint is known"
                    )
                )
            )
        )
        return CompletionEvidence(bool(complete), reason, count, unresolved, bool(edge))


def source_hash():
    return hashlib.sha256(
        "".join(
            hashlib.sha256((ROOT / "native_navigation" / name).read_bytes()).hexdigest()
            for name in ("kernels.cpp", "CMakeLists.txt")
        ).encode()
    ).hexdigest()


@lru_cache(maxsize=1)
def native_planning():
    """Prefer native searches when built; keep offline/reference use portable.

    A missing build permits the Python reference. A broken or stale existing
    build is an error, so it cannot silently change a running installation.
    """
    if not (ROOT / ".tools/native_navigation/current_build.json").exists():
        return None
    return NativePlanning()
