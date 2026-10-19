"""Persistent frontier identities and evidence-based lifecycle, not room labels."""

from collections import deque
import numpy as np

MINIMUM_SPAN_M = 1.5


def boundary_geometry(cells, resolution):
    """Principal-axis extent of the observed boundary, not doorway clearance."""
    points = np.asarray(sorted(cells), dtype=float)
    center = points.mean(axis=0)
    if len(points) > 1:
        _, _, vectors = np.linalg.svd(points - center, full_matrices=False)
        axis = vectors[0]
    else:
        axis = np.array([0.0, 1.0])
    projection = (points - center) @ axis
    low, high = float(projection.min()), float(projection.max())
    midpoint = center + axis * ((low + high) / 2)
    return float((high - low) * resolution + resolution), midpoint, axis


def components(mask):
    remaining = set(map(tuple, np.argwhere(mask).tolist()))
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        region = {seed}
        queue = deque([seed])
        while queue:
            y, x = queue.popleft()
            # Diagonal cells belong to the same sampled boundary; flight
            # connectivity still uses the planner's stricter corner checks.
            for cell in ((y + dy, x + dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx):
                if cell in remaining:
                    remaining.remove(cell)
                    region.add(cell)
                    queue.append(cell)
        result.append(frozenset(region))
    return result


class FrontierLedger:
    RESOLUTION_SCANS = 3

    def __init__(self):
        self.records = {}
        self.next_id = 1
        self.active_id = None
        self.last_stamp = None

    def update(self, snapshot, mask):
        if not snapshot.valid or self.last_stamp is not None and snapshot.stamp <= self.last_stamp:
            return None
        self.last_stamp = snapshot.stamp
        regions = components(mask)
        owners = {}
        for key, row in self.records.items():
            if row["status"] in ("resolved", "merged"):
                continue
            for y, x in row["cells"]:
                for dy, dx in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
                    owners.setdefault((y + dy, x + dx), set()).add(key)
        overlaps = []
        for i, cells in enumerate(regions):
            votes = {}
            for cell in cells:
                for key in owners.get(cell, ()):
                    votes[key] = votes.get(key, 0) + 1
            overlaps.extend((count, key == self.active_id, key, i) for key, count in votes.items())
        assigned = {}
        used = set()
        for _, _, key, i in sorted(overlaps, reverse=True):
            if key not in used and i not in assigned:
                assigned[i] = key
                used.add(key)
        changes = dict(added=[], resolved=[], merged=[])
        present = set()
        current_owner = {}
        for i, cells in enumerate(regions):
            key = assigned.get(i)
            if key is None:
                key = self.next_id
                self.next_id += 1
                self.records[key] = dict(
                    id=key,
                    first_seen_stamp=snapshot.stamp,
                    route_length_m=None,
                    viewpoint_xy=None,
                    reason="Waiting for a reachable viewpoint",
                    parent_id=next(
                        (
                            old
                            for _, _, old, region in sorted(overlaps, reverse=True)
                            if region == i
                        ),
                        None,
                    ),
                )
                changes["added"].append(key)
            row = self.records[key]
            span, midpoint, axis = boundary_geometry(cells, snapshot.model.resolution)
            row.update(
                cells=cells,
                missing_scans=0,
                last_seen_stamp=snapshot.stamp,
                status="active" if key == self.active_id else "pending",
                boundary_xy=snapshot.model.world(np.mean(list(cells), axis=0)).tolist(),
                span_m=span,
                midpoint_xy=snapshot.model.world(midpoint).tolist(),
                axis_xy=axis[::-1].tolist(),
                boundary_points_xy=[
                    snapshot.model.world(cell).tolist()
                    for cell in sorted(cells)[:: max(1, len(cells) // 500)]
                ],
            )
            if span < MINIMUM_SPAN_M:
                row.update(
                    status="filtered",
                    reason=f"Boundary span {span:.2f} m below {MINIMUM_SPAN_M:.1f} m; unresolved",
                    route_length_m=None,
                    viewpoint_xy=None,
                )
                if key == self.active_id:
                    self.active_id = None
            present.add(key)
            for cell in cells:
                current_owner[cell] = key
        for key, row in self.records.items():
            row["present"] = key in present
            if key not in present and row.get("status") not in ("resolved", "merged"):
                successors = [current_owner[cell] for cell in row["cells"] if cell in current_owner]
                if successors:
                    successor = max(set(successors), key=successors.count)
                    row.update(
                        status="merged",
                        merged_into=successor,
                        reason=f"Boundary work continues under F{successor}",
                    )
                    changes["merged"].append(key)
                    if self.active_id == key:
                        self.active_id = successor
                        self.records[successor]["status"] = "active"
                    continue
                row["missing_scans"] = row.get("missing_scans", 0) + 1
                if row["missing_scans"] >= self.RESOLUTION_SCANS:
                    row.update(
                        status="resolved",
                        reason="Boundary absent in three fresh mapped scans",
                        resolved_stamp=snapshot.stamp,
                        route_length_m=None,
                    )
                    changes["resolved"].append(key)
                    if self.active_id == key:
                        self.active_id = None
        return changes if any(changes.values()) else None

    def snapshot(self):
        return [dict(row) for row in self.records.values() if row.get("present")]

    def apply_choice(self, choice):
        for assessment in choice.frontier_assessments or ():
            row = self.records.get(assessment["id"])
            if row and row["status"] not in ("resolved", "merged"):
                row.update(assessment)
        if (
            choice.frontier_id is not None
            and choice.frontier_id in self.records
            and self.records[choice.frontier_id]["status"] not in ("resolved", "merged")
            and self.records[choice.frontier_id].get("present")
        ):
            self.active_id = choice.frontier_id
        for key, row in self.records.items():
            if row["status"] not in ("resolved", "merged", "unreachable", "deferred", "filtered"):
                row["status"] = "active" if key == self.active_id else "pending"

    def report(self):
        rows = []
        for row in self.records.values():
            rows.append(
                {key: value for key, value in row.items() if key not in ("cells", "present")}
            )
            rows[-1]["boundary_cells"] = len(row["cells"])
        rows.sort(
            key=lambda r: (
                r["status"] != "active",
                r["status"] == "resolved",
                r.get("route_length_m") is None,
                r.get("route_length_m") or 0,
                r["id"],
            )
        )
        return dict(
            active_id=self.active_id,
            frontiers=rows,
            pending=sum(r["status"] not in ("resolved", "merged") for r in rows),
            resolved=sum(r["status"] == "resolved" for r in rows),
            minimum_span_m=MINIMUM_SPAN_M,
            filtered=sum(r["status"] == "filtered" for r in rows),
        )
