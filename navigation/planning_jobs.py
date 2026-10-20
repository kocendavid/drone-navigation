"""CPU-heavy planning jobs. Inputs are immutable snapshots, never a ROS node."""

import json
import math

from .centering import routing_geometry
from .exploration import choose_frontier, visible_unknown
from .mission import assess_completion, save_map
from .planning import plan, curve_acceptance, motion_score
from .curves import tangent_tail, LATERAL_ACCELERATION_SHARE
from .prepared_route import PreparedRoute


class PlanningJobs:
    def __call__(self, request):
        kind, data = request
        token, snap = data["token"], data["snap"]
        cfg = data.get("settings")
        try:
            if kind == "plan":
                path = PreparedRoute(
                    plan(snap, data["start"], data["goal"], settings=cfg), snap.model, cfg
                )
                return token, path, None, None
            if kind == "check":
                return (
                    token,
                    assess_completion(snap, data["start"]),
                    None,
                    ("mission_check", snap.stamp),
                )
            if kind == "save":
                folder = data["folder"]
                saved = save_map(snap, folder, data["evidence"])
                (folder / "history.json").write_text(
                    json.dumps(data["history"], indent=2) + "\n", encoding="utf-8"
                )
                return token, saved, None, "mission_saved"
            if kind == "frontier":
                try:
                    choice = choose_frontier(
                        snap,
                        data["start"],
                        data["visited"],
                        data["trail"],
                        catalog=data["catalog"],
                        active_id=data["active_id"],
                        settings=cfg,
                    )
                    return token, PreparedRoute(choice.path, snap.model, cfg), None, choice
                except ValueError as exc:
                    tag = "mission_check" if data["mission_requested"] else "exploration_check"
                    return (
                        token,
                        assess_completion(snap, data["start"]),
                        str(exc),
                        (tag, snap.stamp),
                    )
            if kind == "reconsider":
                current, prefix = data["current"], data["prefix"]
                gain = visible_unknown(
                    snap.model.grid, snap.model.cell(current), snap.model.resolution
                )
                choice = choose_frontier(
                    snap,
                    prefix.anchor,
                    data["visited"],
                    data["trail"],
                    catalog=data["catalog"],
                    active_id=data["active_id"],
                    current_goal=current,
                    settings=cfg,
                )
                _, _, cost = routing_geometry(snap)
                tail = tangent_tail(
                    prefix.points,
                    choice.path,
                    curve_acceptance(snap, cost),
                    cfg.cruise_m_s**2 / (LATERAL_ACCELERATION_SHARE * cfg.acceleration_m_s2),
                )
                original = prefix.points + choice.path[1:]
                blended = prefix.points + tail[1:]
                if motion_score(snap, blended, cost, cfg) < motion_score(snap, original, cost, cfg):
                    choice.path = blended
                else:
                    choice.path = original
                choice.path = PreparedRoute(choice.path, snap.model, cfg)
                choice.path.commitment = prefix
                choice.route_length_m = sum(
                    math.dist(a, b) for a, b in zip(choice.path, choice.path[1:])
                )
                choice.selection_reason += f"; {prefix.length:.1f} m committed prefix"
                if snap.model.cell(choice.goal) == snap.model.cell(current):
                    choice.path.recenter_improves = (
                        motion_score(snap, choice.path, cost, cfg)
                        < motion_score(snap, data["current_route"], cost, cfg) * 0.95
                    )
                return token, current, gain, choice, None
            raise ValueError(f"Unknown planning job: {kind}")
        except (ValueError, OSError) as exc:
            if kind == "reconsider":
                return token, data["current"], None, None, str(exc)
            return token, None, str(exc), "mission_saved" if kind == "save" else None
