"""Export an offline angle/radius sweep; never connects to flight control."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.curves import (
    CORNER_DEVIATION_M,
    LATERAL_ACCELERATION_SHARE,
    corner_limit,
    rounded_route,
)
from navigation.settings import load_settings


def sweep(settings):
    rows = []
    for degrees in (0, 3, 5, 8, 10, 11.9, 12, 12.1, 15, 20, 30, 45, 60, 90, 120):
        theta = math.radians(degrees)
        raw = [[-30, 0], [0, 0], [30 * math.cos(theta), 30 * math.sin(theta)]]
        for cut in (0, 1, 3, 6):
            path = np.asarray(
                raw if cut == 0 else rounded_route(raw, lambda *_: True, maximum_cut=cut)
            )
            limit = min(
                corner_limit(path, i, settings.acceleration_m_s2, settings.cruise_m_s)
                for i in range(1, len(path) - 1)
            )
            rows.append(
                dict(
                    turn_deg=degrees,
                    tangent_cut_m=cut,
                    speed_limit_m_s=limit,
                    retains_cruise=limit >= settings.cruise_m_s - 1e-6,
                )
            )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("results/turn_speed"))
    parser.add_argument("--settings", type=Path)
    args = parser.parse_args()
    cfg = load_settings(args.settings)
    rows = sweep(cfg)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "turn_speed.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lateral = LATERAL_ACCELERATION_SHARE * cfg.acceleration_m_s2
    # Largest isolated kink whose 2 cm fillet can retain cruise; adjacent
    # segment length / curvature can impose a stricter limit.
    raw_angle = math.degrees(
        2 * math.acos(1 / (1 + CORNER_DEVIATION_M * lateral / cfg.cruise_m_s**2))
    )
    summary = dict(
        evidence="Offline geometry and kinematic model, not a measured flight capability",
        cruise_m_s=cfg.cruise_m_s,
        lateral_acceleration_m_s2=lateral,
        minimum_cruise_radius_m=cfg.cruise_m_s**2 / lateral,
        isolated_kink_cruise_threshold_deg=raw_angle,
        corner_deviation_budget_m=CORNER_DEVIATION_M,
        yaw_cap_begins_above_deg=30,
        yaw_cap_fully_applies_at_deg=60,
        yaw_misaligned_speed_m_s=cfg.yaw_misaligned_speed_m_s,
        abrupt_handover_braking_angle_deg=math.degrees(math.acos(0.7)),
        caveat="Total turn angle is not local curvature. Live tracking, clearance, braking and yaw can lower speed.",
    )
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), layout="constrained")
    for cut in (0, 1, 3, 6):
        group = [r for r in rows if r["tangent_cut_m"] == cut]
        axes[0].plot(
            [r["turn_deg"] for r in group],
            [r["speed_limit_m_s"] for r in group],
            marker=".",
            label="Unrounded kink" if cut == 0 else f"Rounded, {cut} m tangent cut",
        )
    axes[0].axvline(12, color="gray", linestyle=":", label="Previous 12° stop threshold")
    axes[0].set(
        xlabel="Total bend angle (degrees)",
        ylabel="Geometry speed limit (m/s)",
        title="A wide bend permits more speed than a sharp kink",
        ylim=(0, cfg.cruise_m_s * 1.05),
    )
    axes[0].legend(fontsize=8)
    radii = np.linspace(0.1, 12, 240)
    axes[1].plot(radii, np.minimum(cfg.cruise_m_s, np.sqrt(lateral * radii)), color="#1f77b4")
    axes[1].axvline(summary["minimum_cruise_radius_m"], color="gray", linestyle=":")
    axes[1].set(
        xlabel="Turn radius (m)",
        ylabel="Geometry speed limit (m/s)",
        title=f"Current lateral budget: {lateral:.2f} m/s²",
        ylim=(0, cfg.cruise_m_s * 1.05),
    )
    for axis in axes:
        axis.grid(alpha=0.2)
    fig.suptitle(
        "Offline turn-speed model — excludes live tracking, yaw and obstacle limits", fontsize=11
    )
    fig.savefig(args.output / "turn_speed.png", dpi=160)
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
