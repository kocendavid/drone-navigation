"""Explicit simulator mass overrides, separate from source asset density."""

import math


def normalized_link_masses(measured, total_kg):
    if (
        not measured
        or isinstance(total_kg, bool)
        or not isinstance(total_kg, (int, float))
        or not math.isfinite(total_kg)
        or not 0.05 <= total_kg <= 25
    ):
        raise ValueError("Choose a simulated drone mass between 0.05 and 25 kg")
    if any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
        for v in measured.values()
    ):
        raise ValueError("Every measured link mass must be finite and positive")
    factor = total_kg / sum(measured.values())
    return {path: value * factor for path, value in measured.items()}


def configured_link_masses(config):
    masses = config.get("link_masses_kg", {})
    if not isinstance(masses, dict):
        raise ValueError("link_masses_kg must be a path-to-mass object")
    if not masses:
        return {}
    if any(
        not isinstance(path, str)
        or not path.startswith(config["vehicle_root_prim_path"] + "/")
        or isinstance(value, bool)
        or not isinstance(value, (float, int))
        or not math.isfinite(value)
        or value <= 0
        for path, value in masses.items()
    ):
        raise ValueError("Mass overrides must name vehicle links and positive finite kg values")
    target = config.get("vehicle_mass_kg")
    if (
        isinstance(target, bool)
        or not isinstance(target, (int, float))
        or not math.isfinite(target)
        or not 0.05 <= target <= 25
        or not math.isclose(sum(masses.values()), target, rel_tol=1e-6)
    ):
        raise ValueError("Link masses must sum to vehicle_mass_kg")
    return masses
