"""Source-time comparisons and pause evidence. No flight-control authority."""

from bisect import bisect_left, bisect_right
import json
import math
from pathlib import Path
import re
import time


def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (list, tuple)):
        return [finite(v) for v in value]
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    return value


def parse_listener(text):
    """Parse PX4's native read-only listener; preserve raw, unadjusted times."""
    fields = {}
    for line in text.splitlines():
        match = re.match(r"^\s+([a-z][a-z0-9_]*):\s*(.*)$", line)
        if not match:
            continue
        key, value = match.groups()
        try:
            if value.startswith("["):
                fields[key] = [finite(float(x)) for x in value.split("]", 1)[0][1:].split(",")]
            elif value.split()[0] in ("True", "False"):
                fields[key] = value.split()[0] == "True"
            else:
                token = value.split()[0]
                fields[key] = (
                    finite(float(token)) if any(c in token.lower() for c in ".enai") else int(token)
                )
        except (ValueError, IndexError):
            continue
    return fields


def interpolate(rows, stamp, time_key, value_key, max_gap):
    """Bracket only: never extrapolate, bridge a reset, or fill a large gap."""
    times = [row[time_key] for row in rows]
    index = bisect_left(times, stamp)
    if index < len(rows) and times[index] == stamp:
        value = rows[index].get(value_key)
        return value if value is not None and all(v is not None for v in value) else None
    if index == 0 or index == len(rows):
        return None
    a, b = rows[index - 1], rows[index]
    gap = b[time_key] - a[time_key]
    if gap <= 0 or gap > max_gap or a.get("segment") != b.get("segment"):
        return None
    x, y = a.get(value_key), b.get(value_key)
    if x is None or y is None or any(v is None for v in [*x, *y]):
        return None
    fraction = (stamp - a[time_key]) / gap
    return [u + (v - u) * fraction for u, v in zip(x, y)]


def world_xy(ned, alignment):
    r, o = alignment["rotation"], alignment["offset"]
    return [r[0][0] * ned[1] + r[0][1] * ned[0] + o[0], r[1][0] * ned[1] + r[1][1] * ned[0] + o[1]]


def statistics(values):
    values = sorted(values)
    if not values:
        return dict(samples=0, mean=None, p95=None, maximum=None)
    return dict(
        samples=len(values),
        mean=sum(values) / len(values),
        p95=values[min(len(values) - 1, math.ceil(0.95 * len(values)) - 1)],
        maximum=values[-1],
    )


def incident_window(records, event, before_s=10, after_s=2):
    center = event["received_wall_s"]
    return [r for r in records if center - before_s <= r["received_wall_s"] <= center + after_s]


def analyze(records, event=None):
    """Use native PX4 time shared with HIL_SENSOR, not DDS-adjusted stamps.

    The installed SITL sets CLOCK_MONOTONIC from HIL_SENSOR.time_usec. Isaac
    records that exact sensor time alongside SimulationManager time and pose.
    Invalid estimates, frame resets, missing brackets and gaps are not errors
    of zero: they are counted as unavailable evidence.
    """
    records = list(records)
    stops = [r for r in records if r["kind"] in ("stop", "deadline_miss")]
    stop = event or (stops[-1] if stops else None)
    anchor = stop["received_wall_s"] if stop else math.inf
    truth = [r for r in records if r["kind"] == "isaac_truth"]
    native = [r for r in records if r["kind"] == "px4_position"]
    references = [r for r in records if r["kind"] == "reference" and r.get("alignment")]
    received = [r for r in records if r["kind"] == "px4_setpoint"]
    # Source/session boundaries prevent numerically overlapping times after Reset.
    if truth:
        before = [r for r in truth if r["received_wall_s"] <= anchor]
        truth = [r for r in truth if before and r["session"] == before[-1]["session"]]
    if native:
        before = [r for r in native if r["received_wall_s"] <= anchor]
        native = [r for r in native if before and r.get("px4_pid") == before[-1].get("px4_pid")]
    if native:
        received = [r for r in received if r.get("px4_pid") == native[-1].get("px4_pid")]
    if references:
        before = [r for r in references if r["received_wall_s"] <= anchor]
        references = [r for r in references if before and r["session"] == before[-1]["session"]]
    truth.sort(key=lambda r: r["px4_sim_us"])
    samples = []
    for row in truth:
        samples.append(
            dict(
                px4_us=row["px4_sim_us"],
                position=row["position_world_xy"],
                sim_ns=[row["isaac_sim_ns"]],
                segment=row["session"],
            )
        )
    intents = []
    for row in references:
        if row.get("sim_ns") is not None and row.get("world_xy") is not None:
            # Later records at an identical source instant replace earlier ones.
            value = dict(
                sim_ns=row["sim_ns"],
                position=row["world_xy"],
                segment=(row["session"], row.get("phase")),
                original=row,
            )
            if intents and intents[-1]["sim_ns"] == row["sim_ns"]:
                intents[-1] = value
            elif not intents or row["sim_ns"] > intents[-1]["sim_ns"]:
                intents.append(value)
    estimation, intended, applied, controller = [], [], [], []
    unavailable = dict(
        invalid_position=0,
        missing_truth=0,
        missing_alignment=0,
        frame_reset=0,
        missing_reference=0,
        missing_received_setpoint=0,
    )
    previous_reset = None
    seen_samples = set()
    received.sort(key=lambda r: r["fields"].get("timestamp", 0))
    received_times = [r["fields"].get("timestamp", 0) for r in received]
    details = []
    for row in native:
        fields = row["fields"]
        stamp = fields.get("timestamp_sample", fields.get("timestamp"))
        identity = (
            row.get("px4_pid"),
            stamp,
            fields.get("xy_reset_counter"),
            fields.get("heading_reset_counter"),
        )
        if identity in seen_samples:
            continue
        seen_samples.add(identity)
        if (
            not fields.get("xy_valid")
            or not stamp
            or any(fields.get(k) is None for k in ("x", "y"))
        ):
            unavailable["invalid_position"] += 1
            continue
        resets = tuple(fields.get(k) for k in ("xy_reset_counter", "heading_reset_counter"))
        if previous_reset is not None and resets != previous_reset:
            unavailable["frame_reset"] += 1
            break  # A new alignment is required; never conceal origin shifts.
        previous_reset = resets
        actual = interpolate(samples, stamp, "px4_us", "position", 200000)
        sim = interpolate(samples, stamp, "px4_us", "sim_ns", 200000)
        if actual is None or sim is None:
            unavailable["missing_truth"] += 1
            continue
        eligible = [r for r in references if r["emitted_wall_s"] <= row["received_wall_s"]]
        if not eligible:
            unavailable["missing_alignment"] += 1
            continue
        alignment = eligible[-1]["alignment"]
        counters = alignment.get("reset_counters")
        if counters is not None and tuple(counters) != resets:
            unavailable["frame_reset"] += 1
            continue
        estimate = world_xy([fields["x"], fields["y"]], alignment)
        error = math.dist(actual, estimate)
        estimation.append(error)
        detail = dict(
            px4_sample_us=stamp,
            isaac_sim_ns=sim[0],
            estimation_error_m=error,
            actual_world_xy=actual,
            estimate_world_xy=estimate,
        )
        target = interpolate(intents, sim[0], "sim_ns", "position", 250000000)
        if target is not None:
            intended.append(math.dist(actual, target))
            detail["intended_tracking_error_m"] = intended[-1]
        else:
            unavailable["missing_reference"] += 1
        index = bisect_right(received_times, stamp) - 1
        setpoint = received[index]["fields"] if index >= 0 else {}
        position = setpoint.get("position")
        if (
            position
            and all(v is not None for v in position[:2])
            and 0 <= stamp - setpoint["timestamp"] <= 250000
        ):
            # Applied setpoints are held between receipt events, never interpolated.
            target = world_xy(position, alignment)
            applied.append(math.dist(actual, target))
            controller.append(math.dist(estimate, target))
            detail["received_setpoint_error_m"] = applied[-1]
            detail["controller_position_error_m"] = controller[-1]
        else:
            unavailable["missing_received_setpoint"] += 1
        details.append(detail)
    stages = {}
    for kind, field in (
        ("feed_clock", "pipe_delay_s"),
        ("feed_map", "geometry_duration_s"),
        ("worker_map", "processing_duration_s"),
        ("cycle", "callback_duration_s"),
    ):
        stages[kind + "_" + field] = statistics(
            [r[field] for r in records if r["kind"] == kind and r.get(field) is not None]
        )
    controls = [r for r in records if r["kind"] == "control_timing"]
    for stage in sorted({name for row in controls for name in row.get("stages", {})}):
        for metric in ("wall_s", "cpu_s"):
            stages["control_" + stage + "_" + metric] = statistics(
                [row["stages"][stage][metric] for row in controls if stage in row.get("stages", {})]
            )
    return dict(
        schema=1,
        estimation_error_m=statistics(estimation),
        intended_tracking_error_m=statistics(intended),
        received_setpoint_error_m=statistics(applied),
        controller_position_error_m=statistics(controller),
        unavailable=unavailable,
        timing_stages=stages,
        control_rejections=[r for r in records if r["kind"] == "control_rejection"][-20:],
        deadline_misses=[r for r in controls if r.get("deadline_missed")][-20:],
        stop=stop,
        aligned_samples=details[-200:],
        interpretation="Estimation compares Isaac/PX4 at raw PX4 sample time. Intended tracking uses bracketed source-time reference samples; received setpoints use zero-order hold. Native receipt confirms PX4 reception, not motor execution. Arrival times never establish source-time equality. Gaps/resets/invalid estimates remain unavailable.",
    )


def read_status(project):
    try:
        status = json.loads(
            (Path(project) / "logs" / "timing_diagnostics.json").read_text(encoding="utf-8")
        )
        status["recording"] = bool(
            # Use shared QPC across Windows Python runtimes. Their monotonic()
            # implementations can have different epochs (GetTickCount vs QPC).
            status.get("recording")
            and -0.05 <= time.perf_counter() - status.get("updated_wall_s", 0) <= 3
        )
        return status
    except (OSError, ValueError):
        return dict(recording=False, reason="Timestamped recorder has not started")
