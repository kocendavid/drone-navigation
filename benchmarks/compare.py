"""Render a comparison from saved benchmark measurements, without rerunning them."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    before = json.loads((ROOT / "reports/python_before.json").read_text())
    after = json.loads((ROOT / "reports/cpp_after.json").read_text())
    confirm = json.loads((ROOT / "reports/python_confirm.json").read_text())
    assert before["backend"] == confirm["backend"] == "python" and after["backend"] == "cpp"
    assert before["domain"] == after["domain"] == confirm["domain"] == 97
    assert before["iterations"] == after["iterations"] == confirm["iterations"]
    assert (
        before["seconds_per_scenario"]
        == after["seconds_per_scenario"]
        == confirm["seconds_per_scenario"]
    )
    assert before["core"]["checksum"] == after["core"]["checksum"] == confirm["core"]["checksum"]
    assert (
        before["core"]["aligned"]
        == after["core"]["aligned"]
        == confirm["core"]["aligned"]
        == before["iterations"]
    )
    assert confirm["harness_sha256"] == after["harness_sha256"]
    assert confirm["publication_mode"] == after["publication_mode"] == "python_submission"
    assert all(r["received"] == r["samples"] for d in (before, after, confirm) for r in d["ros"])
    rows = []
    for index, scenario in enumerate(("Idle", "Python CPU load")):
        for key, unit, description in (
            ("overdue_gap_ms", "ms", "late heartbeat gap beyond 50 ms"),
            ("absolute_interval_error_ms", "ms", "publication interval error"),
            ("publication_us", "µs", "message construction + publication"),
            ("scheduled_lateness_ms", "ms", "new-decision scheduling delay"),
            ("dds_observation_us", "µs", "Python DDS callback observation"),
        ):
            a = (
                max(0, before["ros"][index]["interval_ms"]["p95"] - 50)
                if key == "overdue_gap_ms"
                else before["ros"][index][key]["p95"]
            )
            b = after["ros"][index][key]["p95"]
            rows.append(
                dict(
                    metric=f"{scenario}: {description}",
                    units=unit,
                    python_p95=a,
                    cpp_p95=b,
                    ratio=a / b,
                )
            )
    for key, label in (
        ("clock_observe_consume_fresh", "Trajectory clock update/consume/freshness"),
        ("sample_clock_conversion", "Source timestamp conversion"),
        ("reference_record_and_interpolate_128", "128-sample reference history"),
    ):
        a, b = before["core"][key]["p95"], after["core"][key]["p95"]
        rows.append(dict(metric=label, units="µs", python_p95=a, cpp_p95=b, ratio=a / b))
    result = dict(
        schema=1,
        checkpoint=before["revision"],
        native_source_sha256=after["native_source_sha256"],
        harness_sha256=after["harness_sha256"],
        rows=rows,
    )
    (ROOT / "reports/comparison.json").write_text(json.dumps(result, indent=2) + "\n")
    table = "\n".join(
        f"| {r['metric']} | {r['python_p95']:.3f} {r['units']} | {r['cpp_p95']:.3f} {r['units']} | {r['ratio']:.2f} |"
        for r in rows
    )
    confirmation = confirm["ros"][1]["absolute_interval_error_ms"]["p95"]
    timer = json.loads((ROOT / "reports/cpp_timer_limit.json").read_text())
    timer_error = timer["ros"][1]["absolute_interval_error_ms"]["p95"]
    text = f"""# Native control comparison — 20 October 2026

The Python checkpoint is `{before['revision'][:7]}`. The original Python measurements
were completed before implementing C++. The final confirmation repeats Python
with the exact same harness hash used for the final C++ run. All runs use this
Windows host, ROS 2 Jazzy/Fast DDS, isolated domain 97, a 20 Hz target period and
12 seconds per ROS scenario (240 new commands from a Python ROS timer). There are 10,000 measured
timestamp/history iterations after 256 warmup iterations.

| Metric (95th percentile) | Original Python | C++ | Python / C++ |
|---|---:|---:|---:|
{table}

Lower latency/delay is better. The symmetric interval-error row needs the repeat
policy qualification below. A ratio below one otherwise means the observed C++
case was slower. The core replay produced the same checksum `{before['core']['checksum']}`
and 10,000/10,000 aligned samples on both implementations. The Python confirmation
loaded interval-error p95 was {confirmation:.3f} ms, consistent with the original run.

The final C++ run uses the actual `NativePublisher.submit` path: Python still
generates the 240 commands on its ROS timer, subject to the same CPU/GIL load.
C++ publishes each update immediately and repeats the latest authorized command
after 50 ms without an update. This closes long heartbeat gaps but adds short
intervals when a fresh command arrives just after a repeat. It is not a fixed-rate
resampler and does not delay fresh references to force uniform spacing. Its
symmetric interval-error metric can consequently be larger. The idle/loaded C++
runs published {after['ros'][0]['published_samples']}/{after['ros'][1]['published_samples']}
pairs for 240 new commands each; Python published 240 pairs each. New-decision
timing remains subject to Python scheduling.

A separate earlier native-timer-only experiment measured {timer_error:.3f} ms
loaded interval-error p95. That is a component capacity result, not the behavior
of the actual Python-to-C++ submission pipeline; it is preserved separately in
`reports/cpp_timer_limit.json` and is not substituted for pipeline measurements.

This compares complete implementations, including the C++ high-resolution Windows
waitable timer and native publication thread. It does not isolate language choice
from scheduling changes. Publication measurements include constructing and sending
the two ROS messages, not mapping, frontier selection or the full route-following
callback. Timestamp/history measurements include the Python/C ABI adapter cost but
exclude wire delivery. The background workload runs Python CPU work for 10 ms,
then waits for 10 ms, in another thread of the same process.
Private-topic timings exclude the additional duplicate-flight-publisher graph
lookup; that guard is exercised separately by native ROS regression tests.

The DDS observer remains Python in both tests. Latency uses the first observation
of each new command, not a later repeat. Callback observations can still wait for
the GIL; these values do not measure PX4 reception, network-only latency or motor
response. All 240 new targets were observed in every scenario. Planning and
position callbacks remain Python.

Isaac was stopped, the drone was disarmed, and benchmark topics were private.
No flight commands were sent to the lab's domain 0. These component results do not
establish improved flight tracking or hard real-time behavior. A moving-flight
comparison after the normal disarmed Reset is still required for those claims.

Native source/build fingerprint: `{after['native_source_sha256']}`.
Final harness fingerprint: `{after['harness_sha256']}`.
Reports retain the checkpoint HEAD and mark the implementation worktree dirty;
the source/build fingerprint identifies the actual native code measured.

Raw data: [original Python](reports/python_before.json),
[Python confirmation](reports/python_confirm.json), [C++](reports/cpp_after.json),
[comparison data](reports/comparison.json).
"""
    (ROOT / "COMPARISON.md").write_text(text, encoding="utf-8")
    print(table)


if __name__ == "__main__":
    main()
