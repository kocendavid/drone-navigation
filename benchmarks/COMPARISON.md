# Native control comparison — 20 October 2026

The Python checkpoint is `639c9fe`. The original Python measurements
were completed before implementing C++. The final confirmation repeats Python
with the exact same harness hash used for the final C++ run. All runs use this
Windows host, ROS 2 Jazzy/Fast DDS, isolated domain 97, a 20 Hz target period and
12 seconds per ROS scenario (240 new commands from a Python ROS timer). There are 10,000 measured
timestamp/history iterations after 256 warmup iterations.

| Metric (95th percentile) | Original Python | C++ | Python / C++ |
|---|---:|---:|---:|
| Idle: late heartbeat gap beyond 50 ms | 12.559 ms | 0.528 ms | 23.81 |
| Idle: publication interval error | 12.559 ms | 49.853 ms | 0.25 |
| Idle: message construction + publication | 169.500 µs | 82.600 µs | 2.05 |
| Idle: new-decision scheduling delay | 14.355 ms | 5.097 ms | 2.82 |
| Idle: Python DDS callback observation | 602.100 µs | 419.500 µs | 1.44 |
| Python CPU load: late heartbeat gap beyond 50 ms | 12.648 ms | 0.475 ms | 26.60 |
| Python CPU load: publication interval error | 18.791 ms | 49.804 ms | 0.38 |
| Python CPU load: message construction + publication | 143.200 µs | 78.800 µs | 1.82 |
| Python CPU load: new-decision scheduling delay | 23.165 ms | 12.148 ms | 1.91 |
| Python CPU load: Python DDS callback observation | 521.600 µs | 406.500 µs | 1.28 |
| Trajectory clock update/consume/freshness | 3.300 µs | 2.200 µs | 1.50 |
| Source timestamp conversion | 4.700 µs | 1.800 µs | 2.61 |
| 128-sample reference history | 14.100 µs | 3.900 µs | 3.62 |

Lower latency/delay is better. The symmetric interval-error row needs the repeat
policy qualification below. A ratio below one otherwise means the observed C++
case was slower. The core replay produced the same checksum `1576530000000`
and 10,000/10,000 aligned samples on both implementations. The Python confirmation
loaded interval-error p95 was 18.813 ms, consistent with the original run.

The final C++ run uses the actual `NativePublisher.submit` path: Python still
generates the 240 commands on its ROS timer, subject to the same CPU/GIL load.
C++ publishes each update immediately and repeats the latest authorized command
after 50 ms without an update. This closes long heartbeat gaps but adds short
intervals when a fresh command arrives just after a repeat. It is not a fixed-rate
resampler and does not delay fresh references to force uniform spacing. Its
symmetric interval-error metric can consequently be larger. The idle/loaded C++
runs published 430/389
pairs for 240 new commands each; Python published 240 pairs each. New-decision
timing remains subject to Python scheduling.

A separate earlier native-timer-only experiment measured 0.388 ms
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

Native source/build fingerprint: `479032dabbb9cbe9334d9d6928e2d817f9d522102ce3049cace1118613ce60ce`.
Final harness fingerprint: `7f1bacbcdbc47665ac92b8a0a29706e7c95716941d89fe35dade2fd12575d7f5`.
Reports retain the checkpoint HEAD and mark the implementation worktree dirty;
the source/build fingerprint identifies the actual native code measured.

Raw data: [original Python](reports/python_before.json),
[Python confirmation](reports/python_confirm.json), [C++](reports/cpp_after.json),
[comparison data](reports/comparison.json).
