# Navigation, history and control benchmarks

## History process boundary: current implementation

History now sends **compact telemetry at control-report frequency and map
snapshots only for selected frames**. `RunHistory.observe` selects frames before
enqueueing, using the same policy for local and subprocess history: first map,
phase/reason/goal changes, or new scans after five seconds of delivered simulation
time. Selection advances only after successful submission, so a rejected frame
is retried on the next observation. The subprocess never reselects frames.

Routine records contain the decision fields used by history, capture-time pose,
stamp and freshness. They retain no snapshot arrays, planned path, dashboard
trail or frontier catalog. Speed decisions and measured travel still follow
every report, including reports between map frames. Frames retain the complete
report and snapshot for map, footprint and cell-revision evidence. A stale map
can still be recorded on a transition or interval, with `map_fresh=false`;
freshness is captured at submission rather than recomputed after queue delay.

The control timer requests a 0.05-second period; preview-only mode observes every
0.25 seconds. The separate visualization writer's 0.5-second throttle is
independent of history. Serialization and pipe waits remain on the relay thread.

The FIFO has two configurable admission limits: **2,048 records and 64 MiB of
conservative retained payload size** by default (`queue_max_records` and
`queue_max_bytes` on `RunHistory`). Size accounting includes nested report data,
arrays and their retained bases, intentionally overcharging shared buffers
across records and potentially charging owned array storage twice. This bounds
queued payload retention; it is not a process RSS limit and excludes the one
in-flight RPC, serialization buffers and the child's history caches. Admission
is nonblocking. A full count/byte budget or an oversized record increments
`dropped_records`; selection does not advance. Dequeueing releases the charge.
History reports expose current/peak queue bytes and both configured limits.

Reproduce the history measurement without a ROS node or flight publisher:

```powershell
& C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi\envs\default\python.exe benchmarks/history_benchmark.py --output benchmarks/reports/history_current.json
```

The fresh [history report](reports/history_current.json) records source hashes,
runtime versions, settings and raw samples. The configured map is 320 × 265
cells. Its synthetic snapshot has all production geometry arrays and 720
hit endpoints; the accompanying report is deliberately compact. The snapshot
alone pickles to about **1.79 MiB**. The report also calculates the legacy demand
of sending that snapshot on every report (about **35.83 MiB/s at 20 reports/s**).
Current request demand is calculated from the actual selected frame and
telemetry messages, including eight-byte pipe headers. These are synthetic wire
demands, not measured live flight rates; production reports can increase them.

Across 240 reports advancing simulation time by 0.05 seconds per report, the
same-snapshot case records one frame, the new-scans case records three, and
alternating reason text records 240. The harness asserts that **full-map
requests equal the recorded frame count**, with compact telemetry in between.
It uses production `observe`/selection with direct subprocess calls for isolated
RPC timing. RPCs are unpaced after child startup and include pipe transfer,
processing, disk append and the response. These are background service times,
not control callback latency. The host also runs the active lab, so timings do
not establish a causal effect on control scheduling.

The harness separately runs the actual `RunHistory` queue at 20 wall-clock
reports/s for six seconds and checks that it drains, records two frames and
drops no records. It also asserts two full-map requests, 118 telemetry requests,
and queue memory below its budget, with charges released after drain. This short
check cannot establish sustained capacity for large reports, long histories or
slow disks. Fresh timings, payload sizes and queue depths remain in the JSON.

Remaining implementation costs and limits:

- Serialization and pipe waits run on a relay thread, keeping blocking RPC
  out of the control callback. Pickling still uses the parent process's CPU
  and can compete for the GIL, especially when transitions force many frames.
- The FIFO retains accepted telemetry rather than replacing pending samples
  with the newest one. The record limit still permits about 102 seconds of
  telemetry at 20 reports/s; the byte budget bounds payload retention.
- Events share this queue. When it fills, both states and events can be
  discarded; only the aggregate `dropped_records` count identifies the loss.
- Every reply returns the accumulated history summary and frame-offset index.
  These are bounded (200 events, 1,000 targets, 5,000 trail points, 80 frame
  summaries and 10,000 offsets), but reply size grows with the run.
- Frame creation encodes full grid/mask strings and writes JSON. Frequent
  reason changes bypass the usual interval and increase this disk/CPU cost.
  `close()` offers only a short flush/join window, so a deep backlog is also
  a recording-completeness risk at shutdown.

Regression coverage compares local and real subprocess recordings, including
compact intermediate reports, stale-map frames, map revisions, target/state
transitions and measured travel. Additional tests cover both admission limits,
oversized records, retained array views, byte release, blocked waiter wakeup
and retry after a rejected frame. Existing disk-backed frame retrieval and
speed-decision tests remain applicable. A running navigator loads this change
on its next normal reload; editing the source does not replace its active worker.

## Navigation and dashboard measurements

The navigation harness measures Python reference planning, map geometry,
frontier analysis and real localhost HTTP requests. It uses the production
C++ weighted A* and completion flood fill from `native_navigation/`.
The hybrid full-planner case replaces only
A*: shortcutting, smoothing, route scoring and safety checks remain Python.

```powershell
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c tools\benchmark_navigation.cmd --iterations 24 --requests 12 --rounds 2 --output benchmarks/reports/navigation_current.json
```

The launcher builds the production library in `.tools/native_navigation` in
Release mode. No ROS node or flight publisher is created. Synthetic grids use
the current navigation settings and cover open space, a detour and partially
mapped space. Randomized parity checks compare search cost, diagonal safety
and completion evidence; the three fixture routes must match the Python
planner. Native component timings include ctypes and array conversion costs.

Dashboard trials use the actual HTTP handlers and subprocess transport on
ephemeral localhost ports. Their command queues have no controller or flight
backend. Requests for state and a fake Hold acknowledgement start after the
preview worker enters a real planning job. Four cases compare shared/separate
workers and Python/hybrid planning. Command acknowledgements now encode inline
in every case. Repeated rounds reverse their order; raw
samples, medians and p95 values are retained. Startup/warmup is excluded.

Use fresh reports with matching source hashes when comparing implementations.
Earlier navigation results predate production integration and inline command
acknowledgements, so their old response timings do not describe current handlers.
The native implementation also uses dense arrays instead of Python dictionaries
and sets: these are implementation comparisons, not language-only comparisons.
Synthetic planning and HTTP workloads do not measure flight tracking or packet
loss. Historical numerical comparison tables have been removed from this guide.

## Publisher and timestamp measurements

Generate matched Python/native reports before making performance claims.
The [comparison guide](COMPARISON.md) describes the required evidence.

Run from the repository root, using the existing Pixi ROS environment:

```powershell
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c tools\build_native_control.cmd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c tools\benchmark_control.cmd --backend python --output benchmarks/reports/python_before.json
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c tools\benchmark_control.cmd --backend cpp --output benchmarks/reports/cpp_after.json
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c tools\benchmark_control.cmd --backend python --output benchmarks/reports/python_confirm.json
python benchmarks/compare.py
```

The launcher fixes `ROS_DOMAIN_ID=97`, Fast DDS and the installed profile. The
script refuses any other domain. It uses private `/benchmark/` topics; native
synthetic publication also refuses resolved flight-topic names. Do not run both
backends simultaneously or run heavy validation jobs during these measurements.
Keep simulator and workload conditions the same for the paired runs. These
commands replace their output files; use distinct filenames to preserve a run.

Default measurement parameters are 12 seconds for each idle/loaded ROS scenario,
20 Hz and 10,000 core iterations after warmup. `--seconds` and `--iterations`
override those values and must match between backends. The JSON files retain
platform/runtime data, percentiles, counts, replay checksum, harness hash and
native build hash. Measurements use high-resolution `perf_counter_ns()`/QPC,
not the coarse Windows Python `monotonic()` clock.

Both backends receive new commands from the same Python ROS timer. The C++ case
uses the actual submission API and records native publication times, including
authorized repeats. Late gaps beyond 50 ms and new-command publication cost are
distinct from decision scheduling and symmetric spacing error. `--timer-only`
instead measures native timer capacity without a Python producer; that separate
experiment must not be presented as an integrated pipeline measurement.

The comparison is about publication timing and source-time handling. Mapping,
route planning, physical tracking and actuator response are not measured here.
The native backend preserves the existing Python route decisions and validations.

Validation after building:

```powershell
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c tools\check_all.cmd
node tests/history_ui.test.cjs
```

Native tests compare clocks, resets, ambiguous epochs, stale data and history
against Python; use the real ROS library to check message fields and source-time
subscription; verify command expiry, revocation, duplicate-publisher prevention,
publication during GIL stalls, and safe navigator lifecycle. Native tests are
explicitly skipped when the library is absent; build it before interpreting the
test run as validation of the native backend.
