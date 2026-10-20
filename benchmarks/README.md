# Control publisher and timestamp benchmarks

The Python baseline was recorded at checkpoint `639c9fe`, before the C++
implementation. [Measured comparison](COMPARISON.md) includes the original run,
a Python confirmation, the native run, limits and source/build fingerprints.

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
Keep simulator and workload conditions the same for the paired runs. The saved
files in this directory are the original measurements; rerunning these commands
replaces them, so use other output filenames when preserving a comparison.

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
experiment is saved in `reports/cpp_timer_limit.json` and must not be presented
as an integrated pipeline measurement.

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
