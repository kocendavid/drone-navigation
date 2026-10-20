# Native control comparison guide

Historical numerical tables have been removed. Generate fresh matched reports
using the commands in [the benchmark guide](README.md) before claiming a
performance improvement. Existing historical JSON files are not evidence for
the current worktree; compare their source fingerprints before reusing them.

Keep Python/native harness hashes, sample counts, scenario durations, simulator
conditions, domain and publication mode consistent. Retain runtime versions,
source/build hashes, raw samples, received counts and replay checksums. Separate
late heartbeat gaps, symmetric interval error and new-command scheduling delay.

The native publisher sends new commands immediately and can repeat an authorized
command when no update arrives. A fresh command just after a repeat can create
a short interval. Compare both long gaps and interval error; report publication
counts alongside the number of new commands. Python still schedules decisions.

A native-timer-only run measures component capacity. It cannot replace a run
through the actual Python submission API. Likewise, source timestamp/history
timings include adapter cost but do not measure wire delivery, PX4 reception,
physical tracking or actuator response.

The Python DDS observer can wait for the GIL in either backend. Use the first
observation of each new command and retain missing-message counts. Private-topic
benchmarks also omit the flight publisher's duplicate-publisher graph lookup;
that guard needs separate regression coverage.

The comparison renderer `compare.py` expects the four input filenames shown in
its source, including a separate timer-only report. Rerunning it regenerates a
dated comparison from those files; it does not validate that their hashes match
the current production code. Review provenance before publishing its output.

For the current full-map history transport assessment, see the
[history analysis and reproduction command](README.md#history-process-boundary-current-implementation).
