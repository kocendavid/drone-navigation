> Archived teaching record from 19 October 2026. Current file responsibilities
> and function descriptions are in [the code walkthrough](../../TEACHER_GUIDE.md).

# Isaac / PX4 autonomous drone — teaching guide

Updated: **19 October 2026**.

This is the teaching record. [README.md](../../README.md) is the operation/maintenance reference.

## Purpose and collaboration

Build both a functioning drone and the student's ability to design, evaluate
and debug autonomous systems. The student owns meaningful engineering decisions;
the agent implements, runs repetitive commands, debugs routine issues and verifies.

Before important choices or unexplained behavior, ask for reasoning or a
prediction. Challenge missing assumptions and alternative explanations. Compare
predictions with measurements. Occasionally ask for an explanation or diagnosis
without immediately supplying the answer. Increase independence as understanding
grows. Do not turn every edit into a lesson or make the student relay commands.

Explain scripts and their state changes before execution. Use the authenticated
Isaac Python connection and named service commands; no computer use/UI clicking.
Respect the current experiment and existing authorization for flight/reset.

## System understanding

Isaac supplies physics and sensors. The mapper accumulates free, occupied and
unknown cells using capture-time transforms. Confirmed walls persist until Reset.
The frontier ledger remembers free/unknown boundaries and chooses new work by
evaluated flight-route distance through observed space. Reaching a viewpoint
records a visit; fresh scans must resolve its boundary before it is finished.
Filtering small frontiers does not prove that all rooms have been explored.

The planner prefers clearance and known-space centers. The route follower turns
a geometric path into position/velocity targets. PX4 estimates state and controls
the motors. Turning the nose does not instantly redirect translational velocity.

## Three different diagnostic questions

1. Tracking: is the drone following the requested reference?
2. Estimation: does PX4 agree with Isaac truth at matching source timestamps,
   after conversion into a common coordinate frame?
3. Timing: are observations and control callbacks arriving promptly?

A dashboard error label does not identify its cause. Source time is different
from arrival time. DDS timestamps have an XRCE adjustment. Missing brackets or
recording gaps are unavailable evidence, not zero error. Passing offline tests
does not establish live flight behavior.

## Lesson: clock pauses

At 17:09:25 and 17:10:00 Prague time, geometry construction occupied the clock
pipe reader for 187–188 ms. The mapper continued emitting clocks, but queued
updates reached ages of 265 and 281 ms, beyond the 250 ms watchdog. No reset of
the clock was recorded.

Clock ingestion and geometry processing now run on separate threads. Geometry
keeps only the newest waiting cumulative snapshot, retaining upstream scan
integration and wall confirmation. Failure/EOF still revokes readiness. A ground
check recorded maximum pipe delay of 16 ms. This fixes the observed blockage,
not every possible ROS delay or controller scheduling problem.

## Lesson: room transition, altitude and landing

At 17:28:01 a new route required braking from about 1 m/s. At 0.25 m/s²,
ideal stopping takes four simulated seconds and two metres. This is a planning
calculation, not a measured physical stopping guarantee. Wall time also depends
on simulation speed. A new geometric route cannot instantly redirect momentum.

At 17:28:07 the mapper rejected altitude: Isaac showed chassis height around
0.78 m above the configured floor, below its 0.80 m minimum. PX4 reported around
0.91 m local height. Those heights use different origins; source-aligned world-Z
conversion found about 0.20 m disagreement. Horizontal error was about 0.12 m,
below the tracking guard. A later return-home pause repeated the altitude issue.
Its cause remains unresolved; widening the gate would not explain it.

A separate check found the drone stationary on the floor in Isaac while PX4
remained armed in Land with `landed=False`. Physical floor contact, controller
landing detection and confirmed disarming are separate observations. This is
still an open issue.

## Student’s turn policy

Major turns brake to near-hover while yawing. Acceleration resumes when measured
**and** reference speed are ≤0.15 m/s and measured heading error is ≤30°.
Frontier updates cannot bypass an unfinished braking stage. Shallow changes
keep moving. Yaw rate is 60 degrees/s; yaw acceleration stays at 90 degrees/s².
Reference acceleration is 0.25 m/s². The earlier 0.35 m tracking latch caused
the failure below; the new certified-braking policy changes its live behavior.

## Latest live failure: 18:33:54 Prague time

Runtime `89f3557a335a` was loaded with the new turn policy and 60 degrees/s yaw.
`BRAKING` was active. This failure therefore cannot be attributed to old code.

The recorded reference speed dropped from 1.327 to 0.730 m/s in 0.084 simulated
seconds: about 7.1 m/s², despite the 0.25 m/s² limit. The final reference was
exactly 0.28 m from the latest received position. The target-lead clamp runs
AFTER acceleration limiting and can override that earlier limit. Published
velocity comes from final position advance, so it inherits the abrupt reduction.
The latest recorded DDS position observation was about 204 ms old in host time
at that step. This identifies a receipt gap, not where the delay originated.

The drone then moved ahead of the slowed reference and reached 0.351 m error.
Clock and map were healthy at the stop. The incident reports sequence gaps;
do not claim an exhaustive reconstruction of every intermediate command.
The near-hover change did not resolve the complete tracking problem.

## Implemented: scheduling separation and a feasible stopping segment

The student prioritizes preventing tracking faults rather than adding automatic
recovery, and chose to separate tracking work first. That separation is now
implemented, including blocked-worker publication
checks. It still requires loading through the next navigator restart and a
moving-flight comparison.

Planner workers prepare route footprints and speed profiles. The tracking callback
uses prepared cells to check the current map, retaining immediate movement and
LiDAR checks. Frontier analysis/ledger updates run in a background worker; selection
requires a result for the current map stamp. RViz publication, status-file output,
disk reporting and runtime fingerprinting also run outside the tracking callback.
Background work has bounded pending input and cannot mutate the follower or send
flight targets. Python threads do not provide a real-time scheduling guarantee.

The scheduling change alone did not repair the demonstrated target-lead clamp
discontinuity. The next change addresses the student's observation: a new route
must account for current momentum, not start with an instantaneous redirection.

A route starting at the current XY position can still be infeasible if it ignores
current velocity. The live navigator now reserves a straight braking segment
before a major turn. It checks stopping rays for both commanded and measured
world-frame velocity, plus the entire convex area between those rays, against
the latest observed body-free mask. LiDAR also checks stopping distance in the
motion direction. Reaction allowance is included. Unknown or out-of-map space
cannot certify a stop. Before normal acceleration, a preview checks that the
next reference still has observed stopping space.

The braking reference decelerates continuously at 0.25 m/s². The later 0.28 m
lead clamp cannot override that deceleration: if enforcing the lead limit would
cut velocity, the navigator starts the certified maneuver instead. Likewise,
present-time tracking error above 0.25 m requests braking before the old 0.35 m
latch. Error above 0.35 m remains visible as a diagnostic warning during a
certified maneuver; it does not pause exploration solely for that discrepancy.

Yaw turns concurrently. After reference speed reaches zero, measured speed is
≤0.15 m/s and position error ≤0.15 m, the follower requests replanning to the
retained goal. The 20:43:58–59 recording showed why the additional 0.5 s dwell
was wasteful: reference speed was already zero, but measured speed crossed
0.15 m/s by 0.00054 m/s, resetting the timer and reporting braking again despite
aligned yaw. Planning now requires neither that dwell nor provisional-goal yaw
alignment. A stopped reference reports settling until measured motion qualifies;
the fresh route supplies yaw coordination before acceleration.
Its existing stricter stable-hover gate (including speed ≤0.10 m/s) must also
pass before that planner launches. This happens without operator Resume.
Frontier reconsideration waits during the maneuver. The dashboard path shows
the reserved reference stopping segment; the full envelope is in state and
timestamped diagnostics. An onward route is provisional until replanning.

A small turn loop is not always
possible: radius = speed² / lateral acceleration. At 1.3 m/s and 0.25 m/s²,
the radius is about 6.8 m before reserving acceleration for changes in speed.
At 3 m/s, straight stopping requires 18 m plus a 1.5 m reaction reserve.

This is prevention and a planned handover, not automatic clearing of a fault.
Invalid/stale state, clock or frame data still stop navigation. A blocked or
unobserved stopping envelope is a real Blocked condition. Without the measured
velocity and full envelope checker, the pure follower retains the strict legacy
0.35 m pause policy; the goal navigator always supplies both. The simple response
model does not establish actual PX4 braking performance.

Verification: 316 Python tests pass, including the recorded clamp regression,
3 m/s 90°/180° handovers with modeled response lag, interior/diagonal obstacle
checks, the recorded near-hover timer reset, and the actual navigator callback's
settle → replan → navigate transition with unaligned yaw and the stable-hover gate.
These changes are prepared on disk; they have not been compared in moving Isaac
flight yet. The next experiment must verify the loaded fingerprint and compare
reference acceleration, measured stopping path, tracking warning and pause counts.

Reasoning checks, spread across experiments:

- Which constraint is violated when a later clamp defeats an acceleration limit?
- How might a stale position sample create unnecessary reference braking?
- What evidence should permit recovery, and what should keep it paused?
- When does a connecting curve fit, and when must the drone slow first?

## Key files and resources

| File                                                               | Purpose                                         |
| ------------------------------------------------------------------ | ----------------------------------------------- |
| [README.md](../../README.md)                                             | Launch instructions, terminals and settings.    |
| [navigation/mapping.py](../../navigation/mapping.py)                     | Ray mapping and confirmed-wall policy.          |
| [navigation/frontier_ledger.py](../../navigation/frontier_ledger.py)     | Frontier identity and completion memory.        |
| [navigation/planning.py](../../navigation/planning.py)                   | Known-space search and clearance preference.    |
| [navigation/motion.py](../../navigation/motion.py)                       | Motion reference and turn/tracking governors.   |
| [navigation/braking.py](../../navigation/braking.py)                     | Full stopping-envelope geometry and map checks. |
| [ros2_navigate.py](../../ros2_navigate.py)                               | Coordination, setpoints and pause policy.       |
| [navigation/feed.py](../../navigation/feed.py)                           | Clock reader and separate geometry processor.   |
| [navigation/timing.py](../../navigation/timing.py)                       | Source clock and freshness guards.              |
| [navigation/background.py](../../navigation/background.py)               | Bounded background work and lifecycle.          |
| [navigation/async_frontiers.py](../../navigation/async_frontiers.py)     | Frontier analysis outside tracking.             |
| [navigation/prepared_route.py](../../navigation/prepared_route.py)       | Prepared route cells and current-map checks.    |
| [bridge.py](../../px4_isaac/bridge.py), [model.py](../../px4_isaac/model.py)   | Motor forces and simulated sensors.             |
| [navigation/diagnostics.py](../../navigation/diagnostics.py)             | Source-time comparisons and evidence limits.    |
| [tools/record_timing.py](../../tools/record_timing.py)                   | Independent passive recorder.                   |
| [tools/analyze_timing.py](../../tools/analyze_timing.py)                 | Offline analysis without flight commands.       |
| [tools/lab.ps1](../../tools/lab.ps1)                                     | Named service management.                       |
| [config/navigation_settings.json](../../config/navigation_settings.json) | Experiment settings; verify loaded runtime.     |

Local evidence is ignored by Git; preserve it:

- [logs/current_run_timing_analysis.json](../../logs/current_run_timing_analysis.json)
- [logs/separated_clock_live_check.json](../../logs/separated_clock_live_check.json)
- [logs/tracking_and_room_transition_analysis.json](../../logs/tracking_and_room_transition_analysis.json)
- [logs/route_change_tracking_discontinuity.json](../../logs/route_change_tracking_discontinuity.json)
- [logs/near_hover_turn_checks.txt](../../logs/near_hover_turn_checks.txt)

## Validation and next experiment

The committed baseline passed 291 Python tests and 26 dashboard checks. The
later live failure exposes a case those tests did not establish. Offline checks
are evidence about software behavior, not a blanket claim of flight reliability.

Repeat the same room transition and retain maps, travel, targets and decisions.
Compare pre-switch speed, reference acceleration, measured velocity, observation
age, along/cross-track error, altitude disagreement and pause reasons. Change one
design assumption at a time. Record what was loaded, tested, observed and uncertain.
Scene placement takes precedence over a saved reset baseline. Reset ends on the
ground without automatic takeoff.
