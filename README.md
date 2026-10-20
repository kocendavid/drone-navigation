# Isaac / PX4 drone navigation lab

A teacher and maintainer guide to a simulated quadcopter that uses LiDAR to
discover space, choose useful viewpoints and physically fly there through PX4.
Isaac Sim supplies the scene, sensors and physical response. The operator owns
scene editing and explicit takeoff/flight-mode selection through the dashboard
or PX4 console. The explicit explore-and-land
mission can request landing after confirming completion and saving its map.

The [teaching guide](TEACHER_GUIDE.md) explains each file's responsibilities,
key functions and connections through the system. Previous mentoring notes and
diagnostic findings are preserved in [teaching history](docs/history/TEACHING_NOTES_2026-10-19.md).

Updated **2026-10-19**. The current maze configuration covers 64 × 53 m at
0.2 m resolution, with floor Z near −3.98 m. Scene preparation and disarmed
Isaac/PX4 readiness have been verified in individual ground checks. Recorded
exploration runs are available. Clock/geometry separation is loaded and has a
ground transport check; the latest near-hover turn policy passes offline tests
and still needs moving-flight evaluation. Vertical estimation disagreement and
landing detection remain open issues. The current full Python suite passes 347 tests.
See [current maze setup](docs/MAZE_SETUP.txt) for the detailed procedure.

## Contents

- [Corner-aware exploration](#corner-aware-exploration)
- [optimizedRoutePlanner: return home](#optimizedrouteplanner-return-home)
- [Control the lab through code](#control-the-lab-through-code)
- [Explore, map and land](#explore-map-and-land)
- [What the student is building](#what-the-student-is-building)
- [Essential files](#essential-files)
- [What the terminals do](#what-the-terminals-do)
- [Frames, clocks and data](#frames-clocks-and-data)
- [Current settings](#current-settings)
- [Building and preparing a maze](#building-and-preparing-a-maze)
- [Running navigation and exploration](#running-navigation-and-exploration)
- [Teaching sequence and evidence](#teaching-sequence-and-evidence)
- [Troubleshooting](#troubleshooting)
- [Further reading and experiment history](#further-reading-and-experiment-history)
- [Verification and development](#verification-and-development)

## Corner-aware exploration

The explorer now looks for confirmed wall endings beside unknown space. A local
line fit supports horizontal, vertical and diagonal segments; isolated hits and
the middle of a straight wall do not qualify. These are **occlusion clues**, not
semantic room recognition or proof that a passage exists.

The current selector keeps a persistent list of connected free/unknown frontier
boundaries, with stable IDs while they overlap or move by a neighboring cell.
The **Frontier work list** shows pending, active, unreachable/deferred and resolved
work, along with estimated route lengths. New scans can split or merge boundary
identities; merged work continues under the surviving ID and is not marked explored.

When selecting new work, it prefers the shortest evaluated **planned flight
route** through known safe space, including the center/clearance preference.
It does not compare straight-line screen distance, nor the weighted A* cost as
if that cost were metres. A shortest-grid-distance search generates nearby safe
viewpoint candidates; up to three useful candidates per frontier are fully
planned and their resulting polyline lengths are compared. This is nearest
among evaluated viewpoints, not a proof of the globally shortest continuous tour.
An active frontier and its useful goal are retained during travel, even if a
different frontier would now be closer. Routes to that goal may still recenter.

Reaching a viewpoint records a visit, not completion of a frontier. Its boundary
must disappear from three distinct accepted map scans before it becomes resolved.
Repeated/stale/invalid scans cannot add confirmation. Hold and a new Explore in
the same experiment retain the ledger and visited viewpoints; Reset clears them.
The ledger is included in map/decision frames and the UI records lifecycle updates.
The map draws each frontier boundary and labels its measured span. Only boundaries
with principal-axis span at least **1.5 m** get exploration targets; viewpoints
favor the middle of that boundary over its nearest endpoint. This span is not
a measurement of doorway clearance. Smaller boundaries remain **filtered,
unresolved**, can become eligible as observations expand, and still prevent
claiming complete exploration or automatically landing. Amber squares remain
wall-ending clues, independent of the frontier boundary display.
The existing viewpoint/stall limits can still stop a run with unresolved work;
that is not a declaration that all rooms were explored.

Wall-ending clues remain available to explain occlusions, but no longer override
the nearest-route ordering requested for systematic exploration.
Every destination and route still passes the observed-space and obstacle-margin
checks. No geometry is inserted into the map by this estimate. A doorway can be
known clear while the room beside the far face of its wall remains unseen.

The map shows amber squares at wall-ending clues and “Look around” at the clue
associated with the selected viewpoint. **The square is on the wall; it is not
the drone's destination.** The exploration panel and recorded history include
the selected clue, estimated hidden area and selection reason. Estimates use
sparse rays over a 3 m neighborhood, not a prediction of the full room's area.
As actual LiDAR fills a room, that area ceases to count as unknown. Existing
travel/viewpoint history discourages revisiting the same observation positions.
Continuous reconsideration has no switching-radius rule; frontier identity and
fresh observation evidence determine whether to keep or retire current work.
Optional destination and recentering updates preserve a short prefix of the
current route. The planner starts the new tail at the prefix endpoint rather
than at the drone's current position. Its length is two simulated seconds of
current reference speed, with a 0.5 m minimum and the remaining route as its limit;
`exploration_commit_horizon_s` configures the horizon (maximum 5 s).
The prefix follows existing bends. Applying the result preserves the current
position/velocity reference and trims samples already flown; a result whose
join point has been passed is discarded. Both prefix and tail must pass the
latest body-free map checks. New obstacles, invalid inputs, completion, Hold
and explicit operator goals still take priority. Speed remains subject to
curvature, yaw, acceleration and stopping limits throughout the committed path.

This is a first geometric heuristic: short/thick walls, noisy geometry or
unconfirmed endings can be missed, and two-room behavior needs a live run.
It does not classify rooms or guarantee complete coverage. The independent
enclosure/completion checks remain responsible for declaring exploration done.

Fresh map analysis also checks completion during travel. Once the connected
observed space is enclosed with no boundary to unknown, Explore clears its
remaining destination, hovers and reports **COMPLETE**, even if the final
viewpoint was never reached. Exhausting useful targets with unknown boundaries
still reports unfinished exploration. Explore and land begins its existing
fresh-scan verification and map-saving sequence before requesting landing.

## optimizedRoutePlanner: return home

### Known-space centers and return preview

Routes now favor **clearance over the shortest geometric distance**. A distance
field measures room to both mapped walls and the boundary of observed space.
Its ridges appear as the optional pale center overlay. This is the center of
currently known space, which changes as new observations arrive; it does not
estimate a room centroid or assert that unknown space contains a wall.

A* adds penalties near those boundaries and away from the detected center ridges:
`1 + weight * (scale / (clearance + scale))² + weight * (distance_to_center / (clearance + scale))²`,
with weight 8 and scale 2 m. The earlier clearance-only cost did not use the
displayed centerlines directly. The center term now guides both A* and shortcut
selection. This can select a longer, wider corridor. It is a preference, not a rule that
every point must lie on a center ridge: endpoints, narrow passages and the
existing near-wall start escape still need to connect. Shortcutting preserves
the combined center/clearance objective while comparing several shortcut and
rounding candidates by estimated flight time, speed variation and clearance cost.
The selected route may spend at most 5% more center/clearance cost than the
conservative 2% shortcut baseline. All hard observed-space/footprint checks remain.
Exploration, manual routes, previews and
Return to start share this planner. Frontier ranking still uses geometric
reachability distance, so its reported metres are not weighted route cost.
During exploration, each fresh-map reconsideration can also update the route
to the same destination when its combined motion/clearance score improves by
at least 5%. This is recorded as `route_recentered`, without counting a new
destination. It avoids retaining an edge-following route solely because its
frontier is still useful as more of the corridor becomes known.

The **Return to start** card is at the top of the controls. **Preview return
route** is read-only: it draws a gold dashed route to H and displays its length,
without starting flight. **Return to start** commands the real return and
replans against the then-current map. Start capture requires the first stable
mapped hover; before that, the visible card explains why its buttons are
unavailable. A reset clears home and any pending or displayed return preview.
Known center masks are included in new map-history frames; old frames have none.

**Return home** saves and returns to the first stable, aligned 1 m hover point
in the current experiment. The map marks that point **H**. It is captured once;
later goals, exploration sessions and Hold do not move it. Restart experiment
or restarting the navigator clears it along with the map. It is not a persistent
GPS home or an automatic landing command.

The button can interrupt exploration during flight. The controller cancels
exploration and its automatic landing mission, commands a hover, waits for
measured motion to settle, then plans a fresh route through currently observed
clear space. It uses A* with no diagonal corner cutting, then removes redundant
waypoints only where a straight segment preserves the obstacle margin. The
usual speed/braking limits, smooth yaw and live map/LiDAR checks remain active.
New obstacles trigger the same stop/replan/resume behavior as every other planned
route, retaining home as the destination. If no safe mapped route exists, it holds
as BLOCKED and retries; it never invents a passage or reports a successful return.
Home arrival requires the measured position to
remain within 0.15 m at a stable hover for one simulated second. Use **Land here**
separately if you want to land.

Because the return planner uses the accumulated map rather than replaying the
outbound travel history, it can skip exploration detours and use known
shortcuts between connected rooms. A* minimizes distance weighted by clearance;
candidate scoring then favors gentler, faster routes within a bounded clearance
tradeoff. This is not a guarantee of the mathematically
shortest continuous route or fastest flight: clearance, turns, acceleration and
new obstacles still affect the result. Unknown space remains excluded even if
it would make a shorter path. Home capture, its selected goal, route changes,
interruptions and arrival are included in decision history/map frames.

## Control the lab through code

### Ground readiness and repeatable vehicle physics

The bridge uses a **1.5 kg generic simulated vehicle**, configured by
`vehicle_mass_kg` and the nine `link_masses_kg` entries in `bridge_config.json`.
Those entries preserve the relative mass distribution measured from the asset;
the source densities otherwise produced about **251,537 kg** at the composed
scene scale. This is a lab model choice, not a measured physical drone mass.
The bridge verifies that the total runtime PhysX mass matches the configuration
before calibrating motor thrust.

`rigid_frame: true` makes the eight arm/rotor joints fixed in the session layer.
The force model assumes fixed rotor positions and simulates rotor speed and
reaction torque itself. Allowing the source asset's links to hinge caused
ground bouncing and large IMU spikes; correcting mass alone did not remove that
motion. With the rigid frame, the disarmed vehicle settled with zero linear and
angular velocity and PX4 preflight checks passed in the live recovery check.
These session overrides leave the scene file and referenced asset unchanged and
are reapplied by the bridge loader after a restart. Disarmed zero-wrench calls
are skipped so they do not keep waking a resting articulation.

Reset now requires PX4 preflight OK, a valid near-zero local ground position,
low estimated velocity, fresh actuator feedback and a physically stable upright
vehicle for ten checks before reporting READY. It does not arm or take off.
A latched position-origin reset is reported directly as a reason to reset,
rather than presented only as missing telemetry. A successful ground check is
not a substitute for validating the changed mass/frame model in flight.

The lab can be operated through command-line processes and Isaac's installed
Python execution server. Desktop automation is unnecessary. The operator pastes
one bootstrap into Isaac's Script Editor per application session; afterward the
agent can run scene inspections and bridge scripts directly and read their output.
Explain each script's purpose and state changes before running it. Scene and
flight observations should be discussed with the student as part of the lesson.

### One-time connection for an Isaac session

[tools/enable_lab_control.py](tools/enable_lab_control.py) enables NVIDIA's
`isaacsim.code_editor.python_server` extension on `127.0.0.1:8226`, requires
authentication and records the session token in ignored `logs/isaac_control.json`.
It changes the execution-server settings and extension state. It leaves scene
geometry, physics, drone pose, timeline, arming and flight modes unchanged.
An authenticated local client can execute Python inside Isaac, including later
explicit scene edits and Play/Stop commands. Keep the session file private.

Run this in Isaac's Script Editor:

```python
import runpy
runpy.run_path('C:/Users/finel/Documents/ChatGPT/issaac/tools/enable_lab_control.py')
```

Wait for `[Lab control] READY`. To inspect the scene/timeline/bridge connection
from the workstation, use [tools/isaac_client.py](tools/isaac_client.py):

```text
python tools/isaac_client.py --status
python tools/isaac_client.py --run tools/inspect_vehicle_runtime.py
```

The first command reads status. The second runs a read-only vehicle inspection.
`--run` can execute other Python scripts, and `--code` executes explicit source;
their effects depend on that source. Client requests are recorded in ignored
`logs/lab_control_audit.jsonl`, without the authentication token. The connection
ends when Isaac closes. Disabling `isaacsim.code_editor.python_server` also ends it.

### Terminal services

[tools/lab.ps1](tools/lab.ps1) reports the XRCE agent, Zenoh router, mount TF,
PX4 and navigator. Start reuses existing services, refuses duplicates and redirects
new hidden processes into `logs/lab_*.out.log` and `logs/lab_*.err.log`.

```powershell
powershell -NoProfile -File tools/lab.ps1 status
powershell -NoProfile -File tools/lab.ps1 start all
powershell -NoProfile -File tools/lab.ps1 stop navigator
```

The service command does not arm, take off, switch modes or change Isaac's
timeline. Starting the navigator starts its flight-target stream; PX4 follows
that stream only in Offboard. Stop requires disarmed PX4 or native Hold and
accepts one named service at a time. Reusing a running process retains its loaded
code; after changing source, reload it in Hold. Status identifies processes,
while sensor freshness and PX4 health still require separate checks.

Managed PX4 runs with `-d` (no interactive shell). PX4 client commands remain
available through WSL, for example:

```text
wsl -d Ubuntu --exec /home/finel/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander --instance 0 status
```

### Folding-arm baseline and verified recovery

[tools/lock_drone_arms.py](tools/lock_drone_arms.py) runs in stopped Isaac and
backs up the root layer before setting the four `rotor_arm0..3` hinge limits to
zero. It preserves propeller joints and placement. On 2026-10-14, the direct
inspection found the vehicle upside down and two arm hinges still at ±30°.
After locking those hinges and restarting, the disarmed vehicle settled upright,
bridge traffic was fresh and PX4 reported **Preflight check: OK** and
**Ready for takeoff**. This verifies ground readiness, not maze flight.
The scene edits remain in the open stage until saved.

The connection was verified through live status, script execution and Play/Stop.
Two isolated client tests verify authenticated requests, TCP half-close,
fragmented responses and rejection of nonlocal or unauthenticated endpoints.
Service checks verify reuse of existing processes and rejection of stops in
armed Offboard or with unavailable PX4 status.

## Explore, map and land

### Repeat an experiment from the UI

**Restart experiment** restores Isaac's timeline-start drone pose in the current
open scene, clears the map, visited viewpoints and measured trail, and starts
fresh PX4, DDS and motor/sensor-bridge sessions. The dashboard remains open and
shows reset progress. The reset ends on the ground, disarmed; it does not arm,
take off or start exploration automatically.

The map display clears immediately when you request a reset. Physical reset is
asynchronous: the status shows the Hold handover, simulator/service restart and
ground-state checks. It is complete only when the reset reports READY. An empty
display alone does not mean the vehicle has returned to its starting pose.

The previous map and run history are archived under `logs/experiment_*/`.
`run.json` preserves the controller state and trail; `reset.json` records the
reset result. The stopped starting pose and scene identity are stored in
`logs/experiment_baseline.json` as a record. **Scene placement takes precedence:**
each reset uses the authored starting pose restored by Isaac Stop, including
your placement edits, and updates this record. The previous baseline is archived
as `previous_baseline.json`; it never overrides the scene or blocks an edited
starting position. Maze edits and
locked-arm settings remain in the open scene; the original scene file is not
reopened or overwritten.

The sequence confirms native PX4 Hold before removing flight publishers, then
stops Isaac, restarts PX4/DDS, reloads the bridge and resumes simulation. Three
checks must confirm fresh actuator data, a stable upright vehicle and PX4
Disarmed before declaring reset complete. Failures remain visible in the
dashboard; a retry remains available. A reset is a simulation operation that
interrupts the current flight.

Core source: [navigation/reset.py](navigation/reset.py) manages the bounded
reset job; [navigation/dashboard.py](navigation/dashboard.py) exposes the
authenticated button; [ros2_navigate.py](ros2_navigate.py) replaces the ROS node
and mapping worker while keeping the HTTP server alive.

Live verification on 14 October 2026: two consecutive resets restored the same
upright start at approximately (−28.104, −17.159, −3.553) m. Each cleared mapping
and exploration history, left PX4 Disarmed with Preflight check OK, and kept the
dashboard connected. Mapping intentionally waits for the next stable 1 m hover.
Eight reset tests cover sequencing, repeatability guards, service failures,
authenticated retry and Windows background-service output handles.
The Hold request also precedes the Isaac inspection, so an unresponsive simulator
connection cannot delay requesting native Hold while an experiment is moving.

The **Explore and land here** button starts one mission from a stable 1 m
Offboard hover. It chooses frontier viewpoints, travels through observed space
and incorporates fresh LiDAR scans. When no useful viewpoint remains, it checks
completion independently of the viewpoint-selection heuristic.

The completion rule applies to the connected fixed-height space containing
the drone. Known cells must form an enclosure bounded by observed occupied cells,
with no adjacent unknown cells and no contact with the configured map edge.
The initial footprint remains an explicit clear-space assumption. Three distinct
fresh scans over at least one delivered simulated second must support the result.
Map or position faults reset confirmation. A temporary map gap during travel
can recover after three fresh aligned scans and one simulated second of stable
hover; the mission replans before moving. Frame disagreement, tracking faults,
telemetry loss and operator Hold require intervention. This does not prove
the interior of sealed rooms behind walls or other altitude levels was explored.

If unknown boundaries remain, candidate exhaustion, slow progress and the
viewpoint limit produce **INCOMPLETE** and Hold. They never authorize automatic
landing. **Hold here**, a manual goal or leaving Offboard cancels the mission.

After completion is verified, the controller saves a timestamped directory
under `logs/mission_*/` containing `map.json`, `map.pgm` and `map.yaml`.
The JSON preserves observations, assumed cells and completion evidence. The
PGM/YAML pair is compatible with ROS map loading and preserves unknown space.
Saving must succeed before the controller sends PX4 `VEHICLE_CMD_NAV_LAND`.
The drone lands at its current position, as requested for this flat-floor maze.

The controller retains its Offboard stream until PX4 actually enters Land mode,
then stops competing hover targets. The mission reports **LANDED** only after
fresh landed and disarmed telemetry. Rejection or an unconfirmed handover reports
an incomplete mission. PX4 owns the descent. A new flight needs a new controlled
startup; the mission does not rearm the drone automatically.

### Watch the map in RViz or the browser

The [browser](http://127.0.0.1:8787/) shows the map, frontier choices, mission
phase, reason and saved-map directory. RViz uses the same map evidence through:

| Topic              | ROS message                 | Contents                                                          |
| ------------------ | --------------------------- | ----------------------------------------------------------------- |
| `/navigation/map`  | `nav_msgs/OccupancyGrid`    | Observed free/occupied cells; unknown and assumed cells remain −1 |
| `/navigation/path` | `nav_msgs/Path`             | Current planned route                                             |
| `/navigation/pose` | `geometry_msgs/PoseStamped` | World-frame drone pose                                            |

These topics use Fast DDS/domain 0, reliable transient-local delivery and frame
`world`. The earlier raw-LiDAR RViz window uses Zenoh; launch the map preset in
the controller's middleware environment with:

```cmd
C:\Users\finel\Documents\ChatGPT\issaac\tools\rviz_navigation.cmd
```

[config/navigation.rviz](config/navigation.rviz) provides a top-down map view.
Observer mode also publishes these visualization topics without flight targets.
Closing either viewer leaves the controller running.

Core source: [navigation/mission.py](navigation/mission.py) defines completion,
exports and landing state; [navigation/ros_map.py](navigation/ros_map.py) publishes
the RViz messages; [ros2_navigate.py](ros2_navigate.py) coordinates the mission.
The baseline and mission integration pass 167 tests, including false-completion,
map-save failure, interrupted proof, ROS map layout and native landing handover.
Live whole-maze completion still needs a flight demonstration.

The live farthest-frontier run reached one inspection point, observed 60.48 m²
of new area, and selected its next destination independently. This verifies
target selection, measured travel and repeated observation; it does not yet
verify whole-maze completion or automatic landing.

The command/acknowledgment interface follows the
[PX4 ROS 2 control documentation](https://docs.px4.io/main/en/ros2/offboard_control).
Installed PX4 message definitions and the local decoder determine this build's
timestamp and topic-version behavior.

### How it chooses its own targets

Targets are green viewpoints beside the green/gray boundary: a **frontier**.
A far green point next to an already mapped wall provides little new information.
The explorer first requires a useful view of unknown space and a route entirely
through observed space with vehicle clearance.

It then favors unvisited viewpoints and chooses the furthest eligible one by
travel distance through the map. A wall detour counts toward that distance;
straight-line distance through a wall does not. Expected new observed area
breaks ties. Distance remains a preference within known safe space.

It records reached inspection points and its measured trail. A viewpoint near
the travelled trail receives lower priority, and reached viewpoints within
0.5 m are excluded. Reusing a corridor to reach a new frontier is allowed.
After arrival, it waits for a new scan and repeats with the updated map.
The browser displays the chosen reason, a pale measured trail and inspection
points; RViz also receives `/navigation/trail` as `nav_msgs/Path`.
Completed missions save `history.json` beside their maps. Memory belongs to the
current exploration session; Pause/Resume retains it.

Movement checks current LiDAR endpoints for obstacles along the next segment.
When the forward strip has no endpoint, motion requires a fresh map to certify
the complete remaining route with an observed vehicle footprint. Missing
endpoints alone never certify free space. A real close obstacle still overrides
the map and stops the reference.

For every planned route (manual goal, exploration or return home), a newly detected
obstacle that invalidates the remaining route triggers a hold and a request for a
replacement route against the latest map. Planning waits for measured motion to
settle. Navigation automatically resumes when the replacement passes the current
observed-space and obstacle-clearance checks. An unreachable manual/home goal
remains BLOCKED and retries at most once every two wall seconds; exploration can
retire an unreachable frontier and select another from the updated map.
A temporary map or simulation-clock gap with an active goal also holds; automatic
replanning requires three fresh aligned scans over at least one simulated second
at a stable hover with healthy telemetry and clocks. Operator Hold cancels the
route; tracking, alignment, position and control-callback faults require manual
intervention.

The earlier gain-per-route-length policy favored short moves. This experiment
now uses the furthest-frontier policy requested during the lesson.

### Re-evaluating during travel

The **Re-evaluate frontier targets while flying** checkbox enables a new
experiment. The selector runs in a background thread every two delivered
simulation seconds when a fresh scan is available, while the current route
continues. A different selected destination replaces the route only after its
vehicle footprint is validated against the latest map. The streamed reference
position remains continuous. No switching radius or improvement threshold is
applied; an identical destination cell is simply the same target. Existing
visited-viewpoint memory and physical obstacle-clearance rules still apply.

Uncheck it to use the previous arrival-only policy. Reset to the same starting
conditions between runs before comparing policies. The earlier run started
from a different pose/map and is not a controlled baseline for this experiment.

Initial live observation on 14 October 2026: 12 voluntary target switches,
12.36 m of measured travel and 102.88 m² newly observed, with no reached
viewpoint yet. This supports frequent switching, but does not establish lower
efficiency without a controlled arrival-only comparison. That first trial
paused when new scans invalidated its destination. The controller now retires
an obsolete exploration destination and selects again, while pose and sensor
faults still pause. A later UI Explore command retained the map and reset the
session counters; those measurements belong to a separate trial.

Startup coordinate alignment now requires at least two simulated seconds of
continuous stable hover and three distinct fresh scans. This avoids calibrating
from a brief low-velocity moment during takeoff. An observed 25 cm offset
disappeared when the alignment was recaptured at settled hover, with no scene
change or relaxation of the 20 cm map/flight-controller disagreement guard.

The UI reports new observed area, measured XY distance, new area per metre,
simulation time, target evaluations and switches. Distance sums measured
displacements of at least 2 cm; the displayed trail is sampled more sparsely.
Area counts observed free and occupied cells, excluding the assumed starting
footprint. These are measured observations, not a percentage of the maze's true
area. The current target's potential counts unknown cells in sparse rays up to
3 m around that viewpoint, stopping at known occupied cells. It is optimistic:
unknown geometry is not confirmed until actual LiDAR rays observe it.

Wall observations are currently binary cell labels, not per-cell confidence
scores. Completion requires an enclosed connected observed component supported
by three fresh scans; it does not mean every wall cell was individually observed
three times. Unknown gaps are retained rather than filled to force enclosure.
The mapper traces free space up to a 12 m horizon. A valid farther return
(within the installed sensor's 200 m far range) still clears the observed segment
up to that horizon; dropping the whole ray used to leave artificial unknown
wedges. Only actual returns within the mapping horizon mark occupied cells;
the cutoff never becomes a synthetic wall. Actual selected endpoints remain
available for forward-clearance checks. Missing beams and space beyond the
horizon stay unknown. The 10 m corridor width does not by itself require a
close wall inspection. A changed viewpoint is useful when it resolves occlusion
or missing observations.

## What the student is building

The experiment asks: can a simulated quadcopter use its LiDAR to discover a
room, choose somewhere useful to inspect, and physically fly there with PX4?

The system has two nested feedback loops:

**Navigation loop:**

```text
Isaac LiDAR + capture-time TF
  -> point cloud -> world ray map -> frontier choice -> A* route
  -> position reference -> PX4 -> Isaac movement -> next LiDAR view
```

**Flight loop:**

```text
Isaac IMU/GPS/barometer/magnetometer -> MAVLink simulator bridge -> PX4 EKF
  -> position/attitude/rate control -> motor commands -> bridge rotor forces
  -> Isaac physics -> next simulated sensor measurement
```

The map, planner and explorer choose WHERE to go. PX4 decides how to move the
motors. Isaac computes the physical response. The browser displays results
and accepts the operator's intent. ROS is the message transport between parts.

This is a deterministic geometric baseline, not a trained neural network.
Localization currently uses Isaac's known world pose. This is mapping with
known localization, not SLAM: the drone is not estimating its world pose from
LiDAR yet. A maze tests occlusion and decision-making without changing that.

## Essential files

Read these in order to follow settings, perception, planning and flight control.
File links are relative to this repository.

### [config/navigation_settings.json](config/navigation_settings.json)

The experiment's launch-time parameters: reference speed, distance bands,
obstacle margin, sensor near range, target lead and exploration goal limit.
Changing this file does not reconfigure an already running controller.
Changing near_range_m also requires applying the sensor script in Isaac.

### [navigation/settings.py](navigation/settings.py)

Loads and validates the settings. Rejects a stop boundary too close to the
sensor blind range, inconsistent margins and unsupported speed settings.

### [navigation/mapping.py](navigation/mapping.py)

RayMap converts scans into UNKNOWN, FREE, OCCUPIED and START_AREA cells.
Valid rays clear space before their measured endpoint; endpoints mark walls.
Space behind walls stays unknown. geometry() computes body clearance and
the preferred obstacle margin. No route is allowed to assume unknown free.
Read update() and geometry() together to see how perception becomes planning.

### [navigation/exploration.py](navigation/exploration.py)

- `frontiers()`: free cells touching an unknown cell.
- `visible_unknown()`: estimates information available from a viewpoint.
- `choose_frontier()`: ranks viewpoints, checks routes and chooses a useful goal.
- `ExplorationSession`: selection, travel, arrival, scan, pause and finish.

It chooses a viewpoint inside known traversable space, standing back from
the unknown boundary. It does not command the drone to fly blindly into gray.

### [navigation/planning.py](navigation/planning.py)

plan() finds an A* route through the preferred-clearance mask. Diagonal
corner cutting is rejected; line-of-sight shortcuts are checked again.
A nearby-wall starting hover can escape by increasing obstacle clearance.
Selection can fail even with unknown cells left: they may be unreachable,
excluded by clearance, already visited, or missed by bounded candidate search.

### [navigation/motion.py](navigation/motion.py)

RouteFollower turns the route into slowly advancing position targets.
Distance bands, acceleration, bend speed, final braking and target-lead
limits are applied here. Actual tracking error can pause movement.
FrameAlignment converts between world XY and PX4 local North/East positions.
The installed bridge uses an ENU/NED axis swap; heading bias is NOT a
position-axis rotation. This distinction matters over metre-scale routes.

### [ros2_navigate.py](ros2_navigate.py)

The current experiment coordinator and sole PX4 setpoint publisher.
It receives PX4 position/status/IMU periods, owns the map feed and dashboard,
runs planning in a background thread, sends the 20 Hz Offboard heartbeat,
holds altitude, turns smoothly toward route segments, and advances the exploration session.
Starting the program never arms or takes off. Explicit dashboard flight buttons
can request native takeoff, Offboard and landing. The explore-and-land mission requests native
PX4 Land after confirmed completion and successful map persistence.

During navigation the nose anticipates the upcoming route curve, with a
60°/s yaw-rate limit and 90°/s² reference acceleration. At major direction changes,
it brakes toward near-hover while yawing, then accelerates once both measured and
reference speed are at most 0.15 m/s. Translation can resume while yaw catches up:
the facing-speed cap blends continuously from cruise at 30° misalignment to
0.7 m/s at 60° and above (bounded by the configured cruise).
The shortest angular error handles ±180° wrap. Captured estimator heading bias
corrects the yaw target independently of the world/NED position-axis conversion.
Hold, blocked/settled phases, stale maps and timing faults freeze the heading
reference. Turn limits use delivered simulation time. The Measured state panel
shows actual and commanded compass heading; the trace and decision frames retain
those values. New settings are `yaw_rate_deg_s` and `yaw_acceleration_deg_s2`.
This replaces the older fixed-heading policy after reloading the controller.

### [navigation/flight_controls.py](navigation/flight_controls.py)

Handles the operator's Take off to 1 m, Enable navigation and Land here clicks.
Native PX4 calls run outside the flight-target callback. Takeoff verifies one
Isaac SITL process, disarmed status and `MIS_TAKEOFF_ALT=1`, then uses PX4's
native takeoff command, including its normal arming checks. It never force-arms.
Fresh grounded telemetry is required for takeoff; stable hover, an aligned map
and an established target stream are required before enabling Offboard.
Command delivery is separate from completion: the UI waits for observed hover,
Offboard/READY, or landed/disarmed status before reporting success. Hold/Reset
cancel pending requests; Reset's native Hold is serialized after any already
dispatched native command. Slow prechecks cannot dispatch a takeoff/Offboard
request older than five seconds. A flight-state timeout reports failure without
retrying the native command or claiming the vehicle has stopped.

### [navigation/feed.py](navigation/feed.py), [tools/navigation_map_worker.py](tools/navigation_map_worker.py)

The worker subscribes to Isaac's Zenoh cloud/clock/TF and builds the map.
The parent receives bounded JSON snapshots over its owned process pipe.
The parent uses Fast DDS for PX4. Same ROS domain does not connect different
middleware implementations; the local pipe is the intentional connection.
Stale data, lost worker output or mismatched frames remove navigation readiness.
Do not launch the worker manually: the navigator owns its lifetime.

### [navigation/dashboard.py](navigation/dashboard.py), [config/navigation.html](config/navigation.html)

A localhost server and browser canvas, respectively. The canvas draws the
map, pose, route, frontier boundary and selected goal. Operator commands are
queued for ros2_navigate.py; the web server does not publish PX4 commands.
Observer mode rejects flight commands. The browser is not the flight clock:
closing its tab does not stop an active navigator or exploration session.

### [ros2_hover.py](ros2_hover.py), [ros2_step.py](ros2_step.py)

Shared foundations: hover messages, position freshness/reset guard and the
delivered-IMU simulation pacer. Their standalone hover/one-step programs are
earlier lessons. Do not run them alongside the current navigator as a second
flight-target publisher. Importing their helpers does not run those programs.

### [load_bridge.py](load_bridge.py), [bridge_loader.py](bridge_loader.py)

Operator-run Isaac entry point and source loader. Loads a fresh bridge from
disk while Isaac is stopped, replacing a previous bridge instance.

### [bridge_config.json](bridge_config.json), [px4_isaac/bridge.py](px4_isaac/bridge.py)

Vehicle/sensor/physics configuration and Isaac callbacks. The bridge applies
motor forces to the dynamic chassis and samples the actual physical state.
It does not teleport the drone along the navigation route.

### [px4_isaac/model.py](px4_isaac/model.py), [px4_isaac/transport.py](px4_isaac/transport.py)

Rotor dynamics and sensor conversions; MAVLink UDP transport, respectively.
Rotor coefficients are preliminary. Transport freshness and disarm cut thrust.

### [start-isaac-px4.cmd](start-isaac-px4.cmd), [px4/start-isaac-sitl.sh](px4/start-isaac-sitl.sh), [px4/simulator-udp.sh](px4/simulator-udp.sh)

Windows-to-WSL launcher and external-simulator setup. Use PX4 - Isaac SITL.
The SIH launcher runs separate internal physics and is the wrong simulator
for this experiment. PX4 must be linked to the physical Isaac vehicle.

### [tools/prepare_maze_scene.py](tools/prepare_maze_scene.py)

Runs in stopped Isaac. Backs up the root layer, enables static maze collisions,
disables the old room and reports chassis position/orientation. It does not move
the drone, start the timeline or save the edited scene automatically.

### Installed resources

| Resource              | Workstation path                                           |
| --------------------- | ---------------------------------------------------------- |
| Project               | `C:\Users\finel\Documents\ChatGPT\issaac`                  |
| Isaac installation    | `X:\isaacsim\6.1.0`                                        |
| Saved scene inspected | `C:\Users\finel\Desktop\scene.usd`                         |
| ROS/Pixi environment  | `C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml`            |
| PX4 message overlay   | `C:\px4-tools\ros_ws\install\setup.bat`                    |
| Fast DDS profile      | `C:\px4-tools\fastdds.xml`                                 |
| XRCE agent            | `C:\px4-tools\agent-v2.4.3\bin\MicroXRCEAgent.exe`         |
| PX4 WSL source        | `/home/finel/PX4-Autopilot`                                |
| PX4 binary            | `/home/finel/PX4-Autopilot/build/px4_sitl_default/bin/px4` |
| Generated startup     | `/home/finel/.local/share/px4-isaac/rcS-isaac`             |

Unsaved scene edits can differ from the saved USD; save a separate maze scene.
requirements.txt describes bridge Python dependencies; this lab uses the
existing Isaac and Pixi runtimes instead of installing a new Python stack.

## What the terminals do

### Keep running during flight

**PX4 - Isaac SITL**

WSL flight controller and estimator. The pxh> prompt accepts commander and
status commands. This is not a Windows prompt. Keep one PX4 instance.

**PX4 - XRCE DDS Agent**

Bridges PX4's lightweight DDS client to the Windows ROS/Fast DDS network.
Current launch: MicroXRCEAgent udp4 -p 8888. The agent version is 2.4.3.
One agent is enough; it serves the position/status and target/heartbeat link.

**ROS 2 Jazzy - Zenoh Router**

ROS middleware router serving the Isaac sensor/TF network.
Current process: ros2 run rmw_zenoh_cpp rmw_zenohd, domain 0.

**LiDAR mount TF terminal**

tools/lidar_mount_tf.cmd publishes chassis -> point_cloud mounting geometry.
Current offset: (0,0,0.0591487379025617) m with aligned axes.
Re-measure if the sensor is moved. A frame_id string alone is not a transform.

**ROS - Isaac Goal Navigator**

tools/ros2_navigate.cmd runs ros2_navigate.py. It owns mapping and the local
web server. The default `--control-backend cpp` uses a C++ component as the sole
PX4 target/heartbeat publisher; Python retains planning and route validation.
Keep it running. Its map-worker child is automatic and needs no extra window.

Build the native component in the existing ROS/Pixi environment before launching:

```powershell
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c tools\build_native_control.cmd
```

Source lives in `native_control/`; generated libraries live under ignored
`.tools/native_control/`. Each build has a source fingerprint and a versioned
filename, so rebuilding does not overwrite a loaded Windows DLL. The loader
rejects stale or missing builds rather than silently choosing another backend.
The normal disarmed Reset starts a new navigator process and loads the selected
build. `tools/ros2_navigate.cmd --control-backend python` retains the original
publisher and time implementation for comparison or rollback. Observer mode
does not load the publisher. Benchmark method and results are in
`benchmarks/README.md`.

The native component owns ROS message construction/serialization, a dedicated
publication thread with a 50 ms repeat interval, XRCE timesync subscription,
Isaac trajectory-clock state, source-clock conversion and reference history. It uses a high-resolution
Windows waitable timer and does not depend on the Python GIL to repeat a target.
Each validated Python update publishes immediately; repeats never advance the
reference position or invent a new route.
Python updates can arrive just after a repeat, so actual publication frequency
can exceed the nominal 20 Hz decision rate. This avoids adding a resampling delay.
Permission expires 100 ms after the Python validation callback started.
Invalid, old or revoked commands cannot renew
it; landing, stale telemetry, duplicate publishers and shutdown revoke it.
The native clock guard independently clears velocity/acceleration when trajectory
time becomes stale. A rejected update pauses navigation and preserves the dashboard;
an unpublished reference is not entered into history. The component never arms
the drone or changes PX4 modes. Body-footprint, map and route checks remain in Python.

This reduces publisher scheduling variation, not the cost of Python planning or
the drone's physical response time. It does not provide a hard real-time guarantee
on Windows. Reported `control_backend` and `native_publisher` counters identify
the active backend, repeats, expirations and rejected commands.

Isaac also stays running and playing. The bridge runs inside Isaac, not in an
extra terminal. The LiDAR, /clock and world/body TF graphs run inside the scene.

### Optional viewers and diagnostics

RViz: sensor/frame viewer. Current presets are config/lidar_raw.rviz,
config/lidar_world.rviz and config/lidar_clearance.rviz. The new occupancy map
and planned path are available through Fast DDS using `config/navigation.rviz`
and `tools/rviz_navigation.cmd`. The raw-cloud presets use Zenoh. Closing either
viewer stops no flight.
Browser http://127.0.0.1:8787/: display and operator controls; not a heartbeat.
tools/px4_state_probe.cmd: bounded read-only PX4 ROS delivery/state check.
tools/lidar_probe.py: bounded cloud/time/frame inspection; see docs/history/LIDAR_SETUP.txt.
tools/forward_clearance.cmd: earlier recommendation-only clearance experiment.
tools/ros2_step.cmd and tools/ros2_hover.cmd: earlier flight publishers; stop
before launching the navigator. Native PX4 Hold is the handover mode.

## Frames, clocks and data

Scene metres, Z up. World/map XY use ENU (East, North, Up).
Chassis/sensor convention is FLU (Forward, Left, Up).
PX4 local position uses NED (North, East, Down); target z=-1 means 1 m above
PX4's local origin. The current maze floor is near -3.98 m, read from
config/navigation_settings.json. --floor-z can override it at launch. A changed
scene requires a fresh simulator/bridge/PX4 startup and a new map.

Isaac Zenoh/domain 0 provides /lidar/points (PointCloud2), /clock and /tf.
The mapper obtains capture-time world <- sensor and chassis <- sensor TF.
PX4 Fast DDS/domain 0 provides versioned /fmu/out position/status/IMU topics.
Commands are /fmu/in/offboard_control_mode and /fmu/in/trajectory_setpoint.
Version suffixes depend on installed px4_msgs; the code derives them.

The navigator heartbeat uses steady computer time at 20 Hz (standalone hover and
step tools remain at 10 Hz). The navigator's position progression and velocity
feedforward use the same elapsed Isaac `/clock` time. The Zenoh mapper forwards
small clock records through its session-checked pipe independently of full maps;
the Fast DDS controller does not assume it can subscribe directly to Zenoh.
Delivered IMU periods remain a separate freshness watchdog and conservative
session counter. Counting only delivered IMU periods for trajectory motion while
commanding full velocity made the two references disagree when packets were lost.
Clock resets, gaps above 0.25 simulated seconds, or stale clock records hold
motion rather than catching up; three progressing samples reconfirm the clock.
Between sparse clock publications, the previous validated velocity remains
streamed only while IMU integration advances. Tracking compares current telemetry
against the prior reference advanced by its velocity over the known elapsed
simulation interval; it also records the unadvanced-target error. Otherwise a
0.13 s physics batch at 3 m/s alone looks like a false 0.39 m tracking fault.
Slow Isaac physics therefore makes a flight
take longer in real time rather than compressing its target trajectory.
Commands use timestamp=0 because this installed PX4 decoder assigns receipt
time. Sensor timestamps are XRCE-adjusted and must not be copied into commands.
That receipt policy is specific to the installed decoder, not a universal API rule.

Frame validation compares map pose with the closest trusted PX4 position sample
at cloud receipt, within 0.25 s of host time. Disagreement limits remain 0.2 m
and 15 degrees. This accommodates mapping delay; approximate receipt-time matching
does not establish exact source-time synchronization or hide localization errors.

## Current settings

| Parameter                                                 | Prepared value                                              |
| --------------------------------------------------------- | ----------------------------------------------------------- |
| Cruise with adequate clear straight route                 | Up to 1.20 m/s                                              |
| Full-cruise forward obstacle clearance in braking model   | About 4.13 m                                                |
| Approach between 1.0 and 0.65 m                           | Nominal taper toward 0.08 m/s; braking may lower it         |
| Clearance at/below 0.65 m, or no usable observed corridor | Hold/replan                                                 |
| Reference acceleration / bend speed cap                   | 0.25 m/s² / 0.15 m/s                                        |
| Braking reaction allowance                                | 0.50 simulated seconds                                      |
| Yaw rate / yaw acceleration                               | 60°/s / 90°/s²                                              |
| Heading error above 30°                                   | Travel cap 0.70 m/s; heading alone does not request a stop  |
| Major travel-direction change / replan launch             | Brake to stopped reference / stable hover ≤0.10 m/s         |
| Lead / proactive tracking braking / warning               | 0.28 m / 0.25 m / 0.35 m; certified braking may exceed lead |
| Preferred obstacle margin / assumed vehicle radius        | 0.80 m / 0.30 m                                             |
| Sensor near range / starting-area prior                   | 0.50 m / 0.60 m                                             |
| Map dimensions / resolution                               | 320 × 265 cells / 0.20 m                                    |
| Map origin                                                | (−34, −56) m                                                |
| Map extent                                                | 64 × 53 m                                                   |
| Floor Z                                                   | −3.9803731091662193 m                                       |
| Maximum reached exploration viewpoints                    | 20                                                          |
| Optional route-update commitment horizon                  | 2.0 simulated seconds of current speed; minimum 0.5 m       |

Distances are from the chassis/reference centre, not from rotor tips. The stop
threshold is a decision trigger, not a measured stopping-distance guarantee.
These speeds are reference limits in simulated seconds. The controller may
advance more slowly when actual tracking lags. The dashboard separately shows
reference and measured horizontal speed. A polyline bend/end can also slow it.
The prior and body radius are assumptions; the start area must actually be clear.

The selected cruise cap is now 1.2 m/s. Green cells indicate observed traversable geometry, not permission
to maintain maximum speed. The speed cap reserves reaction travel and stopping
distance: `v × reaction_time + v² / (2 × deceleration) <= clearance − stop_distance`.
With this profile full cruise needs about `0.65 + 1.2 × 0.5 + 1.2²/(2 × 0.25) = 4.13 m`
clear ahead. The reference brakes before entering the bend zone and before goals.
An endpoint-free forward strip requires fresh-map certification of the whole
remaining route; missing sensor returns alone never permit motion. The certified
stopping policy below governs tracking and lead-limit interventions. This kinematic bound
does not establish PX4's measured stopping distance or guarantee cruise speed
in a short corridor. Acceleration alone needs 4.8 simulated seconds to
reach 1.2 m/s from rest. Goal, bend, tracking and yaw caps can reduce it further.

Turn speed now follows effective curvature rather than stopping at a 12°
waypoint-angle threshold. The 0.15 m/s² lateral budget requires a 9.6 m radius
to retain 1.2 m/s; a long 90° arc can therefore be faster than a short 15° kink.
Rounding seeks that radius within observed safe space. Replanning can blend a
new tail tangent to the immutable committed prefix, and intermediate curve
samples no longer clip a longer control step to one sample's distance.
The [turn-speed investigation](docs/TURN_SPEED.txt) records the formulas,
remaining handover guards and reproducible offline sweep. The new behavior
still needs a moving PX4/Isaac flight evaluation after navigator reload.

The navigator now sends finite horizontal **velocity feedforward** equal to the
validated position reference's actual advance per simulated second. Position
remains the primary Offboard mode; height remains position-only. The installed
PX4 position controller adds this velocity to its position-error correction.
Without this term, the small target lead and current position gain can keep
actual speed far below the requested cruise. Healthy callbacks between clock/IMU
batches retain the validated velocity even when neither stream delivered a new
sample in that callback; they do not advance reference position. Requiring new IMU
progress on every callback caused intermittent zero-feedforward pulses. Retention
expires with Isaac clock freshness (0.25 wall seconds) and also requires fresh IMU
timing, valid navigation state and current map checks. Hold, blocked/stale maps,
timing gaps and stale/stopped simulation send zero horizontal feedforward. The
timestamped reference log marks `feedforward_held_between_samples`. A fresh native
landing still owns descent.

Horizontal **acceleration feedforward** now accompanies position and velocity.
It is the vector change in validated reference velocity divided by the same
consumed simulation interval, rotated into local NED without a position offset.
Its magnitude must stay within the configured 0.25 m/s² bound; discontinuities
omit acceleration feedforward instead of amplifying them. Braking supplies
negative acceleration. Healthy callbacks between samples retain the last valid
acceleration with velocity. Hold, invalid/stale inputs and stopped phases clear
both; vertical acceleration stays unset. No PX4 gains were changed. This is
acceleration feedforward, not a new jerk-limited trajectory generator.

The lead calculation intersects the route direction with a 0.28 m circle
around measured position. If enforcing that boundary would cut reference velocity,
the live navigator starts a certified braking maneuver instead of clamping the
position after acceleration limiting. The lead bound is not an instantaneous
constraint during that maneuver. No PX4 gains or acceleration parameters were changed. Offline
tests cover reference/response lag, bends, frame conversion, stale inputs and
stopping. A simplified position-plus-velocity response reaches about 3 m/s on
a 100 m test route; this is not a live Isaac validation or evidence that the
current 58 m maze will allow full cruise. The earlier ideal occluded-room replay
remains separate evidence about exploration, not high-speed flight.

For a route change over about 45 degrees relative to current reference or measured
motion, turn coordination now reserves a stopping segment in the current travel
direction. `navigation/braking.py` checks both stopping rays and the entire convex
area between them against the latest observed body-footprint mask; LiDAR checks
the available stopping distance too. Unknown and out-of-map space cannot certify
a stop. Normal acceleration previews the next stopping envelope, starting braking
before it would run beyond certified space.

During this maneuver the reference decelerates at the configured 0.25 m/s² without
the late position clamp. Frontier switching waits and yaw turns concurrently.
Reference speed must reach zero, measured speed must be ≤0.15 m/s and position
error ≤0.15 m before the follower requests a fresh route to the retained goal.
There is no additional settling timer or yaw gate before this request: yaw must
follow the fresh route rather than its provisional goal bearing. Once reference
speed is zero, remaining measured motion is reported as settling, not renewed braking.
The planner's stable-hover gate (speed ≤0.10 m/s and height checks) still applies.
The fresh route's yaw speed limits apply before acceleration. No Resume command
is required for this normal handover. Shallow updates keep travelling.
Reached intermediate samples advance before the major-turn check, so a zero-length
direction at a sample cannot initiate a braking handover. True turns and stopping-space
checks still apply to the onward segment. Healthy callbacks with no new simulation
time preserve yaw reference and rate; Hold, stale clocks and pacing gaps clear the rate.
Yaw looks along the upcoming curve. Speed retains full yaw permission within
30 degrees of measured yaw error. Above 30 degrees, a separate 0.7 m/s ceiling
applies, including at 90 or 180 degrees; heading error alone never stops travel.
The former heading-only 60-degree braking trigger and near-hover alignment wait
are removed. Sharp changes of travel direction still require momentum braking;
obstacles, tracking limits, curves and destination settling can still stop motion.
The planner rounds corners with tangent quadratic curves, trying cuts up to 3 m
on each leg (previously 1.5 m), only when every sampled segment
preserves observed safe space and the center/clearance cost within 2%. Otherwise
it retries smaller curves, then keeps the original corner and brakes for it.
Curved samples have curvature
and backward braking speed bounds; they are not individual stop points.
A signed tracking governor reduces reference speed for lag or lateral error.
With measured velocity and the envelope checker, its soft checks use the same
tracking reference as the hard tracking guard. Forward speed is capped at the measured forward component
plus an allowed closing speed: relative reaction travel and braking distance must
fit within a 0.20 m predicted separation budget, including lateral error. This
leaves 0.05 m before the 0.25 m guard and applies the configured reaction time and
acceleration bound. Bounded steady lag can retain cruise; growing lag limits actual
reference acceleration rather than only multiplying the cruise ceiling. Lateral
divergence still reduces the cruise cap. The relative-motion model assumes the
current measured forward speed; it cannot guarantee against sudden vehicle braking.
With fresh measured velocity and the envelope checker, error above 0.25 m also
requests certified braking before the former scalar latch. During a certified
maneuver, error above 0.35 m is logged as `tracking_warning`, not a Pause by itself.
The dashboard displays the reserved reference stopping segment, and state/diagnostics
include `braking_envelope_xy`. Blocked/unobserved stopping space and invalid or stale
state/time/frame inputs still stop navigation. The pure follower's fallback without
a measured velocity vector and envelope checker retains its strict 0.35 m pause.

Live tracking can now compare the PX4 position sample with a reference at that
sample's source time. The Isaac bridge sends clock-only pairs (native HIL sensor
microseconds and Isaac simulation nanoseconds) over nonblocking localhost UDP
port 18789. Navigation never consumes simulated physical ground truth through
this channel. Three consistent advancing pairs confirm the mapping; a reset,
session change, offset jump, duplicate or stale pair cannot retain alignment.
The navigator subscribes to XRCE `TimesyncStatus` and removes its signed clock
offset from `VehicleLocalPosition.timestamp_sample`, checking the corresponding
timesync epoch and rejecting ambiguous offsets. Arrival time is not substituted
for sample time.

A bounded reference history uses the exact Isaac clock stamp consumed by each
control callback. It interpolates gaps up to 0.25 simulated seconds. Telemetry
ahead of the last published reference may use at most 0.10 seconds of explicitly
labelled constant-velocity prediction. Samples more than 0.25 seconds behind or
0.10 seconds ahead of the consumed clock are rejected for alignment. Clock pairs
expire after 0.25 wall seconds; timesync reports after 2 wall seconds. History is
cleared across navigation phases, source-clock epochs, frame and DDS clock resets.
When conversion/history is unavailable, the previous conservative prediction and
tracking guards remain active; the sample is not reported as aligned or zero error.
The raw 0.28 m lead and complete stopping-space checks remain in force too.
Activation requires the normal disarmed Reset, which reloads both bridge and navigator.

Every received navigation report also records a compact `type: "speed_decision"`
row in the current `logs/decisions_*.jsonl` run history, without creating an extra
visible event or map frame. Phase/reason changes record stop decisions too;
unchanged idle states do not produce continuous history speed rows. The dashboard,
map frames, `navigation_state.json`, periodic `navigation_trace.jsonl` and the
timestamped recorder's control-rate `reference` records include the same
`speed_decision` breakdown. Disk writes remain on the existing history/reporting
workers, while timestamped diagnostics use the best-effort nonblocking emitter.

Speed-decision fields distinguish:

- `requested_cruise_m_s`: cruise ceiling after any caller restriction.
- `caps_m_s` and `reductions`: obstacle approach/braking, route/goal/corner,
  tracking and yaw ceilings, with each effective reduction and its source.
- `selected_speed_m_s`: desired speed before acceleration and displacement limits.
- `expected_speed_m_s`: resulting reference speed; `current_speed_m_s`: latest
  measured drone speed, or null when unavailable.
- `published_feedforward_speed_m_s`: the speed actually sent as feedforward,
  which can be zero during a clock/watchdog hold even with a retained reference.
- `action`, `reference_acceleration_m_s2`, `acceleration_limited` and
  `ceiling_binding`: whether the command accelerated, decelerated, stayed steady
  or waited for clock progress, and whether the reduced ceiling actually bound it.
- Tracking lead/cross error, raw/predicted position error, prediction interval,
  heading error, waypoint distance and braking cause provide the decision context.
- `tracking_response` records the measured-speed-based ceiling. Measured forward
  speed, current/allowed closing speeds and remaining lead budget explain it.
- `tracking_alignment` identifies exact, interpolated, source-time-predicted or
  fallback comparison, with source stamps, sample age and unavailability reason.
  `tracking.reference_time_aligned` identifies whether that tick used the mapping.
- `acceleration_reference_ned_xy` in state and `acceleration_ned_xy` in reference
  diagnostics record the sent horizontal acceleration; omitted values are null.

`dominant_limit` identifies the final effective ceiling reduction; an acceleration
limit can still keep the current command below that ceiling. A "Slowing for
tracking lag" event therefore displays expected/current speeds and the actual
command action, making continued acceleration visible. Values are in metres and
simulated seconds. Speed reports still pair the latest measured speed with the
command; the tracking alignment fields describe position-error timing separately.

Offline regressions cover the recorded 1.327 → 0.730 m/s clamp discontinuity,
3 m/s 90°/180° handovers with response lag, occupied/unknown stopping space and
automatic settle/replan through the navigator callback. These changes require a
navigator reload and moving-flight comparison. They do not establish measured
PX4 stopping distance, jerk-limited motion, or a globally optimized maze trajectory.

## Building and preparing a maze

The original 8 m room used a 12 x 12 m map centred near (0,0). The operator's
maze was roughly 58 x 18 m. The added room extends below the old Y=-27 map edge.
The configured map now spans world x=-34..30, y=-56..-3, with 320 x 265 cells at
0.2 m resolution. The larger canvas does not import scene walls or pre-fill
free space; only observed LiDAR rays populate it. Map resolution, dimensions,
origin and floor height are launch-time settings. See docs/MAZE_SETUP.txt for
collision preparation and the restart procedure after rebuilding the scene.

Reset now exits the Python navigator after the disarmed reset succeeds; its
launcher starts a fresh process so changed source and settings are actually
loaded. The previous history is copied into the reset archive and restored for
review/download. A source/configuration fingerprint warns in the UI while
unloaded edits exist. Reset never arms or starts exploration.

Map status is emitted at the existing 5 Hz wall-watchdog rate. Geometry fields
are reused when the occupancy and confirmation masks have not changed, even if
the scan counter advances. Map age limits remain unchanged. On a transient
data-age gap, active exploration holds and may replan automatically only after
three fresh aligned scans over at least one simulated second and a stable armed
Offboard hover. Pose faults, tracking faults, worker failure and operator Hold
do not receive this automatic recovery. Stop records include publication,
cloud and clock ages to distinguish a data gap from an estimator disagreement.
The expanded map uses SciPy's exact Euclidean distance transform to avoid the
previous pairwise NumPy distance-field cost. Install `requirements-navigation.txt`
in the ROS navigator's Python environment; the current lab has SciPy 1.18.1.
NumPy/scalar fallbacks remain for comparison, and tests verify equal distances.
See the [SciPy transform documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.distance_transform_edt.html).

Use 2.2 m or wider clear corridors initially. The preferred 0.8 m margin on
both walls consumes 1.6 m before discretization; narrower passages can be
rejected even if a point-sized drone could fit. Width is the free wall-to-wall
gap, not the wall centre spacing. Wall thickness around 0.15-0.3 m is convenient.
Keep a flat unchanged floor, walls at least 1.5 m above it, collisions enabled,
and LiDAR-visible surfaces. Avoid stairs/low tunnels: this planner is fixed-height
2-D, and assumes vertical clearance through the route. It does not plan Z motion.

Suggested first scene: an L-shaped corridor, a second turn into a hidden room,
and a visible dead-end branch. Start with a clear 0.6 m hover area. From the
starting view the sensor should see the first wall/corner, not the whole maze.
End the maze with a closed room/wall so finite returns are available; the mapper
does not turn a missing/no-return beam into verified free space.

Save a separate maze USD. Preserve /World/quadcopter/chassis, sensor path,
physics, ROS sensor/clock/TF graphs and sensor mounting. For a major rebuild,
land, stop Isaac and rebuild the scene; this is a world/origin change, not an
ordinary publisher reload. Reinitialize the bridge/PX4 as described in [the maze setup](docs/MAZE_SETUP.txt).
Start a fresh navigator/map after changing geometry so old free cells are absent.
The operator owns scene changes, takeoff, mode selection and landing.

### Why the rebuilt scene needs a fresh setup

The new maze floor is near Z = −3.98 m, while the old room floor was near −0.77 m.
After the rebuild, the bridge log stopped updating, but PX4 still reported its
old Offboard state and a simulator connection timeout. That mode label did not
prove control of the vehicle in the new scene. The saved maze also lacked
authored collisions and the old room remained active.

Prepare the physical geometry, then restart the bridge and PX4 together so the
position estimate and sensor connection match the new scene. Start a fresh map
as well. Ordinary movement needs no restart; a connected hover can use native
Hold for a publisher reload.

See [docs/MAZE_SETUP.txt](docs/MAZE_SETUP.txt) for the preparation and restart
procedure. On 2026-10-14 the operator's preparation output confirmed 11 colliders,
the old room disabled, floor Z −3.9803731091662193 and an upright chassis.
After the folding-arm correction, fresh PX4/bridge communication and preflight
readiness were verified. Live exploration remains pending.

## Running navigation and exploration

With the simulator bridge and lab services running, the dashboard's **Flight
setup** section shows current arming/mode and the next required step:

1. **Take off to 1 m** explicitly arms the simulated drone and requests native
   takeoff. Wait for the stable 1 m hover and live-map confirmation. Mapping and
   its drone arrow start at this flight height; a ground map is not yet valid.
2. **Enable navigation** hands the hover to PX4 Offboard. Wait for observed
   Offboard and READY; a queued mode request is not confirmation.
3. **Explore unseen space**, **Explore and land here**, or select a destination.

**Land here** requests native landing without requiring a complete map. It waits
for landed and disarmed reports. **Hold here** stops exploration; while a native
flight button is pending, it also cancels that operation and requests native
Hold. **Restart experiment** returns to the starting pose and ground state.
No flight command is sent merely by opening or refreshing the dashboard.
The new explicit flight-button sequence is checked against isolated fake
outputs; its first operator-driven live flight remains to be verified.

The PX4 console remains an alternative for manual setup/recovery. New Python
source requires a navigator reload in native Hold or while disarmed.

If already airborne with a connected bridge and only reloading the program:

1. At the PX4 `pxh>` prompt, select native Hold:

   ```text
   commander mode auto:loiter
   commander status
   ```

   Keep the previous flight publisher running until status reports **Hold**.

2. Press Ctrl+C in the old ROS navigator; answer `Y` if asked to terminate the batch job.
3. In the ROS Jazzy/Pixi terminal run:

   ```cmd
   C:\Users\finel\Documents\ChatGPT\issaac\tools\ros2_navigate.cmd
   ```

4. Reload [the dashboard](http://127.0.0.1:8787/). Wait for **READY** and a fresh map.
5. At `pxh>`, select Offboard and verify the reported mode:

   ```text
   commander mode offboard
   commander status
   ```

6. Click **Explore unseen space** once. **Hold here** cancels exploration.

For a new scene, establish the bridge using docs/BRIDGE_SETUP.txt, then use the
Flight setup buttons above or the manual commander takeoff/Offboard sequence.
READY in the terminal means target readers/heartbeat are established;
the browser also requires fresh map, valid alignment and a stable Offboard hover.

Observer-only preview (no PX4 publishers):

```cmd
C:\Users\finel\Documents\ChatGPT\issaac\tools\ros2_navigate.cmd --observe
```

Do not run observer and controller on the same port. A separate observer may
use --port 8788. The browser controls stay disabled in observer mode.

A session selects -> travels -> reaches -> waits for a NEW scan -> selects again.
It pauses on sensing/frame/tracking faults; Resume requires restored valid data.
It finishes when no useful reachable candidate is found, after three viewpoints
with less than 0.08 m² added each, or after the configured 20 reached viewpoints.
Finishing holds position. It does not imply every cell of a maze was discovered.
Manual goals and Hold cancel exploration. Leaving armed Offboard cancels it too.

## Teaching sequence and evidence

### Engineering collaboration

The aim is both a working drone and the student's ability to design, evaluate
and debug autonomous systems. The student owns meaningful engineering decisions;
the agent handles implementation, repetitive commands, routine debugging and
verification. Desktop/UI automation is excluded from this workflow. Explain a
script's purpose and state changes before executing it; use the authenticated
Isaac connection and service tools instead of making the student relay commands.

- At architecture choices, unexpected behavior and material trade-offs, first
  ask for the student's explanation or prediction. Challenge missing assumptions
  and competing explanations without immediately supplying the answer.
- Keep routine implementation moving. Do not turn every edit into a lesson or
  interrupt for decisions the agent can reasonably make.
- Compare occasional predictions with recorded outcomes. Periodically ask the
  student to explain a component or diagnose a hypothetical failure, increasing
  difficulty and reducing guidance as understanding improves.
- Distinguish an observation, a proposed cause and a verified cause. A dashboard
  label, command acknowledgement or passing unit test alone does not establish
  physical flight behavior.

### Latest lesson: timing, tracking and a room transition — 19 October

Three different comparisons matter: requested versus actual position measures
following error; PX4 estimate versus Isaac truth measures estimation disagreement;
clock/update age measures whether the controller has timely information. Compare
positions at matching source timestamps and in a common coordinate frame before
attributing a difference to communication or controller tuning.

The first diagnostic recording reproduced two clock pauses at 17:09:25 and
17:10:00 Prague time. Map geometry construction occupied the pipe reader for
187–188 ms while the worker continued generating clocks. Updates queued behind
that work and clock ages reached 265 and 281 ms, exceeding the unchanged 250 ms
watchdog. No clock reset was recorded. Clock reading now runs separately from
geometry processing, with one latest cumulative snapshot waiting for processing.
A ground check recorded a maximum clock pipe delay of 16 ms; this does not
guarantee timely delivery under every moving-flight workload.

In the later exploration run, a new frontier route was selected at 17:28:01.
The drone began braking from about 1 m/s. With a reference acceleration bound
of 0.25 m/s², stopping from that speed takes about four simulated seconds and
two metres under constant deceleration. This is a reference-planning calculation,
not a measured physical stopping guarantee; host wall time also depends on
simulation speed. A new geometric route cannot instantaneously redirect velocity.

At 17:28:07, navigation paused because the fixed-height mapper rejected the
altitude, not because the horizontal tracking guard fired. Isaac showed chassis
height around 0.78 m above the configured floor, below the 0.80 m minimum;
PX4 reported local height around 0.91 m. These heights use different origins.
After converting PX4's estimate to world Z and matching source times, the
vertical disagreement was about 0.20 m. Horizontal tracking error at the pause
was about 0.12 m. A later return-home pause repeated the height-gate failure
with horizontal error around 0.064 m. The origin of the vertical disagreement
has not been established; widening the mapper gate would not explain it.

The student chose braking to near-hover while yawing for major route changes,
rather than a wide continuous turn. The new policy waits for both reference and
measured speed at or below 0.15 m/s, then for measured heading error within 30°
before accelerating. Frontier updates cannot bypass an unfinished braking stage.
Shallow route changes continue moving. Prepared yaw rate rises from 45 to 60°/s;
yaw acceleration stays at 90°/s². Acceleration and the 0.35 m tracking guard remain
unchanged. This policy is implemented and tested; verify the dashboard's loaded
and disk runtime versions after Restart before crediting a flight to it.

The separate landing issue is also unresolved: a live check found Isaac's drone
stationary on the floor while PX4 still reported armed in Land and its land
detector reported `landed=False`. Physical floor contact and confirmed controller
landing/disarm must be recorded separately.

Useful follow-up reasoning checks, spread across experiments rather than asked
all at once:

1. What evidence distinguishes actual following lag from comparing samples taken
   at different times?
2. If the reference velocity reaches zero but the drone is still moving, what
   should prevent acceleration onto a newly selected route?
3. How would a controlled straight-flight versus braking/yaw comparison help
   isolate the vertical disagreement?

Next, investigate vertical estimate versus truth through a repeatable maneuver,
then evaluate the new turn policy on the same layout. Record speed, heading,
altitude disagreement and pause reasons around each target switch. Change one
control/model assumption at a time. The independent recorder also now tolerates
a busy Windows status file without abandoning raw recording; a status-file
reporting failure must not be mistaken for loss of the recorded sensor stream.

Evidence stays in ignored local logs:

| Record                                            | What it establishes                                                                        |
| ------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| `logs/current_run_timing_analysis.json`           | Original clock pauses and queued geometry/clock timing                                     |
| `logs/separated_clock_live_check.json`            | Loaded clock separation, healthy ground transport and sampling limits                      |
| `logs/tracking_and_room_transition_analysis.json` | Source-aligned vertical disagreement at two altitude-gate pauses                           |
| `logs/near_hover_turn_checks.txt`                 | 291 passing Python tests, including measured-speed/yaw gating and shallow-turn regressions |

### Repeatable exploration experiment

One larger lab: hide a room behind two turns and press Explore once.
Before running, ask the student to predict which frontier will be selected.
During the run, identify one transition in each layer:

1. More unknown cells become observed after moving around a corner.
2. A frontier goal is selected without an operator destination click.
3. A route remains inside observed space and avoids occupied walls.
4. Position targets advance, measured position follows, PX4 remains Offboard.

After the run, explain one pause/finish using its reason rather than guessing.

Use the same layout twice to compare settings. Record observed area versus
delivered simulated time, viewpoints reached, measured travel, reference versus
actual speed, goal error and any hold/failsafe. Coverage needs a denominator:
without a labelled true traversable-area mask, report newly observed m², not an
invented percentage of the maze. Count FREE and OCCUPIED observations, excluding
the assumed START_AREA. Expected information gain is only a ranking heuristic.

### Experiment records

Generated files under `logs/` stay outside Git.

The dashboard keeps action buttons in fixed positions. Readiness changes are
shown as status and explanations instead of repeatedly moving or hiding controls.
Every action shows immediate sending feedback, then a correlated request status:
queued, accepted/running, completed, paused/blocked or rejected with a reason.
Queueing is not evidence that the controller executed an action. Unacknowledged
queued requests expire after five seconds; a request ID prevents duplicate execution.
The UI checks the small command receipt every 100 ms while waiting, independently
of full-map polling. Destination commands queue without waiting for route search;
the controller's existing planning worker checks reachability before movement.
Hold takes precedence over other commands already in the same controller batch
and cancels those pending movement requests. It also invalidates old planner results.
After a navigator restart, the page can renew its expired command token by
reading the current local page and retrying the explicitly rejected request once
with the same request ID. Requests from a foreign origin remain rejected. A lost
network response is shown as unconfirmed and never automatically resubmitted.

The **history** section combines the observed map, actual capture-time Isaac TF
travel trail, selected destinations (including abandoned ones), and decision log.
Select a recorded frame with the slider or a decision's **Map then** button to
inspect the map used at that time. Recorded-map review leaves live telemetry
visible and disables destination/exploration actions; Hold remains available.
Return to live view before selecting a new destination. Optional filtered LiDAR
endpoints show the returns incorporated into this fixed-height mapper; these are
not the full point cloud or a ROS bag.

The previous run remains selectable after a dashboard-managed reset. Download
either run as JSONL. Files persist across program exits; the dashboard's current/
previous selector is scoped to its running process. The recorder saves a frame
on observed state/goal changes and at five simulated-second intervals when scans
advance. It samples actual travel after at least 0.1 m of displacement. This is
decision evidence, not a recording of every sensor scan.

The controller's `pose_check` records the map pose, selected historical PX4
pose, signed receipt-time difference, position error and heading error. The
existing limits remain 0.20 m, 15 degrees and a 0.25 s nearest-sample gap.
Pose-check stop events preserve the failed comparison even after later checks
pass. These samples are paired by host receipt time; that does not prove their
underlying sensor acquisition times coincide. A transient failure during motion
needs timestamp/estimator investigation before changing the limits.

Maps remain **provisional observations**. Frames include free/occupied cell
revision counts, freshness, and the drone's body-footprint and preferred-margin
checks. A red buffered cell can mean the preferred margin is unmet even when the
body footprint remains clear. Newly changed cells can also invalidate a route
while flying. Neither case alone proves a wall collision or faulty LiDAR.
Compare the selected target, measured trail, filtered endpoints and map at the
blocked event before attributing a cause. This instrumentation does not change
the occupancy algorithm, obstacle margins or completion rule.

`navigation/history.py` writes through a bounded background queue, keeping disk
I/O outside flight callbacks. Recording errors and dropped records are shown in
the dashboard. Memory holds 200 recent events, 1,000 targets, 5,000 trail points
and 80 full frames. Up to 10,000 frame IDs remain retrievable from disk; the JSONL
file retains all successfully recorded records for the run.

For the older sampled trace, `python tools/recover_navigation_history.py` recovers
the latest controller epoch into `logs/recovered_navigation_history.json`.
Recovery on 16 October found 47 selected targets, 303 travel samples and 103
observed state changes. That trace contains no historical grids, so it cannot
reconstruct what LiDAR showed at the earlier edge incident. The recovered JSON
is a separate legacy artifact; new dashboard map replay starts with new recordings.

### Investigation of the wall/edge anomaly, 16 October

The reopened scene had lost its maze collision APIs, old-room deactivation and two
arm locks. These were restored using the existing preparation/arm scripts, backed
up, saved to `C:/Users/finel/Desktop/scene.usd`, and independently verified by a
separate process reading the saved layer. All 11 maze colliders, four zero-degree
arm limits, old-room inactivity and 0.5 m sensor near range are present on disk.
This establishes the repaired baseline; it does not prove which settings were
active during the earlier incident.

`tools/capture_mapping_input.py` records bounded raw ROS clouds and capture-time
TF in NPZ files without publishing any flight commands.
`tools/analyze_mapping_input.py` replays them and compares elevated returns to
audited planar wall bounds. Twelve stationary ground captures had no wall/free
cell flips; 99% of eligible wall returns were within about 8.3 mm of the planar
geometry. These captures do not satisfy the flight mapper's 1 m height gate and
do not establish moving-flight behavior.

`tools/replay_maze_mapping.py` isolates the mapper using ideal noiseless rays
over the 303 recovered travel positions and current planar maze geometry.
The unchanged mapper produced 1,951 wall→free and 1,961 free→wall cell changes.
It assigns whole 20 cm cells directly from individual rays: a ray crossing the
free part of a boundary cell can clear a cell containing an earlier wall hit.
This reproduces map instability without sensor noise or timestamp error, and
provides a plausible mechanism for repeated route invalidation. The replay omits
cylinder geometry, attitude and intermediate travel; it is not a reconstruction
of the unrecorded historical clouds. The old trace records 28 BLOCKED transitions
for the same destination with reason `Route changed; waiting to replan`.

The selected fixed-maze policy now retains confirmed walls until Reset. A first
hit immediately marks an obstacle for planning. Three scan-level hits confirm
the cell; each accepted, distinct-timestamp cloud contributes at most one vote
per cell. A free-only observation resets an unconfirmed hit streak, missing
coverage is neutral, and endpoint hits take precedence over free rays within the
same scan. Confirmed cells remain occupied regardless of subsequent free rays.
Reset creates a new map, clearing both confirmation and tentative evidence.

The same 303-position replay now has **zero confirmed-wall erasures**, with total
wall→free changes reduced from 1,951 to 295. The remaining changes are tentative
cells. There are 275 confirmed cells at the end of this ideal replay. This is a
consistency check, not proof of physical correctness: repeated erroneous hits can
still confirm a false wall. Moving/removed obstacles require a reset in this
static-scene mode. The dashboard distinguishes confirmed walls from tentative
obstacles and reports their counts; replay frames and map exports retain the
confirmation mask and policy. Both kinds of obstacle block navigation.

Raw evidence and results are in `logs/mapping_input_ground.npz`,
`mapping_input_analysis.json`, `ideal_maze_mapping_replay.json` (original),
`ideal_maze_mapping_retained.json` (new policy) and `saved_scene_verification.json`.

| File                                | Evidence                                                                                                                             |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `navigation_state.json`             | Latest controller/goal/pose/speed/exploration snapshot                                                                               |
| `navigation_trace.jsonl`            | New controller's 2 Hz history with simulated integration time, measured XY, speed and goals; observed evidence, never a flight input |
| `decisions_*.jsonl`                 | Per-run maps, filtered hit endpoints, measured TF travel, target selections, controller decisions and command receipts               |
| `recovered_navigation_history.json` | Last legacy trace epoch's sampled targets, travel and states; no historical maps                                                     |
| `ros2_navigation_status.json`       | Diagnostic written on a message change, not a heartbeat                                                                              |
| `isaac_bridge.jsonl`                | Motor/transport/physical bridge diagnostics                                                                                          |
| `navigation_sensor.json`            | Last operator-applied LiDAR near-range setting                                                                                       |
| `mapped_goal_reached.json`          | First live mapped-goal endpoint verification, using the older profile                                                                |
| `exploration_replay.json`           | Offline replay evidence                                                                                                              |
| `maze_preparation.json`             | Operator-run collision preparation and chassis orientation report                                                                    |
| `maze_map_offline_preview.png`      | Synthetic scan preview against saved maze walls; no flight commands                                                                  |

### Verified results and remaining experiments

Live Isaac/PX4 manual goal: planned route 6.52 m; REACHED; about 1-3 cm endpoint
error; approximately 1 m height; Offboard, no failsafe at the endpoint check.
The full measured trajectory/minimum clearance of that run was not recorded.
Current software: 92 navigation tests pass (209 tests in the full suite), including auto arrival/scan/reselection,
stale data, operator cancellation, target lead and speed settings.
Offline ideal 2-D room with an occluding wall: two automatic viewpoints,
30.47 m² newly observed, 10.78 m measured travel, 27.6 simulated seconds.
It verifies geometric logic with a simple response model, not PX4/Isaac flight.
New faster settings and live autonomous exploration are awaiting operator run.
A read-only frontier selection on the earlier room's captured live map found a
reachable viewpoint near (3.05,0.05), with a roughly 1.39 m route and 2.83 m²
estimated information gain. This was previewed, not commanded as a flight.
Proof: exploration_live_choice.json and exploration_dashboard_preview.png.

Run software checks in the Pixi environment:

```cmd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c C:\Users\finel\Documents\ChatGPT\issaac\tools\check_navigation.cmd
```

Run the complete offline replay:

```cmd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml python C:\Users\finel\Documents\ChatGPT\issaac\tools\replay_exploration.py
```

Both are verification tools. They create no flight publishers or mode commands.

## Troubleshooting

### Timestamped timing and tracking evidence

Tracking work is separated from slow navigation work. Planning workers prepare
route cell footprints and speed profiles; the flight callback checks those cells
against the newest map using array operations, plus immediate connector/step and
LiDAR checks. A changed map can still invalidate a route immediately. Frontier
geometry and ledger updates run in a latest-snapshot worker; selection waits for
a result belonging to the current map stamp. Visualization, status notices, disk
reports and runtime fingerprint reads run outside the tracking callback. Reports
retain their capture times. Workers have bounded pending input and cannot publish
late results after shutdown. The follower and its mutable control state still
have one owner; background workers cannot publish flight targets or change mode.

`navigation/process_worker.py` isolates frontier analysis, route selection and
reconsideration, map geometry, ROS visualization, dashboard JSON/route previews,
and history/report serialization in separate processes. On Windows these workers
run below normal priority; numerical libraries use one thread per worker. Small
relay threads transfer immutable snapshots and completed results. The flight
callback never waits for a worker response. Map/frontier/report queues retain
only the newest pending input; history retains its bounded recording queue.
The follower remains owned by the control callback, and new routes still pass
current-map and generation checks. Process isolation removes competition for the
controller's Python execution lock; host CPU scheduling and IPC still have costs.

Clock records are consumed by a dedicated pipe reader. Map clearance and center
geometry are built in a separate process, with just the newest cumulative map
snapshot waiting for processing. This prevents a slow geometry calculation from
holding clock records in the pipe. Scan integration and wall confirmation still
happen upstream in the mapper; discarding an intermediate publication does not
discard its accumulated wall evidence. Snapshot ages retain their original
source times, and the 250 ms clock watchdog and map freshness checks remain in
force. Pipe closure or a processing error revokes readiness; a calculation
finishing afterward cannot clear that failure. This isolates geometry work, but
does not guarantee that upstream ROS clock delivery or host scheduling is timely.

Each callback emits a `control_timing` record, including early returns and rejected
commands. Its named stages record elapsed wall time and CPU time on the control
thread. `callback_gap_s` and `scheduling_delay_s` use QPC, rather than the coarser
Windows monotonic clock. `off_cpu_s` is elapsed time minus this thread's CPU time;
it includes waiting and preemption and does not identify an OS scheduling cause
by itself. Worker `background_started`/`background_work` events include matching
job IDs, process IDs, wall duration and CPU duration for correlation.

Native submission diagnostics separately record command age, Python time before
entering C++, mutex wait, ROS graph-check time and publish time. Rejections identify
stale, future, out-of-order, invalid-frame, competing-publisher, stopped-publisher,
benchmark or publication-error causes. The most recent rejection persists in
`native_publisher.last_rejection` after successful hold commands. A rejected
callback emits `control_rejection` before its stop event; the stop embeds those
timings so its incident retains them even if another diagnostic packet drops.
Incident summaries include per-stage statistics, rejection details and deadline
misses. A late callback following a published command or an over-budget native
publication also emits `deadline_miss` and saves an incident window, even when
the new command itself was accepted. The 100 ms command lease remains enforced, including after graph lookup;
an over-budget command is never made fresh by changing its timestamp.

Build the updated native library with `tools/build_native_control.cmd` inside the
Pixi ROS environment. Changes load at the next navigator restart or successful
ground Reset; an already-running flight continues using its loaded code/library.

The passive `diagnostics` service starts automatically with the navigator and
survives a navigator restart or failed Reset. It never publishes flight commands.
The dashboard's **Controller & clearance → Timestamped diagnostics** panel shows
recording health, aligned error statistics, and download links. A pause saves a
10-second pre-event and 2-second post-event window under `logs/timing_*/`.
Continuous records remain in `events.jsonl`; dropped records and sequence gaps
are reported, so incomplete recordings cannot masquerade as complete evidence.

The recorder also writes `flight_profiles/flight_<bridge-session>_<part>.csv`,
with one instantaneous sample at each integer **Isaac simulation second**.
Columns include a UTC Unix capture timestamp, simulator height, PX4 estimated height, received commanded
height, actual and estimated horizontal speed, expected reference speed,
selected speed ceiling, cruise speed, vertical speeds, phase and slowdown reason.
"Expected" speed describes the navigator reference; it is not an aircraft
response prediction. "Estimated" height is PX4's estimate, not a separate model.
Height comparisons use the configured mapping floor. PX4's world height origin
is calibrated from disarmed, source-aligned observations; starting a recording
airborne leaves that comparison unavailable rather than guessing an offset.
Raw PX4 local height is retained separately. Missing samples stay blank;
interpolation is bounded to 0.2 s, references/setpoints to 0.25 s age, and resets
start separate profiles. Two simulation seconds of buffering allow late streams.

To graph an existing recording, or passively follow the current recorder with
interactive charts that update as simulation-time samples are captured:

```text
python tools/record_flight_profile.py logs/timing_<run>/events.jsonl
python tools/record_flight_profile.py --latest-run --follow
python tools/record_flight_profile.py --plot-csv logs/flight_profiles/flight_<session>_0.csv
```

Outputs default to `logs/flight_profiles/`: CSV, metadata JSON, a whole-flight
PNG and a navigation-detail PNG including the pause and following 12 seconds.
The live follower writes `status.json` with current output paths. Use `--floor-z`
when the recording used a floor different from `config/navigation_settings.json`.
This tool only reads the existing recording; it never commands a flight.
CSV recording and the live viewer use the Python standard library. Offline PNG
exports (including the final export on shutdown) also need `matplotlib`.

The dashboard places **Altitude & speed** in the third column beside the map
and navigation controls, stacking below them on narrower screens. Locally bundled
uPlot 1.6.32 charts provide shared hover values, legend toggles, and drag zoom;
hover labels and time axes show capture date/time in Europe/Prague. Clock pairs
saved with the recording preserve time-of-day through slow simulation and pauses.
hovering or focusing an event in the log marks its source-aligned time on both
charts, with dots on the nearest available samples. Events without their own
frame use a nearby recorded frame when available, labelled as approximate.
Event marker labels show the event's recorded date/time.
**Follow live** or a double-click resets zoom. Select **Navigation & pause** or
**Full flight**, expand the charts, or download the CSV. Captured samples stream
to the browser over server-sent events without waiting for PNG rendering or a
five-second timer. The existing two simulation seconds of source-alignment
buffering still apply. The passive follower serves this viewer
at `http://127.0.0.1:8788/`; it can update while the navigator keeps running.
`tools/lab.ps1 start navigator` and `start all` also start/reuse the `profile`
service. Start only the recorder/viewer with `tools/lab.ps1 start profile`.
An existing dashboard page needs a browser refresh to show the new panel.

- `navigation/diagnostic_wire.py`: bounded, nonblocking localhost observations.
- `tools/record_timing.py`: independent recorder, incident windows and summaries.
- `tools/px4_timing_probe.py`: read-only native PX4 position and received
  `trajectory_setpoint` snapshots, at up to 20 Hz wall time. No firmware rebuild.
- `navigation/diagnostics.py`: offline source-time interpolation and evidence checks.
- `tools/analyze_timing.py`: reproduce a summary from an incident JSON or JSONL log:

```text
python tools/analyze_timing.py logs/timing_<run>/events.jsonl --output logs/timing_summary.json
```

Isaac truth carries both SimulationManager time and the exact HIL_SENSOR time
sent to PX4. The installed SITL source sets its monotonic clock from that sensor
timestamp; native uORB `timestamp_sample` therefore provides the comparison axis.
DDS timestamps are also retained but are explicitly marked XRCE-adjusted and are
not assumed to be simulation timestamps. Capture and arrival times remain separate.
Shared Windows host capture/receipt stamps use `perf_counter()` (QPC), not
`monotonic()`: the installed Python 3.12 GetTickCount64 and Python 3.13 QPC
monotonic clocks have different epochs. Producer-local monotonic stamps and WSL
poll stamps retain their domain labels; only durations within those domains are
compared. Native sample time remains the axis for physical position comparisons.

The analyzer compares Isaac truth and the PX4 estimate at the same native sample
time, then compares truth with the intended source-time route. Received PX4
setpoints use zero-order hold, not interpolation; both physical tracking error
and PX4's internal position error are reported. Native setpoint observations
confirm receipt inside PX4, not motor execution. Snapshot polling can miss
intermediate setpoints and does not establish an exact network latency.
Missing brackets, gaps over 0.2 s in truth or 0.25 s in references, invalid position
estimates, process resets and coordinate resets remain unavailable evidence.
This is a diagnostic approximation with explicit sampling limits, not ground
truth about continuous tracking between samples.

Clock decisions now record the exact consumed interval, clock age, confirmation
state, and backward/jump/stale reason atomically. Stage records include worker
processing duration, pipe age, map geometry computation and navigator callback
duration. These support distinguishing upstream clock delivery gaps, pipe/geometry
delays and control-callback stalls without changing thresholds or automatically
resuming. A recovered clock is displayed separately from the retained pause reason.

| Symptom                                | Responsible layer to inspect                                                                                                 |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| No dots                                | Isaac LiDAR graph, render product, Play, Zenoh router and cloud probe.                                                       |
| Dots but no map                        | cloud frame/stamp, capture-time TF, floor/height and map bounds.                                                             |
| Goal refused                           | observed-space/clearance mask, stable hover, fresh position/map.                                                             |
| No route through a corridor            | width versus twice the preferred margin; inspect unknown boundaries as well as walls. A* does not solve unobserved geometry. |
| Explorer finished with unknown left    | inspect accessibility, clearance, visited points, the limited candidate search and progress/goal limits.                     |
| Map changed but drone does not move    | phase/reason, Offboard status and target lead.                                                                               |
| Automatic Return/Land                  | inspect PX4 failsafe and ROS state before blaming LiDAR.                                                                     |
| Fresh MAVLink motors but no ROS status | XRCE/agent link is a separate connection.                                                                                    |
| Second setpoint publisher              | stop the older hover/step program in native PX4 Hold.                                                                        |
| Changed sensor mounting                | update and restart its static TF; do not guess offsets.                                                                      |

## Further reading and experiment history

Primary-source background:

- [Frontier exploration research](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.616470/full): frontier cells and exploration algorithms.
- [PX4 Offboard documentation](https://docs.px4.io/v1.17/en/flight_modes/offboard): heartbeat and position-control interface.

These explain concepts; local source and live measurements define this install.
The explorer is our small implementation, not a drop-in copy of those packages.

[Bridge setup](docs/BRIDGE_SETUP.txt) is the launch reference;
[mapped navigation](docs/MAPPED_NAVIGATION.txt) covers mapped goals and earlier
results. [EXPERIMENTS.txt](EXPERIMENTS.txt) is the roadmap. The notes under docs/history/ preserve
earlier LiDAR, clearance, flight and movement lessons. Their historical 'pending'
statements are not the current status; use this guide and newer dated evidence.

## Verification and development

Latest full Python check, **19 October 2026: 347 tests passed**. This includes
clock progress during blocked geometry, bounded pending snapshots, failure
latching, busy diagnostic status-file recovery, near-hover turn coordination,
setpoint progress with blocked background workers, prepared-route equivalence,
new-obstacle rejection and isolated ROS node/worker startup and shutdown.
Offline tests and the clock ground check do not establish a successful moving
flight with the newly prepared turn policy or resolve the vertical/landing issues.

Run the full suite in the existing Pixi environment:

```cmd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c C:\Users\finel\Documents\ChatGPT\issaac\tools\check_all.cmd
```

The launcher loads PX4 messages and uses isolated ROS domain 97. Tests exercise
fake flight outputs, local MAVLink sockets, a temporary dashboard and remapped
sensor topics. They do not arm or control the running simulator.

On 2026-10-14, source cleanup passed 119 tests. The subsequent maze adaptation
passed 126 tests. The mission, trail memory and timing changes now pass
**167 tests**, including 70 navigation tests. On 16 October the decision recorder,
legacy recovery, command receipts, session renewal, wall retention and explicit
flight buttons, smooth route yaw and braking-aware faster cruise bring the suite to **209 tests**, including 92 navigation tests.
The operator cockpit places flight telemetry and eight colored system indicators
above the map. Hover or focus an indicator for its readiness details. Flight,
navigation, Hold, Land and recovery controls share the Navigation panel. Loading
indicators identify pending commands, route planning, replay and reset work;
stale telemetry and lost connections cannot retain healthy state indicators.
The event log spans the map and navigation columns beneath them, merging targets,
decisions and command responses, with
local 24-hour timestamps and links to recorded map frames. **Filter event types**
lets you hide categories; frontier boundary updates are hidden by default.
Selections persist in this browser, and **Show all event types** restores them.
Filtering only changes the display; the recorded log retains every event. Map replay and
controller diagnostics are available in expandable sections.

Thirty-one isolated JavaScript interaction checks run with
`node tests/history_ui.test.cjs`; they exercise stable controls, immediate reset
feedback, controller rejection, map replay, cockpit readiness, loading states,
disconnect/recovery and merged event history without sending flight commands.
The new history/UI changes have not been checked in a live flight. The updated
dashboard is serving and the reconnected simulator is playing disarmed; raw
ground sensor captures and saved-scene settings were checked on 16 October.
Button command effects were tested only against isolated fake flight outputs;
no live dashboard flight/reset requests were sent by the agent. The maze-preparation script was
checked on an isolated USD fixture; live maze exploration is still pending.
ROS map/path/pose/trail delivery has also been verified on the live Fast DDS
network. Whole-maze completion and automatic landing remain to be demonstrated.

Python formatting and lint configuration live in `pyproject.toml`. Install
`requirements-dev.txt` in a separate development environment, then run:

```text
black --check .
ruff check .
```

The dashboard HTML, JSON and this README use Prettier 3.9.10 with `.prettierrc.json`:

```text
npx --yes prettier@3.9.10 --check config/navigation.html config/navigation_settings.json bridge_config.json README.md
```

The bridge's pinned MAVLink dependency is in `requirements.txt`. NumPy, ROS,
USD and Isaac APIs come from their existing runtimes; these are not a standalone
pip-installable robotics stack. Private dependencies may be placed in `.vendor/`
for the Isaac bridge; optional offline USD inspection dependencies use `.tools/`.

If `.vendor/` is absent in a new checkout, install the bridge requirements into
that directory using the existing Pixi Python:

```cmd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml python -m pip install --target .vendor -r requirements.txt
```

Generated logs, flight recordings, results, scene assets, caches, local helper
dependencies and shortcut backups stay local and are excluded from Git. Avoid
deleting them when cleaning source: they contain experiment evidence and
operator recovery files. Line endings are specified in `.gitattributes`.
