# Isaac / PX4 drone navigation lab

A teacher and maintainer guide to a simulated quadcopter that uses LiDAR to
discover space, choose useful viewpoints and physically fly there through PX4.
Isaac Sim supplies the scene, sensors and physical response. The operator owns
scene editing and explicit takeoff/flight-mode selection through the dashboard
or PX4 console. The explicit explore-and-land
mission can request landing after confirming completion and saving its map.

Updated **2026-10-16**. The current maze configuration covers 64 × 24 m at
0.2 m resolution, with floor Z near −3.98 m. Scene preparation and disarmed
Isaac/PX4 readiness are verified; live autonomous maze exploration remains pending.
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

Among unvisited reachable viewpoints, the selector favors a view that could
reveal unknown area currently hidden behind a wall near such an ending. If no
useful corner view qualifies, it keeps the farthest useful frontier preference.
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
Continuous reconsideration still has no switching-radius rule.

This is a first geometric heuristic: short/thick walls, noisy geometry or
unconfirmed endings can be missed, and two-room behavior needs a live run.
It does not classify rooms or guarantee complete coverage. The independent
enclosure/completion checks remain responsible for declaring exploration done.

## optimizedRoutePlanner: return home

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
If no safe mapped route exists, it holds as BLOCKED; it never invents a passage
or reports a successful return. Home arrival requires the measured position to
remain within 0.15 m at a stable hover for one simulated second. Use **Land here**
separately if you want to land.

Because the return planner uses the accumulated map rather than replaying the
outbound travel history, it can skip exploration detours and use known
shortcuts between connected rooms. A* minimizes grid distance; shortcutting
reduces unnecessary bends. This is not a guarantee of the mathematically
shortest continuous route or fastest flight: clearance, turns, acceleration and
new obstacles still affect the result. Unknown space remains excluded even if
it would make a shorter path. Home capture, its selected goal, route changes,
interruptions and arrival are included in decision history/map frames.

## Control the lab through code

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
`logs/experiment_baseline.json` and checked on subsequent resets. Changing the
scene or starting pose requires establishing a new baseline. Maze edits and
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
The mapper accepts valid return rays up to 12 m, so the 10 m corridor width does
not by itself require a close wall inspection. A changed viewpoint is useful
when it resolves occlusion or missing observations.

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
runs planning in a background thread, sends the 10 Hz Offboard heartbeat,
holds altitude, turns smoothly toward route segments, and advances the exploration session.
Starting the program never arms or takes off. Explicit dashboard flight buttons
can request native takeoff, Offboard and landing. The explore-and-land mission requests native
PX4 Land after confirmed completion and successful map persistence.

During navigation the nose follows the current planned route segment, with a
45°/s yaw-rate limit and 90°/s² reference acceleration. It turns while travelling;
a sharp corner can temporarily leave the nose behind the direction of movement.
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
web server, and is the one program publishing PX4 targets while in Offboard.
Keep it running. Its map-worker child is automatic and needs no extra window.

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

The heartbeat uses steady computer time at 10 Hz. Route progress uses delivered
simulated IMU integration periods. Slow Isaac physics therefore makes a flight
take longer in real time rather than compressing its target trajectory.
Commands use timestamp=0 because this installed PX4 decoder assigns receipt
time. Sensor timestamps are XRCE-adjusted and must not be copied into commands.
That receipt policy is specific to the installed decoder, not a universal API rule.

Frame validation compares map pose with the closest trusted PX4 position sample
at cloud receipt, within 0.25 s of host time. Disagreement limits remain 0.2 m
and 15 degrees. This accommodates mapping delay; approximate receipt-time matching
does not establish exact source-time synchronization or hide localization errors.

## Current settings

| Parameter                                                 | Prepared value                                      |
| --------------------------------------------------------- | --------------------------------------------------- |
| Cruise with adequate clear straight route                 | Up to 1.00 m/s                                      |
| Full-cruise forward obstacle clearance in braking model   | About 3.15 m                                        |
| Approach between 1.0 and 0.65 m                           | Nominal taper toward 0.08 m/s; braking may lower it |
| Clearance at/below 0.65 m, or no usable observed corridor | Hold/replan                                         |
| Reference acceleration / bend speed cap                   | 0.25 m/s² / 0.15 m/s                                |
| Braking reaction allowance                                | 0.50 simulated seconds                              |
| Target lead / tracking pause guard                        | 0.28 m / 0.35 m                                     |
| Preferred obstacle margin / assumed vehicle radius        | 0.80 m / 0.30 m                                     |
| Sensor near range / starting-area prior                   | 0.50 m / 0.60 m                                     |
| Map dimensions / resolution                               | 320 × 120 cells / 0.20 m                            |
| Map origin                                                | (−34, −27) m                                        |
| Map extent                                                | 64 × 24 m                                           |
| Floor Z                                                   | −3.9803731091662193 m                               |
| Maximum reached exploration viewpoints                    | 20                                                  |

Distances are from the chassis/reference centre, not from rotor tips. The stop
threshold is a decision trigger, not a measured stopping-distance guarantee.
These speeds are reference limits in simulated seconds. The controller may
advance more slowly when actual tracking lags. The dashboard separately shows
reference and measured horizontal speed. A polyline bend/end can also slow it.
The prior and body radius are assumptions; the start area must actually be clear.

The selected cruise cap was doubled from 0.5 to 1.0 m/s after the reset on
16 October. Green cells indicate observed traversable geometry, not permission
to maintain maximum speed. The speed cap reserves reaction travel and stopping
distance: `v × reaction_time + v² / (2 × deceleration) <= clearance − stop_distance`.
With this profile full cruise needs about `0.65 + 1 × 0.5 + 1²/(2 × 0.25) = 3.15 m`
clear ahead. The reference brakes before entering the bend zone and before goals.
An endpoint-free forward strip requires fresh-map certification of the whole
remaining route; missing sensor returns alone never permit motion. Target-lead,
tracking-error and close-range stop limits are unchanged. This kinematic bound
does not establish PX4's measured stopping distance or guarantee twice the actual
flight speed. Offline response-lag and bend tests exercise the faster reference;
the updated ideal occluded-room replay passes with three viewpoints and 30.48 m²
newly observed. Full-speed live tracking and smooth yaw await the operator run.

## Building and preparing a maze

The original 8 m room used a 12 x 12 m map centred near (0,0). The operator's
new maze is roughly 58 x 18 m; its configured map now spans world x=-34..30,
y=-27..-3, with 320 x 120 cells at 0.2 m resolution. Map resolution, dimensions,
origin and floor height are launch-time settings. See docs/MAZE_SETUP.txt for
collision preparation and the restart procedure after rebuilding the scene.

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
Twelve additional isolated JavaScript interaction checks run
with `node tests/history_ui.test.cjs`; they exercise stable controls, immediate
reset feedback, controller rejection and map replay without a browser or flight
commands. Python lint/format and dashboard JavaScript checks also pass.
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
