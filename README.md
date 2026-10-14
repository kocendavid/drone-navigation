# Isaac / PX4 drone navigation lab

A simulated quadcopter that maps LiDAR observations, plans routes, and sends
position targets to PX4. Isaac Sim supplies the scene, sensors and physical
response. This project is also a teaching lab: the operator configures the
scene and performs takeoff, flight-mode handovers and landing.

## Start here

- [Teacher and maintainer guide](TEACHER_GUIDE.txt): system roles, essential
  files, terminals, frames, maze design, operation and evidence.
- [Experiment roadmap](EXPERIMENTS.txt): progression from hover to exploration.
- [Bridge setup](docs/BRIDGE_SETUP.txt): Isaac physics, PX4 launch and motor/sensor
  connection.
- [Mapped navigation](docs/MAPPED_NAVIGATION.txt): map, planner, goal control and
  dated live results.
- [Earlier experiment notes](docs/history/): preserved troubleshooting history;
  dated pending states are not the current system status.

## Architecture

```text
Isaac LiDAR + capture-time TF (ROS / Zenoh)
  -> ray map -> frontier viewpoint -> A* route -> position reference
  -> PX4 (ROS / Fast DDS) -> MAVLink motors -> Isaac rotor forces
  -> physical movement -> next LiDAR view

Isaac IMU / GPS / magnetometer / barometer
  -> MAVLink simulator bridge -> PX4 estimation and flight control
```

The navigator owns a perception worker and receives its map snapshots through
a local pipe. This connects the existing Zenoh sensor network to the Fast DDS
PX4 network. The navigator publishes targets; it never arms or selects a mode.
Map localization uses Isaac's known world pose. LiDAR-based localization/SLAM
and learned exploration are later experiments.

## Source layout

| Location       | Responsibility                                                    |
| -------------- | ----------------------------------------------------------------- |
| `px4_isaac/`   | Physics callbacks, rotor/sensor models and MAVLink transport      |
| `navigation/`  | Ray map, planning, exploration, tracking, settings and dashboard  |
| `lidar_tools/` | Cloud parsing, clearance measurements and earlier flight gate     |
| `config/`      | Navigation settings, browser dashboard and RViz presets           |
| `tools/`       | Operator launchers, inspection scripts and verification commands  |
| `tests/`       | Isolated flight, sensor, mapping, planning and exploration checks |
| `px4/`         | WSL external-simulator startup                                    |
| `docs/`        | Setup references and historical experiment notes                  |

Root entry points remain stable: `load_bridge.py`, `ros2_navigate.py`,
`ros2_hover.py`, `ros2_step.py`, and `start-isaac-px4.cmd`. The hover/step programs
are earlier lessons; run only one flight-target publisher at a time.

## Operation

This checkout is configured for the existing Windows Isaac Sim / ROS 2 Jazzy
Pixi environment and PX4 SITL in WSL Ubuntu:

- Project: `C:\Users\finel\Documents\ChatGPT\issaac`
- ROS environment: `C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml`
- PX4 messages: `C:\px4-tools\ros_ws\install\setup.bat`

After the existing native 1 m hover works, select native PX4 Hold before
replacing a flight publisher. Start the navigator in a Jazzy/Pixi terminal:

```cmd
C:\Users\finel\Documents\ChatGPT\issaac\tools\ros2_navigate.cmd
```

Wait for READY, select Offboard in the PX4 console, and use
<http://127.0.0.1:8787/> to choose destinations or start exploration. The
publisher must stay running during Offboard. Closing the browser does not stop
an exploration session. **Hold here** cancels it and holds measured position.
See the teacher guide for full launch/reset and scene-editing procedures.

For perception-only preview, append `--observe`; all flight commands are
disabled. Use `--port 8788` if another navigator already owns port 8787.

[Navigation settings](config/navigation_settings.json) are loaded at launch.
The prepared profile requests up to 0.5 m/s, slows below 1 m, and stops/replans
at 0.65 m with a 0.5 m LiDAR near range. The preferred wall margin is 0.8 m.
These are reference limits; measured aircraft speed and braking must be evaluated.

## Verification

Run the complete test suite from the existing Pixi environment:

```cmd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml cmd /d /c C:\Users\finel\Documents\ChatGPT\issaac\tools\check_all.cmd
```

The launcher loads the PX4 messages and uses isolated ROS domain 97. Tests
exercise fake flight outputs, local MAVLink sockets, a temporary dashboard and
remapped ROS sensor topics. They do not arm or control the running simulator.
Smaller check launchers remain available in `tools/`.

Cleanup verification on 2026-10-14: **119 tests passed**, along with Python lint,
Python/HTML formatting, dashboard JavaScript syntax and WSL shell syntax checks.

Run an offline occluded-room exploration replay:

```cmd
pixi run --manifest-path C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml python C:\Users\finel\Documents\ChatGPT\issaac\tools\replay_exploration.py
```

The replay uses a simple 2-D response model and creates no flight publishers.
Its output is distinct from an Isaac/PX4 flight result.

## Development

Python formatting and lint configuration live in `pyproject.toml`. Install
`requirements-dev.txt` in a separate development environment, then run:

```text
black --check .
ruff check .
```

The dashboard HTML and JSON use Prettier 3.9.10 with `.prettierrc.json`:

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

## Verified milestones and remaining work

The first live operator-selected mapped goal reached its endpoint after a
planned 6.52 m detour, with approximately 1–3 cm endpoint error and PX4 still
in Offboard without a failsafe at verification. Its full measured trajectory
was not recorded.

The newer exploration selector and faster profile pass software checks. An
offline replay selected two viewpoints and observed 30.47 m² beyond its
initial view. Live autonomous exploration and the faster profile still require
an operator reload and flight check. Standard ROS map/path publication for RViz
is not yet implemented. The rotor parameters remain preliminary.
