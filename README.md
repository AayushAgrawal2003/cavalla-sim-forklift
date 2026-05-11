# Cavalla Sim — MuJoCo forklift

MuJoCo physics sim of a 3-wheel counterbalanced forklift (1MBV15R30), wired
to drop into the Cavalier ROS 2 control stack as a sim-hardware target.

![demo](forklift_demo.mp4)

## Operating modes

Three increasingly-integrated ways to drive the sim:

| Mode | What runs | When to use |
|------|-----------|-------------|
| **Scripted demo** | `run_sim.py` writes directly to `data.ctrl` | Sanity-check the model; reproduces a 12-phase motion script and asserts 16 invariants (CoM, wheel contact, fork height, etc.) |
| **Manual ROS bridge** | `mujoco_driver_node` + `ros2 topic pub` (or `sim/test_bridge.py`) | Smoke-test individual commands; develop new automation modules against the sim |
| **Full Cavalier stack** | bridge + orchestrator + automation mux + `safety_node` + a controller (e.g. `line_follower`) | Closed-loop integration: the sim is the *only* "hardware" the real Cavalier code sees |

## What's here

| Path | What it is |
|------|------------|
| `forklift.xml` | MuJoCo model — 2 front drive wheels coupled by a tendon (single throttle), 1 rear steer, mast tilt, fork lift, yellow guide line on the floor |
| `run_sim.py` | 12-phase scripted demo + 16 validation checks; opens the interactive viewer |
| `sim/record_demo.py` | Offscreen renderer → mp4 (used for `forklift_demo.mp4`) |
| `sim/bridge_ws/` | colcon workspace: `forklift_msgs`, `mujoco_bridge`, plus pinned copies of Cavalier's `drop_off`, `drop_off_msgs`, `forklift_driver` |
| `sim/test_bridge.py` | End-to-end pub/sub test (15 checks, exercises every command axis) |
| `sim/run_line_follow.sh` | Closed-loop line-following: bridge + safety + mux + adapter + Cavalier line_follower (unchanged) + activator |
| `sim/record_line_follow.sh` | Same, but also encodes an mp4 (`forklift_line_follow.mp4`) |
| `cavalier_system/` | Submodule → [Cavalla-io/cavalier_system](https://github.com/Cavalla-io/cavalier_system) (`dev` branch). Pinned upstream Cavalier monorepo; the bridge talks to its `forklift_msgs` / `drop_off_msgs` interfaces |

## Cloning

The repo pulls in Cavalier as a submodule (which itself has nested submodules
under `modules/`). Clone recursively:

```bash
git clone --recurse-submodules https://github.com/AayushAgrawal2003/cavalla-sim-forklift.git
# or, if you already cloned without submodules:
git submodule update --init --recursive
```

## Quick start

### Standalone scripted demo

```bash
python3 run_sim.py
```

Runs the 12 phases (drive · steer · lift · tilt · lower · reverse), prints
`16/16 checks passed`, opens the MuJoCo viewer.

### ROS 2 bridge — Cavalier-driven sim

The bridge is a drop-in replacement for `forklift_driver/driver_node`. Same
topics in both directions, so the rest of Cavalier (orchestrator, automation
mux, safety_node, fork PID, Nav2) drives it unchanged.

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/safety/command` | `forklift_msgs/ForkliftDirectCommand` |
| Pub | `/forklift/drive_feedback` | `forklift_msgs/ForkliftDriveFeedback` |
| Pub | `/forklift/steering_angle` | `std_msgs/Float32` |
| Pub | `/forklift/fork_height` | `std_msgs/Float32` |
| Pub | `/joint_states` | `sensor_msgs/JointState` |

**Build (once):**

```bash
cd sim/bridge_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

**Run the sim:**

```bash
source /opt/ros/jazzy/setup.bash
source sim/bridge_ws/install/setup.bash
python3 -m mujoco_bridge.mujoco_driver_node
```

**Drive it (other terminal):**

```bash
source /opt/ros/jazzy/setup.bash
source sim/bridge_ws/install/setup.bash

# self-test — 15 checks across drive, steer, lift, tilt, estop, brake interlock
python3 sim/test_bridge.py

# or a manual command
ros2 topic pub --rate 20 /safety/command forklift_msgs/msg/ForkliftDirectCommand \
  "{drive_speed: 0.5, brake_interlock_released: true}"
```

## Control architecture

`mujoco_driver_node` is a **drop-in replacement** for
`forklift_driver/driver_node` — the ROS node that speaks CAN to the Curtis
MBV15 controller on real hardware. It subscribes to the same
`/safety/command` topic and publishes the same feedback topics, so every
layer above the driver runs unchanged.

```
       ┌─────────────────────────────┐
       │ orchestrator / mission state│
       └──────────────┬──────────────┘
                      │ AutomationGoal
       ┌──────────────▼──────────────────────────────┐
       │ automation modules                          │
       │   line_follower · drop_off · pallet_approach│
       └──────────────┬──────────────────────────────┘
                      │ AutomationCommand on
                      │   /automation/command_sources/<name>
       ┌──────────────▼──────────────┐
       │ automation_command_mux      │   merges sources by priority
       └──────────────┬──────────────┘
                      │ /automation/command
       ┌──────────────▼──────────────┐
       │ safety_node                 │   brake interlock,
       │                             │   teleop priority, e-stop
       └──────────────┬──────────────┘
                      │ /safety/command  (ForkliftDirectCommand)
       ┌──────────────▼──────────────┐
       │ mujoco_driver_node          │   ← bridge (this repo)
       │   ──────────────────────    │   replaces forklift_driver/driver_node
       │   data.ctrl ↔ MuJoCo step   │
       └──────────────┬──────────────┘
                      │ ctrl[drive, steer, mast_tilt, fork_lift]
       ┌──────────────▼──────────────┐
       │ MuJoCo physics              │
       └──────────────┬──────────────┘
                      │ chassis pose, joint states
       ┌──────────────▼──────────────────────────────────────────┐
       │ mujoco_driver_node publishes:                           │
       │   /odom                  /forklift/drive_feedback       │
       │   /joint_states          /forklift/steering_angle       │
       │   /drop_off/line_state   /forklift/fork_height          │
       └─────────────────────────────────────────────────────────┘
```

**Command axes** (`ForkliftDirectCommand` → MuJoCo actuators):

| Field | Range | Sim mapping |
|-------|-------|-------------|
| `drive_speed` | -1..1 | Both front-wheel motors equally (unified throttle) |
| `steering_angle` | -1..1 | Rear steer position, scaled to ±1.22 rad |
| `lift_speed` | -1..1 | Integrated → fork-lift position (max 0.40 m/s) |
| `tilt_speed` | -1..1 | Integrated → mast-tilt position (max 0.50 rad/s) |
| `estop_active` | bool | Drive forced to 0; brake reported engaged |
| `brake_interlock_released` | bool | When `false`, drive command is gated to 0 |

**Perception shortcut.** In sim, `/drop_off/line_state` is computed from
ground-truth chassis pose (knowing where the yellow floor line is and where
the front-low camera sits on the chassis) instead of running OpenCV on a
rendered frame. The controller doesn't know the difference — same message,
same units. This keeps the loop fast and the test deterministic; everything
*above* the perception layer (PD controller, automation mux, safety, driver)
is unmodified Cavalier code.

The standalone scripted mode in `run_sim.py` bypasses all of the above and
writes directly to `data.ctrl` on a 100 Hz loop — useful for model validation
without the ROS stack running.

## Closed-loop line-following demo

The Cavalier `line_follower_controller` (from `modules/drop-off`) runs **unmodified**
against the sim. The bridge skips perception entirely — instead of rendering an
image and running OpenCV detection, it computes the `drop_off_msgs/LineState`
the controller expects directly from MuJoCo ground truth (where the floor line
is, where the simulated front-low camera is on the chassis).

Data flow (closed loop):

```
MuJoCo physics
  ↓ chassis pose
bridge → /odom + /drop_off/line_state
  ↓
line_follower_controller   (UNCHANGED Cavalier code; PD on offset+yaw)
  ↓ AutomationCommand on /automation/command_sources/line_follower
automation_cmd_adapter     (unwraps .cmd → ForkliftDirectCommand)
  ↓ /automation/command_sources/line_follower_cmd
automation_command_mux     (UNCHANGED; merges automation sources)
  ↓ /automation/command
safety_node                (UNCHANGED; brake interlock, teleop priority)
  ↓ /safety/command
bridge → MuJoCo ctrl       (drive, steer, lift, tilt actuators)
```

Run it:

```bash
./sim/run_line_follow.sh --init-y 0.5 --duration 30
```

The forklift starts 0.5 m left of a yellow line at y=0, drives in reverse
along the line (the controller's default direction for drop-off approach),
and converges to y≈0. Bench-tuned gains in
`sim/bridge_ws/install/drop_off/share/drop_off/config/line_follower.yaml`
(`k_lat=0.4`, `k_yaw=3.5`, `low_drive_norm=0.08`).

## Wiring into the real Cavalier stack

In `cavalier_system/modules/controls/launch.sh`, comment out the line that
starts the CAN driver:

```bash
# _add_window driver_node ros2 run forklift_driver driver_node
```

Then run the bridge on the host with the same `ROS_DOMAIN_ID` as the controls
container. DDS discovery is automatic (`network_mode: host`).

See [`sim/README.md`](sim/README.md) for full integration notes.

## Model parameters

| | |
|-|-|
| Wheels | 2 front drive (R=152.5 mm, track=918 mm) + 1 rear steer (R=153 mm), wheelbase 1055 mm |
| Mass | 3200 kg chassis + 1100 kg counterweight + 400 kg battery |
| Mast | tilt range +4° / -6°, lift range 0–1.2 m |
| Forks | L-shaped, 1150 mm long, 100 mm wide, 40 mm thick, 300 mm spacing |

Drive both front wheels are mechanically coupled via a fixed tendon, so the
viewer exposes a single `drive` slider — same as the real forklift's unified
throttle. The other actuators are `steer`, `mast_tilt`, `fork_lift`.

## CAD source

The original STEP file from ZW3D (AP214, ~48 MB) is hosted on Google Drive
since it exceeds reasonable size for git. Download with:

```bash
curl -L "https://drive.google.com/uc?export=download&id=1S6o4qJPj40MmrCi4RoucNmgwd1kFU2cF" \
  -o forklift.stp
```

`*.stp` is gitignored so the file won't be tracked if dropped in the repo root.
