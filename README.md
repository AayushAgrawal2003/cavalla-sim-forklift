# Cavalla Sim — MuJoCo forklift

MuJoCo physics sim of a 3-wheel counterbalanced forklift (1MBV15R30), wired
to drop into the Cavalier ROS 2 control stack as a sim-hardware target.

![demo](forklift_demo.mp4)

## What's here

| Path | What it is |
|------|------------|
| `forklift.xml` | MuJoCo model — 2 front drive wheels coupled by a tendon (single throttle), 1 rear steer, mast tilt, fork lift |
| `run_sim.py` | 12-phase scripted demo + 16 validation checks; opens the interactive viewer |
| `sim/record_demo.py` | Offscreen renderer → mp4 (used for `forklift_demo.mp4`) |
| `sim/bridge_ws/` | colcon workspace with `forklift_msgs` + `mujoco_bridge` |
| `sim/test_bridge.py` | End-to-end pub/sub test (15 checks, exercises every command axis) |

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
