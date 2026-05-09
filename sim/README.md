# MuJoCo ↔ Cavalier bridge

Drop-in physics simulator for the Cavalier forklift control stack. The
bridge replaces `forklift_driver/driver_node` (which speaks CAN to the
real Curtis MBV15 motor controller). It exposes the **same ROS 2 topics**
in **both directions**, so the rest of Cavalier (orchestrator, automation
mux, safety_node, fork_height controller, Nav2 bridge) is unchanged —
the sim is the hardware.

## Topic interface (matches the real driver_node)

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/safety/command` | `forklift_msgs/ForkliftDirectCommand` | unified throttle + steer + lift/tilt; identical to the message that drives the real CAN bus |
| Pub | `/forklift/drive_feedback` | `forklift_msgs/ForkliftDriveFeedback` | RPM, current, brake, switches, odometer (synthesized from sim state) |
| Pub | `/forklift/steering_angle` | `std_msgs/Float32` | rear steer angle, radians |
| Pub | `/forklift/fork_height` | `std_msgs/Float32` | matches what the pull-wire encoder publishes on hardware |
| Pub | `/joint_states` | `sensor_msgs/JointState` | for `tricycle_odometry` and `robot_state_publisher` |

### Command axis mapping

`ForkliftDirectCommand` → MuJoCo actuators:

| Field | Sim mapping |
|-------|-------------|
| `drive_speed` (-1..1) | both front-wheel motors equally — unified throttle |
| `steering_angle` (-1..1) | rear steer position, scaled to ±1.22 rad |
| `lift_speed` (-1..1) | integrated → fork-lift position target (max 0.40 m/s) |
| `tilt_speed` (-1..1) | integrated → mast-tilt position target (max 0.50 rad/s) |
| `estop_active` | drive forced to 0; brake reported engaged |
| `brake_interlock_released` | when False, drive command is gated to 0 |

`side_shift_speed` and `fork_spread_speed` are accepted but currently
no-op (the model has no side-shift / fork-spread joints — easy to add).

## Running

### One-time build

```bash
cd /home/testbench3/cavalla-sim-forklift/sim/bridge_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

### Launch the sim (with viewer)

```bash
source /opt/ros/jazzy/setup.bash
source /home/testbench3/cavalla-sim-forklift/sim/bridge_ws/install/setup.bash
python3 -m mujoco_bridge.mujoco_driver_node
```

A MuJoCo viewer window opens. The node logs `mujoco_driver ready;
subscribed to /safety/command`.

### Headless mode (CI / no display)

```bash
MUJOCO_HEADLESS=1 python3 -m mujoco_bridge.mujoco_driver_node
```

### Pointing at a different model

```bash
FORKLIFT_XML=/path/to/other_forklift.xml python3 -m mujoco_bridge.mujoco_driver_node
```

## Driving it from the Cavalier stack

Two integration patterns:

### A. Sim is the only "hardware"

Bring up the controls container with `driver_node` disabled and run
this bridge alongside on the host. They share `network_mode: host`, so
DDS discovery is automatic on `ROS_DOMAIN_ID=0` (or whatever the
container is set to).

In `modules/controls/launch.sh`, comment out:

```bash
# _add_window driver_node                 ros2 run forklift_driver driver_node
```

Then on the host:

```bash
source /opt/ros/jazzy/setup.bash
source /home/testbench3/cavalla-sim-forklift/sim/bridge_ws/install/setup.bash
ROS_DOMAIN_ID=0 python3 -m mujoco_bridge.mujoco_driver_node
```

The orchestrator publishes goals → automation sources publish on
`/automation/command_sources/*` → mux → safety_node → `/safety/command`
→ this bridge → MuJoCo physics. Same code path as the real robot.

### B. Smoke-test directly with `ros2 topic pub`

```bash
ros2 topic pub --rate 20 /safety/command forklift_msgs/msg/ForkliftDirectCommand \
  "{drive_speed: 0.5, steering_angle: 0.2, lift_speed: 0.0, brake_interlock_released: true}"
```

## Verification — `test_bridge.py`

The script in `sim/test_bridge.py` walks every command axis and checks
the corresponding feedback. Run it against a live bridge node:

```bash
# terminal 1
python3 -m mujoco_bridge.mujoco_driver_node

# terminal 2
source /opt/ros/jazzy/setup.bash
source /home/testbench3/cavalla-sim-forklift/sim/bridge_ws/install/setup.bash
python3 /home/testbench3/cavalla-sim-forklift/sim/test_bridge.py
```

Expected last line: `Result: 15/15 bridge checks passed`.

## Layout

```
sim/
├── README.md                     ← this file
├── test_bridge.py                ← end-to-end pub/sub test
└── bridge_ws/                    ← colcon workspace
    └── src/
        ├── forklift_msgs/        ← copy of Cavalier msg defs (built standalone here)
        └── mujoco_bridge/
            └── mujoco_bridge/
                └── mujoco_driver_node.py   ← the bridge
```
