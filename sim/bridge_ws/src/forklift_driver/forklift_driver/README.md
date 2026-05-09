# forklift_driver

ROS 2 package that bridges between the rest of the control stack and the forklift's MBV15/Curtis motor controller over a CANopen bus.

This package contains two nodes:

- `driver_node` — the CAN translator (documented below)
- `safety_node` — the safety gate / command multiplexer (documented below; see `AGENTS.md` for scope)

---

## driver_node

### Purpose

`driver_node` is a **pure translator** between ROS and the forklift hardware. It has two jobs:

1. Take `ForkliftDirectCommand` messages from the safety node and turn them into CAN messages for the MBV15/Curtis controller.
2. Read CAN messages back from the controller and publish hardware feedback as ROS messages.

It does **not** make decisions on its own. It never shapes, scales, limits, or second-guesses a command — it just forwards what the safety node tells it to do. Anything that looks like a control decision (safety trips, speed caps, presets, PID loops, etc.) belongs in a different node. See `AGENTS.md` for the full scope rules.

### Subscriptions

| Topic              | Type                                  | Description                                                                                  |
|--------------------|---------------------------------------|----------------------------------------------------------------------------------------------|
| `/safety/command`  | `forklift_msgs/ForkliftDirectCommand` | The **only** command input to the driver. All commands must come through the safety node.   |

### Publications

| Topic                        | Type                                  | Rate  | Description                                                                 |
|------------------------------|---------------------------------------|-------|-----------------------------------------------------------------------------|
| `/forklift/steering_angle`   | `std_msgs/Float32`                    | 20 Hz | Measured steering angle in radians, read from the MBV15 (TPDO3).           |
| `/forklift/drive_feedback`   | `forklift_msgs/ForkliftDriveFeedback` | 20 Hz | Combined drive/odometry/switch feedback decoded from TPDO1 and TPDO2.      |

All topics published by this node live under `/forklift/*`, which is reserved exclusively for data read from the physical hardware over CAN.

#### `/forklift/drive_feedback` contents

Decoded from the MBV15 TPDO1 (`0x183`) and TPDO2 (`0x283`) frames:

**From TPDO1 (drive):**
- `motor_speed_rpm` — signed, `-4000..4000`; positive is forward
- `motor_current_amps` — motor current in amps (0.1 A resolution)
- `drive_fault_code` — Curtis F2A fault code; `0` means no fault
- `battery_percent` — battery level `0..100`

**From TPDO2 (odometer + I/O):**
- `odometer_cm` — cumulative distance since controller power-on (monotonic)
- `estop_active` — `true` when the e-stop is pressed (protocol bit is active-low)
- `interlock_on` — interlock engaged
- `forward_switch` / `reverse_switch` — commanded drive direction from the controller's own switch inputs
- `mode_auto` — `true` = autonomous mode, `false` = manual/teleop
- `electromagnetic_brake` — `true` = brake is clamped/engaged, `false` = released

### Command → CAN mapping

When a `ForkliftDirectCommand` arrives on `/safety/command`, the driver forwards the following fields to `MBV15Interface.send_commands`:

- `drive_speed` — normalized drive effort, `-1.0` to `1.0`
- `steering_angle` — normalized steering target (currently passed as-is to the CAN interface)
- `lift_speed`, `tilt_speed`, `side_shift_speed` — normalized hydraulic commands
- `accel_time_s`, `decel_time_s` — drive curve parameters. If either is `0.0`, the driver substitutes its default:
  - `DEFAULT_ACCEL_S = 1.0`
  - `DEFAULT_DECEL_S = 0.2`
- `brake_interlock_released` — when `True`, releases the electromagnetic brake on the hardware

The `fork_spread_speed` and `estop_active` fields of `ForkliftDirectCommand` are currently not forwarded by `driver_node` (e-stop is enforced upstream by the safety node).

### CAN interface

The driver talks to the MBV15/Curtis controller through `can_interface.mbv15_iface.MBV15Interface`:

- Channel: `can0`
- Bitrate: `250000`
- CANopen node ID: `0x03`
- Outgoing PDOs: `RPDO1` (flags + drive + steering), `RPDO2` (pump + aux valves)
- Incoming PDOs: `TPDO1` (drive state), `TPDO2` (I/O + e-stop), `TPDO3` (steering angle)

If `can0` is not available at startup, the interface falls back to **mock mode**: the node continues to run and log warnings, but CAN sends and reads become no-ops. This is useful for bringup and testing off-vehicle.

### Feedback loop

A 20 Hz timer (`_feedback_timer`) drains any pending frames from the CAN bus via `MBV15Interface.read_feedback()` and publishes the latest steering angle. CAN read failures are logged at most once every 5 seconds to avoid log spam.

### Shutdown behavior

On `KeyboardInterrupt` (or any clean shutdown path), the driver calls `MBV15Interface.stop_all()`, which publishes zeroed RPDO1/RPDO2 frames so the forklift does not latch onto the last command after the node exits.

---

## safety_node

### Purpose

`safety_node` has two jobs and only two jobs:

1. Decide whether the system is in a safe state to send commands to the hardware.
2. Multiplex the teleop and automation command streams into the single `/safety/command` topic that the driver consumes.

It does **not** shape, scale, or second-guess commands. It picks a source, optionally zeroes the output if nothing is safe to forward, and publishes. See `AGENTS.md` for the full scope rules.

### Subscriptions

| Topic                             | Type                                  | Description                                                                                       |
|-----------------------------------|---------------------------------------|---------------------------------------------------------------------------------------------------|
| `/teleop/command`                 | `forklift_msgs/ForkliftDirectCommand` | Operator command stream.                                                                          |
| `/teleop/safety_status`           | `std_msgs/UInt8`                      | Adamo Web heartbeat status: `0` Safe, `1` Unfocused, `2` High Latency, `3` Disconnected.          |
| `/automation/command`             | `forklift_msgs/ForkliftDirectCommand` | Muxed auto command (`automation_command_mux` only; producers use `/automation/command_sources/…`). |
| `/automation/auto_lift_effort`    | `std_msgs/Float32`                    | PID lift effort from `fork_height_controller`. Merged onto `lift_speed` when auto is selected.   |
| `/automation/target_fork_height`  | `std_msgs/Float32`                    | Fork height setpoint in **mm** (from `profiled_fork_height` / PID). Used to detect new auto fork targets (resumes auto fork PID after teleop intervention). |

### Publications

| Topic                    | Type                                  | Rate  | Description                                                                                     |
|--------------------------|---------------------------------------|-------|-------------------------------------------------------------------------------------------------|
| `/safety/command`        | `forklift_msgs/ForkliftDirectCommand` | 10 Hz | The gated, multiplexed command. The driver's only command input.                                |
| `/safety/teleop_state`   | `std_msgs/Bool`                       | 10 Hz | `True` when teleop is healthy and the brake interlock lockout is not active. Used by UIs.       |
| `/safety/command_source` | `std_msgs/UInt8`                      | 10 Hz | Mux branch for UIs / time series. `data`: `0` stop (no fresh source), `1` teleop, `2` auto, `3` stop (teleop unsafe, no auto). |

### Source selection (mux)

At 10 Hz, the node picks exactly one source for `/safety/command`:

1. **Teleop** — if teleop has been *active* within the last `TELEOP_PRIORITY_TIMEOUT = 3.0 s` **and** teleop is *safe*.
2. **Auto** — otherwise, if `/automation/command` is fresh (within `AUTO_CMD_TIMEOUT_SEC = 0.5 s`).
3. **Stop** — otherwise. Publishes a zeroed `ForkliftDirectCommand` with `brake_interlock_released = false`.

"Active" teleop means any of `drive_speed`, `steering_angle`, `lift_speed`, `tilt_speed`, `side_shift_speed`, or `fork_spread_speed` is above `ACTIVITY_THRESHOLD = 0.01` in the latest `/teleop/command`.

"Safe" teleop requires all of:

- `/teleop/safety_status` is `0` (Safe) or `2` (High Latency) — codes `1` and `3` are unsafe.
- `/teleop/safety_status` was received within `heartbeat_timeout_sec = 0.75 s`.
- `/teleop/command` was received within `command_timeout_sec = 1.0 s`.

Key consequence: **when teleop is unsafe (browser closed, heartbeat lost, status code 1/3), the mux falls through to automation** as long as `/automation/command` is fresh. Automation is no longer gated on teleop health.

### Automation command path

When the auto branch is selected, the latest `/automation/command` is passed through verbatim. All fields are autonomy's to set: `drive_speed`, `steering_angle`, `lift_speed`, `tilt_speed`, `side_shift_speed`, `fork_spread_speed`, `accel_time_s`, `decel_time_s`, `estop_active`, and `brake_interlock_released`.

Lift merge rule: if `/automation/auto_lift_effort` is fresh (within `AUTO_EFFORT_TIMEOUT_SEC = 0.5 s`), auto fork is not canceled, and the auto command's `lift_speed` is ~zero, the PID effort is substituted onto `lift_speed`. This lets `fork_height_controller` drive the lift axis without requiring the autonomy stack to also publish `/automation/command`.

NaN values on `/automation/auto_lift_effort` (published by `fork_height_controller` when the PID is inactive) are treated as zero.

### Brake interlock

Each source owns its own `brake_interlock_released`:

- **Teleop branch** — carries `/teleop/command.brake_interlock_released` through, with one exception: if the teleop heartbeat has been absent for more than `BRAKE_LOCKOUT_TIMEOUT_SEC = 2.0 s`, the node locks teleop's cached interlock to `false` until the operator re-arms it with a falling-then-rising edge on the brake interlock button. This prevents a stale "released" state from leaking through after a network drop.
- **Auto branch** — uses `/automation/command.brake_interlock_released` directly. The teleop lockout does **not** apply, so autonomy can release the brake with no teleop heartbeat present.

### Teleop fork intervention

If the operator moves any fork axis (`lift_speed`, `tilt_speed`, `side_shift_speed`, `fork_spread_speed`) above `ACTIVITY_THRESHOLD`, the node sets `auto_fork_canceled = true`, which suppresses the `/automation/auto_lift_effort` merge described above. The flag clears automatically when a new `/automation/target_fork_height` message arrives.

This only affects the PID lift-effort merge. It does **not** cancel the full `/automation/command` channel — autonomy still has full authority on its own topic.

### Parameters / tunables

All timings currently live as instance attributes in `ForkliftSafetyNode.__init__`:

| Name                          | Default | Meaning                                                           |
|-------------------------------|---------|-------------------------------------------------------------------|
| `heartbeat_timeout_sec`       | `0.75`  | Max age of `/teleop/safety_status` before teleop is unsafe.      |
| `command_timeout_sec`         | `1.0`   | Max age of `/teleop/command` before teleop is unsafe.            |
| `AUTO_CMD_TIMEOUT_SEC`        | `0.5`   | Freshness window for `/automation/command`.                      |
| `AUTO_EFFORT_TIMEOUT_SEC`     | `0.5`   | Freshness window for `/automation/auto_lift_effort`.             |
| `TELEOP_PRIORITY_TIMEOUT`     | `3.0`   | How long teleop retains priority after its last active command.  |
| `ACTIVITY_THRESHOLD`          | `0.01`  | Minimum axis magnitude to count as teleop "activity".            |
| `BRAKE_LOCKOUT_TIMEOUT_SEC`   | `2.0`   | Heartbeat gap after which the teleop brake interlock is locked.  |

### How to integrate

**Autonomy stack.** Run `automation_command_mux` as the sole publisher to `/automation/command`. Behaviors publish to `/automation/command_sources/…` (see mux defaults). Keep the mux output ≥2 Hz in practice so the 0.5 s auto freshness window is met.

**Teleop stack.** No changes. Continue publishing `/teleop/command` and `/teleop/safety_status` as before. Your commands still win whenever you are active and safe.

**Driver node.** No changes. It still consumes only `/safety/command`.

---

## Running

Build and source the workspace, then run:

```bash
ros2 run forklift_driver driver_node
```

The safety node must be running and publishing on `/safety/command` for the driver to receive any commands:

```bash
ros2 run forklift_driver safety_node
```

### CAN setup

Before starting the driver on real hardware, bring up the CAN interface:

```bash
sudo ip link set can0 up type can bitrate 250000
```

If `can0` is not up, the driver will log a warning and run in mock mode.

---

## Topic namespace conventions

This package follows the repository-wide namespace rules:

- `/forklift/*` — hardware feedback read over CAN (owned by `driver_node`)
- `/safety/*` — outputs of the safety node (consumed by `driver_node`)
- `/teleop/*`, `/automation/*` — not touched by the driver; they must go through the safety node first

Do not add topics under `/forklift/*` that aren't direct hardware readings, and do not add subscriptions to `driver_node` outside of `/safety/*`.

---

## Dependencies

- ROS 2 (`rclpy`, `std_msgs`)
- `forklift_msgs` (in-tree)
- `python-can >= 4.4.0`
- `pyyaml >= 6.0.2`
- A SocketCAN-capable interface (`can0`) for hardware operation
