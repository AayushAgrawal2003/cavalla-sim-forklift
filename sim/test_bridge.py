"""
End-to-end test: publish ForkliftDirectCommand on /safety/command, verify
the simulated forklift responds on /forklift/* and /joint_states.

Assumes mujoco_driver_node is already running (headless or with viewer).
Walks through every command axis (drive, steer, lift, tilt, estop) and
asserts the expected feedback. Prints PASS/FAIL per check.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from forklift_msgs.msg import ForkliftDirectCommand, ForkliftDriveFeedback


CMD = '/safety/command'
T_STEER = '/forklift/steering_angle'
T_FB = '/forklift/drive_feedback'
T_FORK_H = '/forklift/fork_height'
T_JS = '/joint_states'


class BridgeTester(Node):
    def __init__(self):
        super().__init__('bridge_tester')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_cmd = self.create_publisher(ForkliftDirectCommand, CMD, qos)

        self.fb: ForkliftDriveFeedback | None = None
        self.steer_angle: float | None = None
        self.fork_h: float | None = None
        self.js: JointState | None = None

        self.create_subscription(ForkliftDriveFeedback, T_FB,
                                 lambda m: setattr(self, 'fb', m), 1)
        self.create_subscription(Float32, T_STEER,
                                 lambda m: setattr(self, 'steer_angle', m.data), 1)
        self.create_subscription(Float32, T_FORK_H,
                                 lambda m: setattr(self, 'fork_h', m.data), 1)
        self.create_subscription(JointState, T_JS,
                                 lambda m: setattr(self, 'js', m), 10)

    def send(self, **kwargs) -> None:
        msg = ForkliftDirectCommand()
        # Defaults: zero everything, brake engaged.
        msg.drive_speed = float(kwargs.get('drive_speed', 0.0))
        msg.steering_angle = float(kwargs.get('steering_angle', 0.0))
        msg.lift_speed = float(kwargs.get('lift_speed', 0.0))
        msg.tilt_speed = float(kwargs.get('tilt_speed', 0.0))
        msg.side_shift_speed = float(kwargs.get('side_shift_speed', 0.0))
        msg.fork_spread_speed = float(kwargs.get('fork_spread_speed', 0.0))
        msg.estop_active = bool(kwargs.get('estop_active', False))
        msg.accel_time_s = float(kwargs.get('accel_time_s', 0.0))
        msg.decel_time_s = float(kwargs.get('decel_time_s', 0.0))
        msg.brake_interlock_released = bool(kwargs.get('brake_interlock_released', True))
        self.pub_cmd.publish(msg)

    def hold_for(self, duration_s: float, **cmd) -> None:
        """Publish `cmd` at 20 Hz for `duration_s` real seconds."""
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            self.send(**cmd)
            rclpy.spin_once(self, timeout_sec=0.05)


def report(name: str, passed: bool, detail: str = '') -> bool:
    tag = '[PASS]' if passed else '[FAIL]'
    print(f'  {tag} {name}' + (f'  ({detail})' if detail else ''), flush=True)
    return passed


def main():
    rclpy.init()
    t = BridgeTester()
    print('Waiting for bridge feedback...', flush=True)

    deadline = time.monotonic() + 5.0
    while rclpy.ok() and time.monotonic() < deadline and (t.fb is None or t.steer_angle is None):
        t.send(drive_speed=0.0, brake_interlock_released=False)
        rclpy.spin_once(t, timeout_sec=0.1)

    if t.fb is None:
        print('No feedback from /forklift/drive_feedback — is mujoco_driver running?', flush=True)
        sys.exit(1)

    odo_start = t.fb.odometer_cm
    results = []

    print('\n--- 1. drive forward ---', flush=True)
    t.hold_for(3.0, drive_speed=0.6, brake_interlock_released=True)
    results.append(report('motor_speed_rpm > 0', t.fb.motor_speed_rpm > 5,
                          f'rpm={t.fb.motor_speed_rpm}'))
    results.append(report('forward_switch true', t.fb.forward_switch))
    results.append(report('brake released', not t.fb.electromagnetic_brake))
    results.append(report('odometer advanced',
                          t.fb.odometer_cm - odo_start > 100,
                          f'Δodo={t.fb.odometer_cm - odo_start}cm'))

    print('\n--- 2. drive in reverse ---', flush=True)
    t.hold_for(2.0, drive_speed=-0.5, brake_interlock_released=True)
    results.append(report('motor_speed_rpm < 0', t.fb.motor_speed_rpm < -5,
                          f'rpm={t.fb.motor_speed_rpm}'))
    results.append(report('reverse_switch true', t.fb.reverse_switch))

    print('\n--- 3. steer left then right ---', flush=True)
    t.hold_for(0.8, drive_speed=0.0, steering_angle=0.6, brake_interlock_released=True)
    s_left = t.steer_angle
    t.hold_for(0.8, drive_speed=0.0, steering_angle=-0.6, brake_interlock_released=True)
    s_right = t.steer_angle
    results.append(report('steer left positive rad', s_left > 0.2,
                          f'rad={s_left:.3f}'))
    results.append(report('steer right negative rad', s_right < -0.2,
                          f'rad={s_right:.3f}'))

    # Re-center steer for subsequent fork tests
    t.hold_for(0.4, steering_angle=0.0, brake_interlock_released=True)

    print('\n--- 4. lift forks ---', flush=True)
    fork_before = t.fork_h
    t.hold_for(2.5, lift_speed=1.0, brake_interlock_released=True)
    fork_after_up = t.fork_h
    results.append(report('fork height increased',
                          (fork_after_up - fork_before) > 0.5,
                          f'{fork_before:.3f}→{fork_after_up:.3f}m'))

    print('\n--- 5. lower forks ---', flush=True)
    t.hold_for(2.5, lift_speed=-1.0, brake_interlock_released=True)
    fork_after_down = t.fork_h
    results.append(report('fork height decreased',
                          (fork_after_up - fork_after_down) > 0.5,
                          f'{fork_after_up:.3f}→{fork_after_down:.3f}m'))

    print('\n--- 6. tilt mast (raise forks first so tilt motion is visible) ---', flush=True)
    t.hold_for(1.5, lift_speed=1.0, brake_interlock_released=True)
    js_before_tilt = t.js
    t.hold_for(1.0, tilt_speed=-1.0, brake_interlock_released=True)
    tilt_idx = t.js.name.index('tilt') if t.js and 'tilt' in t.js.name else -1
    tilt_back = t.js.position[tilt_idx] if tilt_idx >= 0 else 0.0
    t.hold_for(1.5, tilt_speed=1.0, brake_interlock_released=True)
    tilt_fwd = t.js.position[tilt_idx] if tilt_idx >= 0 else 0.0
    results.append(report('mast tilted backward (negative)', tilt_back < -0.03,
                          f'rad={tilt_back:.3f}'))
    results.append(report('mast tilted forward (positive)', tilt_fwd > 0.03,
                          f'rad={tilt_fwd:.3f}'))

    print('\n--- 7. e-stop forces brake on while drive is commanded ---', flush=True)
    t.hold_for(1.5, drive_speed=0.8, estop_active=True, brake_interlock_released=True)
    results.append(report('brake engaged under estop', t.fb.electromagnetic_brake))
    results.append(report('estop_active reflected', t.fb.estop_active))

    print('\n--- 8. brake interlock locked: drive command should not move wheels ---', flush=True)
    odo_pre = t.fb.odometer_cm
    t.hold_for(1.5, drive_speed=0.8, brake_interlock_released=False)
    delta = t.fb.odometer_cm - odo_pre
    results.append(report('drive blocked when brake interlock not released',
                          delta < 5, f'Δodo={delta}cm in 1.5s'))

    # Final stop so the sim doesn't keep running away.
    t.send(drive_speed=0.0, brake_interlock_released=False)
    rclpy.spin_once(t, timeout_sec=0.1)

    passed = sum(results)
    total = len(results)
    print(f'\nResult: {passed}/{total} bridge checks passed', flush=True)

    t.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
