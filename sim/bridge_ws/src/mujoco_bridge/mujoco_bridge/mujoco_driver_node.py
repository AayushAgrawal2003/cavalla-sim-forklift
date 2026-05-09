"""
MuJoCo bridge node — drop-in replacement for forklift_driver/driver_node.

Subscribes to the same `/safety/command` topic the real CAN driver listens to,
drives the MuJoCo simulation, and publishes the same feedback topics
(`/forklift/drive_feedback`, `/forklift/steering_angle`, `/forklift/fork_height`,
plus `/joint_states` for sim2's tricycle_odometry node).

Mapping of ForkliftDirectCommand → MuJoCo actuators:
  drive_speed (-1..1)     → single 'drive' motor on the front-axle tendon
                            (both front wheels mechanically coupled)
  steering_angle (-1..1)  → rear steer position, ±1.22 rad
  lift_speed (-1..1)      → integrated into fork-lift position target (max 0.4 m/s)
  tilt_speed (-1..1)      → integrated into mast-tilt position target (max 0.5 rad/s)
  estop_active            → zero drive + slam brake
  brake_interlock_released→ when False, drive is forced to 0

The MuJoCo viewer runs in the main thread; ROS executor + physics run in
background threads to keep the UI responsive.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from forklift_msgs.msg import ForkliftDirectCommand, ForkliftDriveFeedback

try:
    from drop_off_msgs.msg import LineState
    HAVE_LINE_STATE = True
except ImportError:
    HAVE_LINE_STATE = False


# Velocity scaling for axes commanded as normalized "speed" but driven as
# position in MuJoCo (we integrate the speed cmd into a position target).
LIFT_MAX_MPS = 0.40       # m/s at |lift_speed|=1
TILT_MAX_RPS = 0.50       # rad/s at |tilt_speed|=1
STEER_MAX_RAD = 1.22      # mapping for steering_angle=±1.0

DRIVE_RPM_MAX = 4000      # for ForkliftDriveFeedback.motor_speed_rpm clamp
WHEEL_RADIUS_M = 0.1525   # front drive wheel radius (matches XML)

# Front-low camera mount (in chassis body frame): looks along body +X (forks).
CAM_FWD_OFFSET_M = 0.55   # meters ahead of chassis origin
IMAGE_WIDTH_PX = 640
IMAGE_HEIGHT_PX = 480
PIXELS_PER_METER = 320.0  # camera scale: 1m lateral offset → ~half image width

# Line geometry (in world frame). Default: straight along world +X at y=0.
LINE_Y = 0.0


class MujocoDriverNode(Node):
    def __init__(self, model_path: str):
        super().__init__('mujoco_driver')

        # -------- MuJoCo --------
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Actuator IDs by name (so we don't depend on declaration order)
        self._act = {
            n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            for n in ('drive', 'steer', 'mast_tilt', 'fork_lift')
        }
        for k, v in self._act.items():
            if v < 0:
                raise RuntimeError(f"Actuator {k!r} not found in MuJoCo model")

        self._mj_lock = threading.Lock()
        self._step_count = 0

        # Optional initial pose offset (env-driven so demos can place the
        # forklift away from the line at startup). qpos layout for the root
        # freejoint: [x, y, z, qw, qx, qy, qz].
        init_y = float(os.environ.get('SIM_INIT_Y', '0.0'))
        init_yaw = float(os.environ.get('SIM_INIT_YAW_RAD', '0.0'))
        if init_y or init_yaw:
            self.data.qpos[1] = init_y
            self.data.qpos[3] = math.cos(init_yaw / 2.0)   # qw
            self.data.qpos[6] = math.sin(init_yaw / 2.0)   # qz
            mujoco.mj_forward(self.model, self.data)

        # -------- Command state --------
        self._cmd_drive = 0.0
        self._cmd_steer = 0.0
        self._cmd_lift_speed = 0.0
        self._cmd_tilt_speed = 0.0
        self._estop = False
        self._brake_released = False
        self._last_cmd_time = self.get_clock().now()

        # Integrated targets for position-controlled axes
        self._lift_target = 0.0
        self._tilt_target = 0.0

        # Odometer integration (cm, monotonically increasing magnitude)
        self._odometer_cm = 0.0
        self._last_odo_time = time.monotonic()

        # -------- ROS interface --------
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(
            ForkliftDirectCommand, '/safety/command', self._on_command, cmd_qos
        )

        self.pub_steer = self.create_publisher(Float32, '/forklift/steering_angle', 1)
        self.pub_drive_fb = self.create_publisher(ForkliftDriveFeedback, '/forklift/drive_feedback', 1)
        self.pub_fork_h = self.create_publisher(Float32, '/forklift/fork_height', 1)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        if HAVE_LINE_STATE:
            self.pub_line = self.create_publisher(LineState, '/drop_off/line_state', 10)
        else:
            self.pub_line = None

        self.create_timer(0.02, self._feedback_50hz)
        # Line-state at ~25 Hz (the real detector runs at ~20 fps).
        if self.pub_line is not None:
            self.create_timer(0.04, self._line_state_25hz)

        self.get_logger().info(
            f"mujoco_driver ready; model='{model_path}'  "
            f"subscribed to /safety/command  publishing /forklift/* + /joint_states"
        )

    # ---------- ROS callbacks ----------
    def _on_command(self, msg: ForkliftDirectCommand):
        self._cmd_drive = float(np.clip(msg.drive_speed, -1.0, 1.0))
        self._cmd_steer = float(np.clip(msg.steering_angle, -1.0, 1.0))
        self._cmd_lift_speed = float(np.clip(msg.lift_speed, -1.0, 1.0))
        self._cmd_tilt_speed = float(np.clip(msg.tilt_speed, -1.0, 1.0))
        self._estop = bool(msg.estop_active)
        self._brake_released = bool(msg.brake_interlock_released)
        self._last_cmd_time = self.get_clock().now()

    # ---------- Physics ----------
    def step_physics(self, dt: float) -> None:
        """Advance MuJoCo by `dt` worth of internal timesteps and apply the latest cmd."""
        steps = max(1, int(round(dt / self.model.opt.timestep)))

        # Snapshot command state (cheap; no lock needed for floats/bools, but
        # we lock the MuJoCo data section to keep the feedback timer consistent).
        if self._estop or not self._brake_released:
            drive = 0.0
        else:
            drive = self._cmd_drive

        steer_rad = self._cmd_steer * STEER_MAX_RAD
        ds_lift = self._cmd_lift_speed * LIFT_MAX_MPS
        ds_tilt = self._cmd_tilt_speed * TILT_MAX_RPS

        with self._mj_lock:
            # Integrate velocity-style commands into position targets, clamped to joint range.
            self._lift_target = float(np.clip(
                self._lift_target + ds_lift * dt, 0.0, 1.2
            ))
            self._tilt_target = float(np.clip(
                self._tilt_target + ds_tilt * dt, -0.105, 0.070
            ))

            self.data.ctrl[self._act['drive']] = drive
            self.data.ctrl[self._act['steer']] = steer_rad
            self.data.ctrl[self._act['fork_lift']] = self._lift_target
            self.data.ctrl[self._act['mast_tilt']] = self._tilt_target

            for _ in range(steps):
                mujoco.mj_step(self.model, self.data)
                self._step_count += 1

    # ---------- Feedback ----------
    def _feedback_50hz(self) -> None:
        with self._mj_lock:
            steer_q = float(self.data.joint('steering').qpos[0])
            lift_q = float(self.data.joint('lift').qpos[0])
            tilt_q = float(self.data.joint('tilt').qpos[0])
            drv_l_v = float(self.data.joint('drive_fl').qvel[0])
            drv_r_v = float(self.data.joint('drive_fr').qvel[0])
            rear_v = float(self.data.joint('roll_rear').qvel[0])
            drv_l_p = float(self.data.joint('drive_fl').qpos[0])
            drv_r_p = float(self.data.joint('drive_fr').qpos[0])
            rear_p = float(self.data.joint('roll_rear').qpos[0])

        # Steering angle (rad)
        m = Float32(); m.data = steer_q
        self.pub_steer.publish(m)

        # Fork height (m)
        m = Float32(); m.data = lift_q
        self.pub_fork_h.publish(m)

        # Drive feedback (synthesized from sim state)
        avg_omega = 0.5 * (drv_l_v + drv_r_v)        # rad/s
        rpm = avg_omega * 60.0 / (2.0 * math.pi)
        rpm = max(-DRIVE_RPM_MAX, min(DRIVE_RPM_MAX, rpm))

        # Odometer: integrate |v| at wheel contact between feedback ticks.
        now = time.monotonic()
        v_lin = abs(avg_omega) * WHEEL_RADIUS_M
        self._odometer_cm += v_lin * 100.0 * (now - self._last_odo_time)
        self._last_odo_time = now

        fb = ForkliftDriveFeedback()
        fb.motor_speed_rpm       = int(rpm)
        fb.motor_current_amps    = float(abs(rpm) * 0.05)   # synthetic, ~0-200A
        fb.drive_fault_code      = 0
        fb.battery_percent       = 90
        fb.odometer_cm           = int(self._odometer_cm)
        fb.estop_active          = self._estop
        fb.interlock_on          = self._brake_released
        fb.forward_switch        = self._cmd_drive > 0.01
        fb.reverse_switch        = self._cmd_drive < -0.01
        fb.mode_auto             = True
        fb.electromagnetic_brake = (not self._brake_released) or self._estop
        self.pub_drive_fb.publish(fb)

        # Joint states (consumed by sim2 tricycle_odometry & robot_state_publisher)
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = ['drive_fl', 'drive_fr', 'roll_rear', 'steering', 'lift', 'tilt']
        js.position = [drv_l_p, drv_r_p, rear_p, steer_q, lift_q, tilt_q]
        js.velocity = [drv_l_v, drv_r_v, rear_v, 0.0, 0.0, 0.0]
        self.pub_js.publish(js)

        # Odometry (consumed by line_follower x_correct travel check)
        with self._mj_lock:
            cx, cy, cz = (float(x) for x in self.data.body('chassis').xpos)
            cw, cqx, cqy, cqz = (float(x) for x in self.data.body('chassis').xquat)
            v_world = self.data.cvel[self.model.body('chassis').id]  # [angvel(3), linvel(3)]
            vx, vy = float(v_world[3]), float(v_world[4])
            wz = float(v_world[2])
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = cx
        odom.pose.pose.position.y = cy
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=cqx, y=cqy, z=cqz, w=cw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

    # ---------- Sim-derived line state ----------
    def _line_state_25hz(self) -> None:
        """Compute the LineState message the real line_detector would emit,
        directly from MuJoCo ground truth (no rendering / image processing).

        Sim line: straight along world +X at y=LINE_Y. Camera mounted
        CAM_FWD_OFFSET_M ahead of the chassis on the body +X axis, looking
        along body +X. Maps the world-frame offset and heading-error to the
        signed pixel offset / yaw-rad the controller expects.
        """
        with self._mj_lock:
            cx = float(self.data.body('chassis').xpos[0])
            cy = float(self.data.body('chassis').xpos[1])
            qw = float(self.data.body('chassis').xquat[0])
            qz = float(self.data.body('chassis').xquat[3])

        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        cam_x = cx + CAM_FWD_OFFSET_M * math.cos(yaw)
        cam_y = cy + CAM_FWD_OFFSET_M * math.sin(yaw)

        # Lateral offset of the line (perpendicular distance projected onto
        # the camera's local right axis). Line is at y=LINE_Y along +x.
        # Sign chosen so the controller's drive_dir(-1) * k_lat * offset_norm
        # term steers the rear-steer reverse-driving forklift toward the line
        # in our MuJoCo actuator convention. Equivalent to flipping
        # `offset_sign_front_low` on a bench setup.
        lateral_world = (cam_y - LINE_Y) * math.cos(yaw)
        offset_px = float(-lateral_world * PIXELS_PER_METER)

        # Yaw of the line relative to camera up-direction. Sign chosen so the
        # controller's k_yaw term provides negative feedback for our actuator
        # convention (analogous to `yaw_sign_front_low` per-camera flip).
        yaw_rad = float(-yaw)

        ls = LineState()
        ls.header.stamp = self.get_clock().now().to_msg()
        ls.header.frame_id = 'front_low_camera'
        # The sim "always sees" the line within ±10 m of origin along x.
        ls.guide_detected = abs(cam_x) < 18.0
        ls.guide_offset_px = offset_px
        ls.guide_yaw_rad = yaw_rad
        ls.guide_offset_m = float(lateral_world)
        ls.stop_detected = False
        ls.stop_distance_px = 0.0
        ls.yellow_stop_detected = False
        ls.yellow_stop_distance_px = 0.0
        ls.image_width = IMAGE_WIDTH_PX
        ls.image_height = IMAGE_HEIGHT_PX
        self.pub_line.publish(ls)


def _resolve_model_path() -> str:
    # Highest priority: env var. Otherwise auto-locate forklift.xml relative
    # to this file or the cwd.
    env = os.environ.get('FORKLIFT_XML')
    if env:
        return env

    here = Path(__file__).resolve()
    for cand in (
        here.parents[4] / 'forklift.xml',
        here.parents[3] / 'forklift.xml',
        Path.cwd() / 'forklift.xml',
        Path('/home/testbench3/cavalla-sim-forklift/forklift.xml'),
    ):
        if cand.is_file():
            return str(cand)
    raise FileNotFoundError("forklift.xml not found; set FORKLIFT_XML env var")


def main(args=None):
    rclpy.init(args=args)
    headless = os.environ.get('MUJOCO_HEADLESS', '0') == '1'

    model_path = _resolve_model_path()
    node = MujocoDriverNode(model_path)

    # ROS executor in background thread so spin() doesn't block the viewer.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Real-time pacing for physics. We step in small batches and the viewer
    # polls in the main thread.
    physics_dt = 0.02      # 50 Hz wall pacing — internal mj_step is finer
    last_wall = time.monotonic()

    if headless:
        node.get_logger().info("Running headless (no viewer); Ctrl-C to exit")
        try:
            while rclpy.ok():
                node.step_physics(physics_dt)
                now = time.monotonic()
                slack = (last_wall + physics_dt) - now
                if slack > 0:
                    time.sleep(slack)
                last_wall = time.monotonic()
        except KeyboardInterrupt:
            pass
    else:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            node.get_logger().info("Viewer up; press ESC in the window to quit")
            while rclpy.ok() and viewer.is_running():
                node.step_physics(physics_dt)
                viewer.sync()
                now = time.monotonic()
                slack = (last_wall + physics_dt) - now
                if slack > 0:
                    time.sleep(slack)
                last_wall = time.monotonic()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
