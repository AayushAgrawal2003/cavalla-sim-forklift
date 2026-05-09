"""
Record an mp4 of the closed-loop line-following demo.

Subscribes to /joint_states and the chassis pose via /odom (both published by
the bridge), or — if launched standalone — runs a fresh MuJoCo instance and
mirrors the controller's commands from /safety/command.

This script is the simplest path: it spins up its own MuJoCo, subscribes to
/safety/command, drives the sim, and renders frames offscreen → mp4.

Pre-req: the rest of the line-follow stack (safety_node, mux, adapter,
line_follower_controller, line_follow_runner) must be running already so
/safety/command is alive.

Usage:
    python3 sim/record_line_follow.py [out.mp4] [--duration 30 --init-y 0.5]
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from forklift_msgs.msg import ForkliftDirectCommand  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent

LIFT_MAX_MPS = 0.40
TILT_MAX_RPS = 0.50
STEER_MAX_RAD = 1.22


class CmdMirror(Node):
    def __init__(self):
        super().__init__('record_line_follow_mirror')
        self.create_subscription(ForkliftDirectCommand, '/safety/command',
                                 self._on_cmd, 1)
        self.cmd = ForkliftDirectCommand()

    def _on_cmd(self, msg: ForkliftDirectCommand):
        self.cmd = msg


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out', nargs='?', default=str(ROOT / 'forklift_line_follow.mp4'))
    p.add_argument('--model', default=str(ROOT / 'forklift.xml'))
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--duration', type=float, default=30.0)
    p.add_argument('--init-y', type=float, default=0.5)
    p.add_argument('--init-yaw', type=float, default=0.0)
    args = p.parse_args()

    rclpy.init()
    mirror = CmdMirror()
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(mirror), daemon=True)
    spin_thread.start()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    # Initial pose: forklift offset from line so we can see convergence.
    data.qpos[1] = args.init_y
    data.qpos[3] = math.cos(args.init_yaw / 2.0)
    data.qpos[6] = math.sin(args.init_yaw / 2.0)
    mujoco.mj_forward(model, data)

    act = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
           for n in ('drive', 'steer', 'mast_tilt', 'fork_lift')}

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
    cam.distance = 9.0
    cam.elevation = -30.0
    cam.azimuth = 110.0

    sim_dt = model.opt.timestep
    frame_dt = 1.0 / args.fps
    steps_per_frame = max(1, int(round(frame_dt / sim_dt)))

    ffmpeg = subprocess.Popen(
        [
            'ffmpeg', '-loglevel', 'error', '-y',
            '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-s', f'{args.width}x{args.height}',
            '-r', str(args.fps),
            '-i', '-',
            '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
            args.out,
        ],
        stdin=subprocess.PIPE,
    )

    total_frames = int(args.duration * args.fps)
    print(f'Recording {args.duration:.1f}s ({total_frames} frames) → {args.out}',
          flush=True)

    # Velocity-cmd integrators (same as bridge)
    lift_target = 0.0
    tilt_target = 0.0

    for i in range(total_frames):
        cmd = mirror.cmd
        if cmd.estop_active or not cmd.brake_interlock_released:
            drive_n = 0.0
        else:
            drive_n = float(np.clip(cmd.drive_speed, -1.0, 1.0))
        steer_n = float(np.clip(cmd.steering_angle, -1.0, 1.0))

        ds_lift = float(np.clip(cmd.lift_speed, -1.0, 1.0)) * LIFT_MAX_MPS
        ds_tilt = float(np.clip(cmd.tilt_speed, -1.0, 1.0)) * TILT_MAX_RPS
        lift_target = float(np.clip(lift_target + ds_lift * frame_dt, 0.0, 1.2))
        tilt_target = float(np.clip(tilt_target + ds_tilt * frame_dt, -0.105, 0.070))

        for _ in range(steps_per_frame):
            data.ctrl[act['drive']] = drive_n
            data.ctrl[act['steer']] = steer_n * STEER_MAX_RAD
            data.ctrl[act['fork_lift']] = lift_target
            data.ctrl[act['mast_tilt']] = tilt_target
            mujoco.mj_step(model, data)

        renderer.update_scene(data, camera=cam)
        ffmpeg.stdin.write(renderer.render().tobytes())

        if i % 30 == 0:
            x, y = data.body('chassis').xpos[0], data.body('chassis').xpos[1]
            print(f'  t={i*frame_dt:5.1f}s pos=({x:+6.2f},{y:+6.2f}) '
                  f'cmd[drv={drive_n:+.3f} steer={steer_n:+.3f}]', flush=True)

    ffmpeg.stdin.close()
    ffmpeg.wait()
    print(f'wrote {total_frames} frames → {args.out}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
