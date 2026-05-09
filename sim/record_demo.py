"""
Record an mp4 of the forklift simulation.

Runs the same 12-phase scripted demo as run_sim.py, renders frames offscreen
with mujoco.Renderer, and pipes them to ffmpeg.

Usage:
    python3 sim/record_demo.py [out.mp4] [--width 1280 --height 720 --fps 30]
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from pathlib import Path

# Use EGL for headless GL on Jetson/Tegra — no X window pops up.
os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = str(ROOT / 'forklift.xml')


def build_phases():
    DRV = 0.50
    STR = 0.10
    # ctrl order: [drive, steer, mast_tilt, fork_lift]
    return [
        ('SETTLE',         1.0, [ 0.0,  0.0,  0.0,   0.0]),
        ('DRIVE FORWARD',  3.0, [ DRV,  0.0,  0.0,   0.0]),
        ('STEER LEFT',     3.0, [ DRV,  STR,  0.0,   0.0]),
        ('TURN COMPLETE',  2.0, [ DRV,  0.0,  0.0,   0.0]),
        ('STEER RIGHT',    4.0, [ DRV, -STR,  0.0,   0.0]),
        ('STRAIGHTEN',     2.0, [ DRV,  0.0,  0.0,   0.0]),
        ('STOP + LIFT',    4.0, [ 0.0,  0.0,  0.0,   0.8]),
        ('TILT FORWARD',   1.5, [ 0.0,  0.0,  0.07,  0.8]),
        ('TILT BACK',      1.5, [ 0.0,  0.0, -0.10,  0.8]),
        ('TILT LEVEL',     1.5, [ 0.0,  0.0,  0.0,   0.8]),
        ('LOWER FORKS',    4.0, [ 0.0,  0.0,  0.0,   0.0]),
        ('REVERSE',        3.0, [-DRV,  0.0,  0.0,   0.0]),
    ]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out', nargs='?', default=str(ROOT / 'forklift_demo.mp4'))
    p.add_argument('--model', default=DEFAULT_MODEL)
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--fps', type=int, default=30)
    args = p.parse_args()

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = chassis_id
    cam.distance = 7.0
    cam.elevation = -22.0
    cam.azimuth = 135.0

    scene_opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(scene_opt)
    scene_opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = False  # hide axle tendon line

    sim_dt = model.opt.timestep
    frame_dt = 1.0 / args.fps
    steps_per_frame = max(1, int(round(frame_dt / sim_dt)))

    phases = build_phases()
    total_s = sum(d for _, d, _ in phases)
    total_frames = int(total_s * args.fps)
    print(f'Recording {total_s:.1f}s ({total_frames} frames) at {args.fps} fps → {args.out}',
          flush=True)

    ffmpeg = subprocess.Popen(
        [
            'ffmpeg', '-loglevel', 'error', '-y',
            '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-pix_fmt', 'rgb24',
            '-s', f'{args.width}x{args.height}',
            '-r', str(args.fps),
            '-i', '-',
            '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
            '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart',
            args.out,
        ],
        stdin=subprocess.PIPE,
    )

    frame_idx = 0
    for name, duration, ctrl in phases:
        n_frames = max(1, int(round(duration * args.fps)))
        print(f'  [{name}] {duration:.1f}s ({n_frames} frames)', flush=True)
        for _ in range(n_frames):
            for _ in range(steps_per_frame):
                data.ctrl[:] = ctrl
                mujoco.mj_step(model, data)
            renderer.update_scene(data, camera=cam, scene_option=scene_opt)
            frame = renderer.render()
            ffmpeg.stdin.write(frame.tobytes())
            frame_idx += 1

    ffmpeg.stdin.close()
    ffmpeg.wait()
    print(f'wrote {frame_idx} frames → {args.out}')


if __name__ == '__main__':
    main()
