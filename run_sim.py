"""
Scripted demo of the 1MBV15R30 AGV forklift (3-wheel config) in MuJoCo.

Exercises all key functionalities:
  1. Settle
  2. Drive forward (differential drive, both front wheels)
  3. Steer left while driving (continuous momentum)
  4. Continue driving to show lateral displacement
  5. Steer right while driving (continuous momentum)
  6. Continue driving to show opposite lateral displacement
  7. Stop and raise forks
  8. Tilt mast forward then back
  9. Lower forks
  10. Drive in reverse

Actuator layout (5 actuators):
  [0] drive_left   — motor on front-left wheel
  [1] drive_right   — motor on front-right wheel
  [2] steer         — position ctrl on rear steering
  [3] mast_tilt     — position ctrl on mast tilt
  [4] fork_lift     — position ctrl on fork lift
"""

import mujoco
import mujoco.viewer
import numpy as np
import math
import os

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "forklift.xml")

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Actuator indices
ACT_DRV_L = 0
ACT_DRV_R = 1
ACT_STEER = 2
ACT_TILT  = 3
ACT_LIFT  = 4

log = {
    "time": [], "x": [], "y": [], "z": [], "heading_deg": [],
    "speed": [], "drv_l_wvel": [], "drv_r_wvel": [], "rear_wvel": [],
    "steer_deg": [], "fork_h": [], "tilt_deg": [],
    "fork_tip_z": [], "phase": [],
}


def quat_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def record(phase):
    pos = data.body("chassis").xpos.copy()
    quat = data.body("chassis").xquat.copy()
    vel = data.cvel[model.body("chassis").id][3:].copy()
    log["time"].append(data.time)
    log["x"].append(pos[0])
    log["y"].append(pos[1])
    log["z"].append(pos[2])
    log["heading_deg"].append(math.degrees(quat_to_yaw(quat)))
    log["speed"].append(np.linalg.norm(vel))
    log["drv_l_wvel"].append(data.joint("drive_fl").qvel[0])
    log["drv_r_wvel"].append(data.joint("drive_fr").qvel[0])
    log["rear_wvel"].append(data.joint("roll_rear").qvel[0])
    log["steer_deg"].append(math.degrees(data.joint("steering").qpos[0]))
    log["fork_h"].append(data.joint("lift").qpos[0])
    log["tilt_deg"].append(math.degrees(data.joint("tilt").qpos[0]))
    log["fork_tip_z"].append(data.geom("fork_l_tip").xpos[2])
    log["phase"].append(phase)


def run_phase(name, duration, ctrl):
    steps = int(duration / model.opt.timestep)
    log_every = int(0.1 / model.opt.timestep)
    for s in range(steps):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        if s % log_every == 0:
            record(name)


print("=" * 70)
print("  FORKLIFT AGV — 3-WHEEL FUNCTIONAL DEMO")
print("  Model: 1MBV15R30V995V1  |  2 front drive, 1 rear steer")
print("=" * 70)

DRV = 0.50
STR = 0.10

#                                         drv_l  drv_r  steer   tilt    lift
phases = [
    ("1_SETTLE",         1.5, [ 0.0,   0.0,   0.0,    0.0,    0.0  ]),
    ("2_DRIVE_FWD",      3.0, [ DRV,   DRV,   0.0,    0.0,    0.0  ]),
    ("3_STEER_LEFT",     3.0, [ DRV,   DRV,   STR,    0.0,    0.0  ]),
    ("4_DRIVE_TURNED",   2.0, [ DRV,   DRV,   0.0,    0.0,    0.0  ]),
    ("5_STEER_RIGHT",    4.0, [ DRV,   DRV,  -STR,    0.0,    0.0  ]),
    ("6_STRAIGHTEN",     2.0, [ DRV,   DRV,   0.0,    0.0,    0.0  ]),
    ("7_STOP_LIFT",      4.0, [ 0.0,   0.0,   0.0,    0.0,    0.8  ]),
    ("8_TILT_FWD",       2.0, [ 0.0,   0.0,   0.0,    0.07,   0.8  ]),
    ("9_TILT_BACK",      2.0, [ 0.0,   0.0,   0.0,   -0.10,   0.8  ]),
    ("10_TILT_LEVEL",    2.0, [ 0.0,   0.0,   0.0,    0.0,    0.8  ]),
    ("11_LOWER_FORKS",   4.0, [ 0.0,   0.0,   0.0,    0.0,    0.0  ]),
    ("12_REVERSE",       3.0, [-DRV,  -DRV,   0.0,    0.0,    0.0  ]),
]

for name, dur, ctrl in phases:
    print(f"\n  [{name}] ({dur}s)")
    run_phase(name, dur, ctrl)
    pos = data.body("chassis").xpos
    print(f"    pos=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:.3f})  "
          f"hdg={log['heading_deg'][-1]:+.1f}°  "
          f"fork={log['fork_h'][-1]:.3f}m  "
          f"tilt={log['tilt_deg'][-1]:+.2f}°  "
          f"steer={log['steer_deg'][-1]:+.1f}°  "
          f"drv_l={log['drv_l_wvel'][-1]:+.1f}  "
          f"drv_r={log['drv_r_wvel'][-1]:+.1f}rad/s")

# ==================== Validation ====================
print("\n" + "=" * 70)
print("  VALIDATION")
print("=" * 70)

checks = []


def check(name, passed, detail=""):
    checks.append((name, passed))
    print(f"  [{'PASS' if passed else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def phase_idx(p):
    return [i for i, ph in enumerate(log["phase"]) if ph == p]


# Stability (allow lean during turns; reject flying off)
z_dev = max(abs(z - 0.30) for z in log["z"])
check("Chassis Z stable (no flying off)", z_dev < 0.15, f"max dev={z_dev:.4f}m")

# Forward drive
idx = phase_idx("2_DRIVE_FWD")
dx = log["x"][idx[-1]] - log["x"][idx[0]]
check("Forward drive moves +X", dx > 0.3, f"dx={dx:.3f}m")

# Drive wheels spin
avg_wl = np.mean([log["drv_l_wvel"][i] for i in idx])
avg_wr = np.mean([log["drv_r_wvel"][i] for i in idx])
check("Front drive wheels spin", abs(avg_wl) > 0.5 and abs(avg_wr) > 0.5,
      f"L={avg_wl:.2f} R={avg_wr:.2f}rad/s")

# Rear wheel rolls passively
avg_rear = np.mean([abs(log["rear_wvel"][i]) for i in idx])
check("Rear wheel rolls passively", avg_rear > 0.1, f"rear={avg_rear:.2f}rad/s")

# Steering left: heading change during steer phase
idx_l = phase_idx("3_STEER_LEFT")
hdg_l_start = log["heading_deg"][idx_l[0]]
hdg_l_end = log["heading_deg"][idx_l[-1]]
hdg_delta_l = hdg_l_end - hdg_l_start
steer_l = log["steer_deg"][idx_l[-1]]
check("Steer left: wheel turns", abs(steer_l) > 5, f"steer={steer_l:+.1f}°")
check("Steer left: heading changes", abs(hdg_delta_l) > 3, f"Δheading={hdg_delta_l:+.1f}°")

# Y displacement: measured across steer + drive_turned phases for full arc
idx_driven = phase_idx("4_DRIVE_TURNED")
y_before_turn = log["y"][idx_l[0]]
y_after_arc = log["y"][idx_driven[-1]]
dy_l = y_after_arc - y_before_turn
check("Steer left: lateral displacement", abs(dy_l) > 0.1, f"dy={dy_l:+.2f}m")

# Steering right: heading change during steer phase
idx_r = phase_idx("5_STEER_RIGHT")
hdg_r_start = log["heading_deg"][idx_r[0]]
hdg_r_end = log["heading_deg"][idx_r[-1]]
hdg_delta_r = hdg_r_end - hdg_r_start
steer_r = log["steer_deg"][idx_r[-1]]
check("Steer right: wheel turns opposite", steer_r * steer_l < 0,
      f"left={steer_l:+.1f}° right={steer_r:+.1f}°")
check("Steer right: heading reverses direction", hdg_delta_r * hdg_delta_l < 0,
      f"left Δ={hdg_delta_l:+.1f}° right Δ={hdg_delta_r:+.1f}°")

# Fork lift
idx_lift = phase_idx("7_STOP_LIFT")
fk0 = log["fork_h"][idx_lift[0]]
fk1 = log["fork_h"][idx_lift[-1]]
check("Forks rise", fk1 > fk0 + 0.15, f"{fk0:.3f}→{fk1:.3f}m")

tip0 = log["fork_tip_z"][idx_lift[0]]
tip1 = log["fork_tip_z"][idx_lift[-1]]
check("Fork tip Z rises in world", tip1 > tip0 + 0.10, f"{tip0:.3f}→{tip1:.3f}m")

# Tilt forward
idx_tf = phase_idx("8_TILT_FWD")
tilt_f = log["tilt_deg"][idx_tf[-1]]
check("Mast tilts forward", tilt_f > 1.0, f"tilt={tilt_f:+.2f}°")

# Tilt back
idx_tb = phase_idx("9_TILT_BACK")
tilt_b = log["tilt_deg"][idx_tb[-1]]
check("Mast tilts backward", tilt_b < -1.0, f"tilt={tilt_b:+.2f}°")

# Lower forks
idx_low = phase_idx("11_LOWER_FORKS")
fk_low = log["fork_h"][idx_low[-1]]
check("Forks lower back", fk_low < fk1 * 0.5, f"from {fk1:.3f}→{fk_low:.3f}m")

# Reverse (measure distance traveled, not world-X, since heading may have changed)
idx_rev = phase_idx("12_REVERSE")
rev_x0, rev_y0 = log["x"][idx_rev[0]], log["y"][idx_rev[0]]
rev_x1, rev_y1 = log["x"][idx_rev[-1]], log["y"][idx_rev[-1]]
rev_dist = math.sqrt((rev_x1 - rev_x0)**2 + (rev_y1 - rev_y0)**2)
check("Reverse moves backward", rev_dist > 0.2, f"dist={rev_dist:.3f}m")

# Drive wheels spin backward in reverse
avg_rev_wl = np.mean([log["drv_l_wvel"][i] for i in idx_rev])
check("Drive wheels reverse", avg_rev_wl < -0.3, f"avg_L={avg_rev_wl:.2f}rad/s")

passed = sum(1 for _, p in checks if p)
total = len(checks)
print(f"\n  Result: {passed}/{total} checks passed")
if passed == total:
    print("  ALL CHECKS PASSED")

# ==================== Log table ====================
print("\n" + "=" * 70)
print("  MOTION LOG (every 0.5s)")
print("=" * 70)
print(f"  {'t':>5} {'phase':<16} {'X':>7} {'Y':>7} {'Z':>6} "
      f"{'hdg°':>6} {'spd':>5} {'drvL':>6} {'drvR':>6} {'rear':>6} "
      f"{'str°':>6} {'lift':>5} {'tlt°':>6} {'tipZ':>5}")
print("  " + "-" * 114)
for i in range(0, len(log["time"]), 5):
    print(f"  {log['time'][i]:5.1f} {log['phase'][i]:<16} "
          f"{log['x'][i]:+7.3f} {log['y'][i]:+7.3f} {log['z'][i]:6.3f} "
          f"{log['heading_deg'][i]:+6.1f} {log['speed'][i]:5.3f} "
          f"{log['drv_l_wvel'][i]:+6.2f} {log['drv_r_wvel'][i]:+6.2f} {log['rear_wvel'][i]:+6.2f} "
          f"{log['steer_deg'][i]:+6.1f} {log['fork_h'][i]:5.3f} "
          f"{log['tilt_deg'][i]:+6.2f} {log['fork_tip_z'][i]:5.3f}")

print("\n" + "=" * 70)
print("  Opening interactive viewer... (use actuator sliders on the right)")
print("=" * 70)

mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)
mujoco.viewer.launch(model, data)
