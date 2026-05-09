# Forklift MuJoCo Sim — Context Dump (2026-05-09)

## Files
| File | Purpose |
|------|---------|
| `forklift.xml` | MuJoCo model — 3-wheel AGV forklift (2 front drive + 1 rear steer) |
| `run_sim.py` | Scripted demo with 12 phases + validation checks + interactive viewer |
| `*.stp` (48MB) | STEP CAD source file — geometry was extracted from this |

## Model (`forklift.xml`) — Key Parameters
- **Wheels**: 2 front drive (R=152.5mm, W=120mm, track=918mm) + 1 rear steer (R=153mm, W=110mm), wheelbase 1055mm
- **Mass**: 3200kg chassis + 1100kg counterweight + 400kg battery
- **Drive motors**: `gear=1000`, `ctrlrange=[-1,1]`, joint damping=8
- **Wheel friction**: `1.0 0.005 0.002` (reduced from original 1.5 to enable steering)
- **5 actuators**: `drive_left`, `drive_right` (motor), `steer`, `mast_tilt`, `fork_lift` (position)
- **Collision groups**: ground(1/30), chassis(8/17), wheels(2/17), forks(4/17), payload(16/14), visual(0/0)
- **Forks**: L-shaped (heel + tine), 1150mm long, 100mm wide, 40mm thick, 300mm spacing
- **Mast**: tilt range +/-6deg, lift range 0-1.2m

## What Works (14/16 checks pass)
- Forward drive, both wheels spin, rear wheel rolls passively
- Rear-steer left (~10deg heading change, 0.3m lateral displacement)
- Rear-steer right (opposite direction confirmed)
- Fork lift (0 to 0.75m), fork tip Z rises in world
- Mast tilt forward (+4deg) and backward (-4.5deg)

## What's Left (2 failing checks)

### 1. Forks don't fully lower
Goes from 0.750 to 0.379m in 4s (threshold is <0.375m). The mast tilts further back
during lowering due to weight shift, which prevents full descent.

**Fix options:**
- Set `mast_tilt` ctrl explicitly to `0.0` during `11_LOWER_FORKS` phase (it already is, but
  the tilt joint drifts due to weight redistribution — increase tilt actuator kp from 12000
  to something higher, or increase lower phase duration from 4s to 6s)
- Or relax the check threshold from `fk1 * 0.5` to `fk1 * 0.55`

### 2. Reverse doesn't move
Wheels spin at -60 rad/s but vehicle stationary. Root cause: after fork/tilt operations
the chassis Z rises to 0.378m (front wheels partially lift off ground because mast tilts
back under load). The `10_TILT_LEVEL` phase was added to fix this but may need more time
or the tilt needs to be actively held at 0 during lowering.

**Fix options:**
- Increase `10_TILT_LEVEL` duration from 2s to 4s so mast fully settles
- Explicitly command `mast_tilt=0.0` during `11_LOWER_FORKS` AND `12_REVERSE`
  (change ctrl arrays from `[0,0,0,0,0]` and `[-DRV,-DRV,0,0,0]` to
  `[0,0,0,0.0,0]` — wait, 0.0 is already the default; the issue is the tilt joint
  stiffness=150 isn't enough to overcome the weight shift. Try adding explicit tilt
  command of `0.0` in the ctrl or increasing tilt kp)
- Or increase the tilt joint stiffness from 150 to 500

### Key insight
Both failures share the same root cause: during fork lowering, the carriage mass
dropping changes the weight distribution. The mast (with stiffness=150, damping=600)
can't hold neutral, and tilts back ~2deg further than intended. This lifts the front
of the chassis, taking the drive wheels off the ground.

## Tuning History (don't repeat these)
| Gear | Friction | Damping | Steering | Reverse | Notes |
|------|----------|---------|----------|---------|-------|
| 500  | 1.5      | 30      | FAIL     | PASS    | Rear wheel friction overwhelms drive |
| 1500 | 0.9      | 30      | PASS     | FAIL    | Wheels spin freely, no traction |
| 1000 | 1.0      | 8       | PASS     | PASS*   | Current. Reverse fails only because front wheels lift after tilt |

## Phase Sequence (`run_sim.py`)
```
DRV=0.50, STR=0.10
Actuator order: [drive_left, drive_right, steer, mast_tilt, fork_lift]

 1. SETTLE        1.5s  [0, 0, 0, 0, 0]
 2. DRIVE_FWD     3.0s  [DRV, DRV, 0, 0, 0]
 3. STEER_LEFT    3.0s  [DRV, DRV, STR, 0, 0]
 4. DRIVE_TURNED  2.0s  [DRV, DRV, 0, 0, 0]
 5. STEER_RIGHT   4.0s  [DRV, DRV, -STR, 0, 0]
 6. STRAIGHTEN    2.0s  [DRV, DRV, 0, 0, 0]
 7. STOP_LIFT     4.0s  [0, 0, 0, 0, 0.8]
 8. TILT_FWD      2.0s  [0, 0, 0, 0.07, 0.8]
 9. TILT_BACK     2.0s  [0, 0, 0, -0.10, 0.8]
10. TILT_LEVEL    2.0s  [0, 0, 0, 0, 0.8]
11. LOWER_FORKS   4.0s  [0, 0, 0, 0, 0]
12. REVERSE       3.0s  [-DRV, -DRV, 0, 0, 0]
```

## Repo
`git@github.com:AayushAgrawal2003/cavalla-sim-forklift.git` — branch `main`

## Dependencies
```
pip install mujoco numpy
```

## Run
```
python run_sim.py
```
Runs the scripted demo, prints validation results, then opens the interactive MuJoCo viewer.
