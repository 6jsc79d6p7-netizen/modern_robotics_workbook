# Modern Robotics — Learning Project

Working through *Modern Robotics: Mechanics, Planning, and Control*
(Lynch & Park) with intuition-first notes + from-scratch code.

- Theory notes live in `notes/`, code in `notebooks/`, shared helpers in `mr/`.
- Per-topic workflow: run `/mr-topic <topic>` (theory → discussion → code).
- See `CLAUDE.md` for the learning approach.

## Progress

| # | Chapter / Topic | Note | Notebook | Status |
|---|-----------------|------|----------|--------|
| 2 | Configuration Space | | | ⬜ not started |
| 3 | Rigid-Body Motions | | | ✅ done |
| 3a | ↳ Rotations & angular velocity | [03a](notes/03a_rotations.md) | [03a](notebooks/03a_rotations.ipynb) | ✅ done |
| 3b | ↳ Rigid-body motions & twists (SE(3)) | [03b](notes/03b_rigid_motions_twists.md) | [03b](notebooks/03b_rigid_motions_twists.ipynb) | ✅ done |
| 3c | ↳ Wrenches | [03c](notes/03c_wrenches.md) | — | ✅ done |
| 4 | Forward Kinematics | | | ✅ done |
| 4a | ↳ Product of Exponentials (space form) | [04a](notes/04a_forward_kinematics_space.md) | [mujoco](mujoco/) | ✅ done |
| 4b | ↳ PoE body form + URDF | [04b](notes/04b_forward_kinematics_body.md) | [mujoco](mujoco/) | ✅ done |
| 5 | Velocity Kinematics & Statics | | | ⬜ not started |
| 6 | Inverse Kinematics | | | ⬜ not started |
| 7 | Kinematics of Closed Chains | | | ⬜ not started |
| 8 | Dynamics of Open Chains | | | ⬜ not started |
| 9 | Trajectory Generation | | | ⬜ not started |
| 10 | Motion Planning | | | ⬜ not started |
| 11 | Robot Control | | | ⬜ not started |
| 12 | Grasping & Manipulation | | | ⬜ not started |
| 13 | Wheeled Mobile Robots | | | ⬜ not started |

Status legend: ⬜ not started · 🟨 in progress · ✅ done

## Up next

**Ch. 5 — Velocity Kinematics & Statics** — the **Jacobian** `J(θ)`: how joint
velocities map to the end-effector twist (`V = J θ̇`), and its transpose mapping
wrenches to joint torques (`τ = Jᵀ F`). Builds on the space/body PoE split from
Ch. 4 (space vs body Jacobian) and the twist/wrench duality from 3b/3c. Natural
MuJoCo follow-on: check `J` against finite-differenced FK and against MuJoCo's
`mj_jac`.

> **Chapter 4 — complete.** Theory notes 4a (space PoE) / 4b (body PoE + URDF).
> Practiced **hands-on in MuJoCo** (`mujoco/`): a 3R arm whose PoE forward
> kinematics (both forms, from `mr/se3.py`) matches MuJoCo's `mj_forward` to
> ~1e-16, plus headless renders. This is the preferred practice mode going
> forward wherever a simulator hook exists.

## MuJoCo (`mujoco/`)

Hands-on simulator experiments (the north star). MuJoCo 3.9 + imageio live in
`.venv`; rendering works headlessly.
- `arm3r.xml` — a 3R spatial arm (MJCF).
- `fk_check.py` — validates our PoE forward kinematics (space & body form) vs
  MuJoCo's `mj_forward`.
- `render.py` — renders the arm at a given joint configuration to a PNG.

## Parked topics (revisit later)

- **Quaternions & other rotation representations** (L&P Appendix B: Euler
  angles, roll–pitch–yaw, unit quaternions, Cayley–Rodrigues). The main text
  sticks to rotation matrices + exponential coordinates, so this isn't needed
  to progress. But simulators/hardware lean on quaternions heavily (MuJoCo,
  Isaac, ROS store orientation as `q ∈ S³` — no gimbal lock, stable to
  integrate). Slot in a short standalone note (`notes/0B_quaternions.md`) right
  when we start touching **MuJoCo**, where it becomes practically necessary.
  Key hook: `q = (cos(θ/2), ω̂ sin(θ/2))` — spends a 4th number to kill the
  `1/sinθ` singularity in the SO(3) matrix log.
