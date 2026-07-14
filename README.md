# Robotics Self-Study

Intuition-first, build-oriented self-study of robotics, worked **book-by-book**
toward **deep-learning robotics** (language-conditioned manipulation + semantic
nav on a wheeled mobile manipulator). Notes lead with geometry and worked
examples; code is from-scratch first, then cross-checked.

- **[Modern Robotics](notes/modern_robotics/)** (Lynch & Park — [book PDF](https://hades.mech.northwestern.edu/images/7/7f/MR.pdf)) — ✅ **complete** (all 13 chapters).
- **[Underactuated Robotics](notes/underactuated/)** (MIT 6.832, Russ Tedrake — [online text](https://underactuated.csail.mit.edu/index.html)) — 🟨 **up next**.
- Notes live in **book-wise folders** under `notes/` (`modern_robotics/`,
  `underactuated/`, each with its own `figures/`). Cross-cutting build notes
  (`proj_*.md`) and the project `00_roadmap.md` stay at `notes/` root.
- Code in `notebooks/` + `mujoco/`, shared helpers in `mr/`. See `CLAUDE.md` for
  the learning approach.

## 🎬 Milestone — learned pick-and-place in sim

First working policy from the practical build: a **Diffusion Policy** (vision +
state, conv-U-Net, DDIM sampling) trained entirely in MuJoCo on **clean scripted +
teleop demos**, doing a target-highlight-conditioned pick-and-place on a Franka.

![Diffusion Policy pick-and-place](pick_place/media/dp_pickplace.gif)

The arm grasps the highlighted object and drops it in the highlighted bin
(grounding = a mask → SAM/Grounding-DINO on hardware). Diffusion reached **~60%**
at its best checkpoint vs **~10%** for ACT on *identical* data — see
[`pick_place/`](pick_place/) and the
[iteration-loop note](notes/proj_iteration_loop.md) for how cleaning the data
(removing empty-gripper "successes") made that comparison trustworthy.

## Progress

| # | Chapter / Topic | Note | Notebook | Status |
|---|-----------------|------|----------|--------|
| 2 | Configuration Space (DOF, C-space, topology) | | | ✅ folded into Ch 3+ |
| 3 | Rigid-Body Motions | | | ✅ done |
| 3a | ↳ Rotations & angular velocity | [03a](notes/modern_robotics/03a_rotations.md) | [03a](notebooks/03a_rotations.ipynb) | ✅ done |
| 3b | ↳ Rigid-body motions & twists (SE(3)) | [03b](notes/modern_robotics/03b_rigid_motions_twists.md) | [03b](notebooks/03b_rigid_motions_twists.ipynb) | ✅ done |
| 3c | ↳ Wrenches | [03c](notes/modern_robotics/03c_wrenches.md) | — | ✅ done |
| 4 | Forward Kinematics | | | ✅ done |
| 4a | ↳ Product of Exponentials (space form) | [04a](notes/modern_robotics/04a_forward_kinematics_space.md) | [mujoco](mujoco/) | ✅ done |
| 4b | ↳ PoE body form + URDF | [04b](notes/modern_robotics/04b_forward_kinematics_body.md) | [mujoco](mujoco/) | ✅ done |
| 5 | Velocity Kinematics & Statics | | | ✅ done |
| 5a | ↳ The Jacobian (space + body) | [05a](notes/modern_robotics/05a_jacobian_velocity_kinematics.md) | Ex 5.3 (by hand) | ✅ done |
| 5b | ↳ Statics, singularities, manipulability | [05b](notes/modern_robotics/05b_statics_singularities_manipulability.md) | Ex 5.3 (by hand) | ✅ done |
| 6 | Inverse Kinematics | | | ✅ done |
| 6a | ↳ The IK problem & analytic solutions | [06a](notes/modern_robotics/06a_inverse_kinematics_analytic.md) | Ex 6.10 (by hand) | ✅ done |
| 6b | ↳ Numerical IK (Newton–Raphson, pseudoinverse) | [06b](notes/modern_robotics/06b_inverse_kinematics_numerical.md) | Ex 6.10 (by hand) | ✅ done |
| 7 | Kinematics of Closed Chains | | | ✅ done |
| 7a | ↳ Parallel mechanisms, IK/FK, delta robot | [07a](notes/modern_robotics/07a_closed_chains_kinematics.md) | — (design exercise) | ✅ done |
| 7b | ↳ Differential kinematics & singularities | [07b](notes/modern_robotics/07b_differential_kinematics_singularities.md) | — (design exercise) | ✅ done |
| 7c | ↳ Applied: haptic teleop (delta + wrist) | [07c](notes/modern_robotics/07c_haptic_teleop_delta.md) | — (design exercise) | ✅ done |
| 8 | Dynamics of Open Chains | | | ✅ done |
| 8a | ↳ Manipulator eqn, fwd/inv dynamics, Newton–Euler, contact, sim-vs-real model | [08a](notes/modern_robotics/08a_open_chain_dynamics.md) | — (haptic teleop exercise) | ✅ done |
| 8b | ↳ Applied: self-dynamics compensation (haptics) | [08b](notes/modern_robotics/08b_self_dynamics_compensation.md) | — (design exercise) | ✅ done |
| 9 | Trajectory Generation | [09](notes/modern_robotics/09_trajectory_generation.md) | [DP pick-place](pick_place/) 🎬 | ✅ done |
| 9–10 | ↳ Learned SOTA (combined) | [09_10s](notes/modern_robotics/09_10_learned_sota.md) | [DP pick-place](pick_place/) 🎬 | ✅ done |
| 10 | Motion Planning | [10](notes/modern_robotics/10_motion_planning.md) | [DP pick-place](pick_place/) 🎬 | ✅ done |
| 11 | Robot Control | | | ✅ done |
| 11a | ↳ Control fundamentals, PID, position/velocity/torque, computed torque | [11a](notes/modern_robotics/11a_control_fundamentals.md) | — (build-focused) | ✅ done |
| 11b | ↳ Force / hybrid / impedance / admittance control (contact-rich) | [11b](notes/modern_robotics/11b_impedance_force_control.md) | [CartesianImpedanceController](pick_place/cartesian_controller.py) | ✅ done |
| 12 | Grasping & Manipulation | | | ✅ done |
| 12a | ↳ What makes a grasp hold (friction cone, form/force closure, grasp matrix) | [12a](notes/modern_robotics/12a_grasping_theory.md) | — | ✅ done |
| 12b | ↳ Learned grasping (Contact-GraspNet / AnyGrasp) | [12b](notes/modern_robotics/12b_learned_grasping.md) | — | ✅ done |
| 13 | Wheeled Mobile Robots | | | ✅ done |
| 13a | ↳ Overview, holonomic vs nonholonomic, omnidirectional kinematics | [13a](notes/modern_robotics/13a_wheeled_overview_omnidirectional.md) | [mecanum](mujoco/) | ✅ done |
| 13b | ↳ Nonholonomic robots (diff-drive/car), constraint, controllability | [13b](notes/modern_robotics/13b_nonholonomic_robots.md) | — | ✅ done |
| 13c | ↳ Odometry + mobile manipulation (combined Jacobian) | [13c](notes/modern_robotics/13c_odometry_mobile_manipulation.md) | [mecanum](mujoco/) | ✅ done |

Status legend: ⬜ not started · 🟨 in progress · ✅ done

## 🎉 Modern Robotics — complete

All 13 chapters done — the full geometric + control language (SE(3), forward
kinematics, the Jacobian, IK, closed chains, dynamics, trajectory/motion planning,
control, grasping, and wheeled mobile manipulation), each with intuition-first
notes, figures, and either by-hand exercises or a MuJoCo demo. **Ch 13 (wheeled
mobile robots) was the culmination** — the combined mobile-manipulator Jacobian
`[J_base | J_arm]` pulls in Ch 3/4/5/6/11 (and 12) at once. Per-chapter recaps below.

## Next: Underactuated Robotics (MIT 6.832, Russ Tedrake)

The next book/course, in this same repo under
[`notes/underactuated/`](notes/underactuated/). Where *Modern Robotics* treated the
robot as a kinematic / quasi-static object you command directly, **underactuated**
robotics is about systems you *can't* just command — dynamics you must **exploit
rather than cancel**: balance, legged locomotion, dynamic/acrobatic manipulation,
and the optimal-control machinery (LQR, dynamic programming, trajectory
optimization / direct collocation, RL) that learned policies for those systems sit
on. On the north-star path once the statically-stable wheeled base is solid. See
[`notes/underactuated/00_overview.md`](notes/underactuated/00_overview.md).

## Chapter recaps (Modern Robotics)

**Ch. 9 & 10 — done ✅** (theory + learned SOTA, and the "toy" realized as the
**pick-place milestone above** — a working Diffusion Policy). Per CLAUDE.md these
were *special handling* (gist + SOTA + toy). We taught **9 and 10's SOTA together**
(user's call — the learned methods dissolve the Ch9/Ch10 boundary). Three notes:
- [09](notes/modern_robotics/09_trajectory_generation.md) — classical trajectory-gen gist (path ×
  time scaling; joint/task/SE(3) straight lines; cubic/quintic/trapezoid/S-curve
  jerk ladder; via points; the `(s,ṡ)` phase-plane time-optimal picture).
- [10](notes/modern_robotics/10_motion_planning.md) — classical motion-planning gist (the
  piano-mover's problem; **C-space obstacles**; A\*; grid/sampling(RRT,RRT\*,PRM)/
  potential-field/optimization taxonomy; completeness ladder).
- [09_10s](notes/modern_robotics/09_10_learned_sota.md) — **keystone SOTA note**: the
  classical→learned spectrum (classical+TOPP → cuRobo → MπNets → **ACT/Diffusion
  Policy** → **VLA**), action chunking, multimodality, the **VLN/semantic-nav**
  branch, and what MR substrate survives underneath (this is the Phase-0→robot-
  learning bridge).

**PIVOT — the practical build is up and working.** The **MuJoCo pick-and-place
build** is running toward the north star (roadmap M1→M2→M3), and Ch 9–10's learned
SOTA is now *realized*, not just read. **Done:** a **Franka Panda** env with
**phone 6-DoF teleop** ([`proj_phone_teleop.md`](notes/proj_phone_teleop.md),
also the best hands-on Ch-3 SE(3)/transform-tree exercise), including a
**walled-bin place challenge**; a **scripted privileged-state expert + human
teleop** demo pipeline feeding a **cleaned dataset** (after catching empty-gripper
"successes" — see the [iteration-loop note](notes/proj_iteration_loop.md)); and two
policies trained on *identical* clean data — **ACT** (~10%) vs a **Diffusion
Policy** (~60%, the milestone above; sampling beats ACT's mode-averaging on the
multimodal grasp). **Next:** a **DiT / flow-matching** run (Rung 4) for a
three-way comparison, then **π0** (Rung 5), more teleop data to push past the
overfitting wall, and **real hardware** (SO-100). CUDA parts
(training/π0/cuRobo/AnyGrasp) run on **cloud GPU**; the Mac does env, teleop, and
eval. **MR book now complete; next course = Underactuated Robotics.**

> **Ch. 13 — complete** (the culmination; elevated Tier-2 — the chosen embodiment).
> Notes 13a (omnidirectional/mecanum bases, holonomic vs **nonholonomic**, the
> `H` wheel-Jacobian + the wheel-force/decomposition FAQs), 13b (diff-drive/car,
> the Pfaffian **no-sideways-slip** constraint, the **parallel-park-wiggle /
> Lie-bracket** controllability picture, Dubins/Reeds–Shepp + feedback gist), 13c
> (**odometry** via `F=H†` + screw integration, and the **combined
> mobile-manipulator Jacobian** `V_e=[J_base|J_arm][u;θ̇]`, `[u;θ̇]=J_e†V`).
> Practiced via a **MuJoCo mecanum demo** (`mujoco/mecanum_kinematics.py`):
> `F·H=I₃`, matches the book's `F`, and shows **odometry drift** under wheel slip.
> A deep **9-entry 13c FAQ** captures the frames discussion (which frame `F`/`V_d`/
> `X_d` live in, superposition of `J_base+J_arm`, reading the task-space control
> law, and — for the mobile pick-place build — **emit body-frame EE deltas**, the
> `ΔX=T_ed` shortcut, and how `J_e` handles a nonholonomic (and even a quadruped) base).

> **Ch. 11 & 12 — complete.** 11a (control fundamentals: PID, position/velocity/
> torque, computed torque), 11b (force/hybrid/**impedance**/admittance for
> contact-rich tasks; realized as `pick_place/cartesian_controller.py`). 12a (what
> makes a grasp hold: contacts, the **friction cone**, form vs **force closure**,
> the grasp matrix — with figure), 12b (the **learned** replacement:
> Contact-GraspNet / AnyGrasp eyeballing a grasp from a point cloud).

> **Ch. 8 — complete** (Tier-2, conceptual; Lagrangian derivations skipped). Notes
> 8a (core) + 8b (applied). 8a covers the manipulator equation `τ = M(θ)θ̈ +
> C(θ,θ̇)θ̇ + g(θ) + f(θ̇)`, **forward vs inverse dynamics** (simulators vs
> controllers), the **mass matrix** (symmetric-PD, config-dependent, inertia
> ellipsoid), Coriolis/centrifugal, the **Newton–Euler two-sweep algorithm** (taught
> as the outward-motion / inward-force relay picture, no algebra), **why contact
> makes dynamics hard** (the soft-contact problem MuJoCo/Isaac exist for), and a
> **sim-vs-real model-acquisition** section (Pinocchio/RNEA, system ID as
> least-squares, sensing pitfalls). 8b covers **self-dynamics compensation**.
> Practiced via a **haptic-teleop design exercise** (8a §11) instead of book
> problems: bilateral master/slave control, the `cancel g,C / use M` computed-torque
> law, and **sensorless force estimation** `F_env = (Jᵀ)⁺[τ_meas − (Mθ̈+Cθ̇+g)]` (the
> residual), incl. the friction-as-phantom-force and `(Jᵀ)⁺`-vs-`J⁻¹` gotchas. Also
> banked: the **policy→IK→controller→motor** stack and what you actually command a
> motor (PWM→current→torque), 8a §9–10.

> **Chapter 7 — complete.** Notes 7a (parallel mechanisms, the IK-easy/FK-hard
> **duality**, loop-closure DOF counting, 3×RPR + Stewart–Gough, and a full
> **delta-robot** treatment incl. worked IK §7a.1 with figure), 7b (the
> **constraint Jacobian** `H q̇=0`, `q̇_p=−H_p⁻¹H_a q̇_a`, the Stewart static
> `τ=JᵀF` shortcut, and the **three singularity types** with figure), 7c
> (**applied haptic teleop** capstone). Practiced via a **design exercise instead
> of book problems** — a delta+wrist haptic master for teleoperated surgery —
> exercising `τ=JᵀF` display, `J_a` via `H`, the force/torque decoupling (delta→`f`,
> wrist→`m`, block-antidiagonal Jacobian at grip = wrist center), space↔body
> Jacobian conversion (adjoint of the **end-effector** pose, or build native via
> the frame-free `H` step), and slave→master frame transport. FAQs in 7a/7b
> capture the "if I knew the passive angles", "how is `H` compiled" (two routes +
> dimension bookkeeping), and "real control loop / `J⁻¹` direct vs invert `J`"
> discussions.

> **Chapter 6 — complete.** Notes 6a (analytic IK: structure of solutions, the
> geometric 2R solve, workspace annulus, redundancy, **spherical-wrist
> decoupling**) and 6b (numerical IK: **Newton–Raphson** on the body Jacobian,
> the **pseudo-inverse**/SVD from 5b earning its keep, **damped least squares**
> near singularities). **Exercise 6.10 worked by hand.** This is the
> "policy → EE pose → IK → joint targets" stack from the north star.

> **Chapter 5 — complete.** Theory notes 5a (the Jacobian, space + body) / 5b
> (statics `τ=JᵀF`, singularities, manipulability). Practiced **by hand** through
> book **Exercise 5.3** (4R planar chain): space & body screw axes, `M`, the body
> Jacobian, statics for two tip wrenches (incl. the `p×f` wrench-moment subtlety),
> and a full singularity analysis (singular ⟺ joints collinear). Key intuitions
> banked: *columns of `J` are joint screws*; *space vs body = world's view vs
> gripper's view of the same screws*; *avoid near-singular poses, gauge by the
> eigenvalue ratio of `A=JJᵀ`* (SVD parked → revisit as the pseudo-inverse in Ch. 6).

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
- `mecanum_base.xml` + `mecanum_kinematics.py` — **Ch 13** 4-mecanum omnidirectional
  base driven kinematically by our own `H`/`F` maps: verifies `F=H†` matches the
  book & `F·H=I₃`, integrates the body twist with the screw exponential, and shows
  **odometry drift** (ground-truth vs estimate ghost) under wheel slip →
  `mecanum_demo.gif` + `notes/modern_robotics/figures/13c_mecanum_drift.png`.

## Parked topics (revisit later)

- **Quaternions & other rotation representations** (L&P Appendix B: Euler
  angles, roll–pitch–yaw, unit quaternions, Cayley–Rodrigues). The main text
  sticks to rotation matrices + exponential coordinates, so this isn't needed
  to progress. But simulators/hardware lean on quaternions heavily (MuJoCo,
  Isaac, ROS store orientation as `q ∈ S³` — no gimbal lock, stable to
  integrate). Slot in a short standalone note (`notes/modern_robotics/0B_quaternions.md`) right
  when we start touching **MuJoCo**, where it becomes practically necessary.
  Key hook: `q = (cos(θ/2), ω̂ sin(θ/2))` — spends a 4th number to kill the
  `1/sinθ` singularity in the SO(3) matrix log.
