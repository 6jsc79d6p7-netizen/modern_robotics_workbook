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
| 5 | Velocity Kinematics & Statics | | | ✅ done |
| 5a | ↳ The Jacobian (space + body) | [05a](notes/05a_jacobian_velocity_kinematics.md) | Ex 5.3 (by hand) | ✅ done |
| 5b | ↳ Statics, singularities, manipulability | [05b](notes/05b_statics_singularities_manipulability.md) | Ex 5.3 (by hand) | ✅ done |
| 6 | Inverse Kinematics | | | ✅ done |
| 6a | ↳ The IK problem & analytic solutions | [06a](notes/06a_inverse_kinematics_analytic.md) | Ex 6.10 (by hand) | ✅ done |
| 6b | ↳ Numerical IK (Newton–Raphson, pseudoinverse) | [06b](notes/06b_inverse_kinematics_numerical.md) | Ex 6.10 (by hand) | ✅ done |
| 7 | Kinematics of Closed Chains | | | ✅ done |
| 7a | ↳ Parallel mechanisms, IK/FK, delta robot | [07a](notes/07a_closed_chains_kinematics.md) | — (design exercise) | ✅ done |
| 7b | ↳ Differential kinematics & singularities | [07b](notes/07b_differential_kinematics_singularities.md) | — (design exercise) | ✅ done |
| 7c | ↳ Applied: haptic teleop (delta + wrist) | [07c](notes/07c_haptic_teleop_delta.md) | — (design exercise) | ✅ done |
| 8 | Dynamics of Open Chains | | | ✅ done |
| 8a | ↳ Manipulator eqn, fwd/inv dynamics, Newton–Euler, contact, sim-vs-real model | [08a](notes/08a_open_chain_dynamics.md) | — (haptic teleop exercise) | ✅ done |
| 8b | ↳ Applied: self-dynamics compensation (haptics) | [08b](notes/08b_self_dynamics_compensation.md) | — (design exercise) | ✅ done |
| 9 | Trajectory Generation | [09](notes/09_trajectory_generation.md) | toys (planned) | 🟨 theory done |
| 9–10 | ↳ Learned SOTA (combined) | [09_10s](notes/09_10_learned_sota.md) | toys (planned) | 🟨 theory done |
| 10 | Motion Planning | [10](notes/10_motion_planning.md) | toys (planned) | 🟨 theory done |
| 11 | Robot Control | | | ⬜ not started |
| 12 | Grasping & Manipulation | | | ⬜ not started |
| 13 | Wheeled Mobile Robots | | | ⬜ not started |

Status legend: ⬜ not started · 🟨 in progress · ✅ done

## Up next

**Ch. 9 & 10 — theory notes done; toy examples next.** Per CLAUDE.md these are
*special handling* (gist + SOTA + toy). We taught **9 and 10's SOTA together**
(user's call — the learned methods dissolve the Ch9/Ch10 boundary). Three notes:
- [09](notes/09_trajectory_generation.md) — classical trajectory-gen gist (path ×
  time scaling; joint/task/SE(3) straight lines; cubic/quintic/trapezoid/S-curve
  jerk ladder; via points; the `(s,ṡ)` phase-plane time-optimal picture).
- [10](notes/10_motion_planning.md) — classical motion-planning gist (the
  piano-mover's problem; **C-space obstacles**; A\*; grid/sampling(RRT,RRT\*,PRM)/
  potential-field/optimization taxonomy; completeness ladder).
- [09_10s](notes/09_10_learned_sota.md) — **keystone SOTA note**: the
  classical→learned spectrum (classical+TOPP → cuRobo → MπNets → **ACT/Diffusion
  Policy** → **VLA**), action chunking, multimodality, the **VLN/semantic-nav**
  branch, and what MR substrate survives underneath (this is the Phase-0→robot-
  learning bridge).

**PIVOT — now building the practical project, not toy examples.** Ch 9–10 theory
(+ rich FAQ) is done; **Ch 11/12/13 are deferred.** Next is a real
**MuJoCo pick-and-place build** toward the north star (roadmap M1→M2→M3): scripted
privileged-state expert **+ human teleop** demos → conditioned **flow-matching**
policy (Rung 4) → **π0** (Rung 5). Decisions: **Franka Panda**, **cloud GPU** for
the CUDA parts (cuRobo/AnyGrasp/training/π0 — none run on the Mac), **phased**
(env + data on the Mac first), and a **walled-bin planning challenge** in the place
phase. First concrete piece: the **phone 6-DoF teleop** data interface —
[`proj_phone_teleop.md`](notes/proj_phone_teleop.md) (also the best hands-on Ch-3
SE(3)/transform-tree exercise). NB: rebuilding `.venv` dropped MuJoCo — reinstall
`mujoco` + `mujoco_menagerie` when Phase 0 starts.

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
