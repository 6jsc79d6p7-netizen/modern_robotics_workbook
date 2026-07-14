# Underactuated Robotics — course overview & plan

The next book/course after *Modern Robotics*: **MIT 6.832 "Underactuated
Robotics"** by **Russ Tedrake**. Free online text + lectures:
<https://underactuated.mit.edu> (uses **Drake** / `pydrake` for examples).

This note is the launchpad; per-topic notes will live alongside it in this folder
(`notes/underactuated/`), each intuition-first with a `figures/` subfolder, same
workflow as the MR notes.

---

## What "underactuated" means — and why it's the natural next step

*Modern Robotics* almost always treated the robot as something you **command
directly**: a fully-actuated, kinematic / quasi-static object. Want the arm at a
pose? Solve IK, send joint targets, a controller cancels the dynamics (computed
torque, Ch 11) and you get there. The dynamics were an *obstacle to cancel*.

An **underactuated** system has **fewer independent actuators than configuration
DOF** — or, more precisely, at some states it **cannot command an arbitrary
acceleration**. You physically *can't* cancel the dynamics; you have to **exploit**
them. A passive-ankle walker, a quadrotor, an acrobot, a robot throwing or catching,
a legged robot mid-stride — none can be driven pose-to-pose. You must plan and
control *through* the natural dynamics.

This is exactly the boundary we hit at the end of MR:
- **Ch 13 quadruped tangent** — a legged base is force/contact-constrained and
  underactuated; the clean `J_e†` gives way to dynamics-aware, constrained control.
- **Ch 11 controls** — computed torque *cancels* dynamics; underactuated systems
  are where that stops being possible, and optimal control takes over.

## Why it's on the north-star path (even with legged deferred)

Humanoids / legged locomotion are parked at roadmap **M4** — but the **machinery**
this course teaches is *not* deferred, because it's the substrate under learned
control:

- **Optimal control** (LQR, dynamic programming / value iteration) — the classical
  spine of RL; understanding value functions and Bellman here demystifies the RL
  used for manipulation policies.
- **Trajectory optimization** (direct collocation, shooting, iLQR/DDP) — the same
  family as the model-based planners (cuRobo-style) from the Ch 9/10 SOTA note, and
  what generates demos / warm-starts for learning.
- **Policy search / RL** — taught grounded in dynamics rather than as a black box.
- **Lyapunov / stability** — how to *certify* a controller, useful anywhere.

So even for a statically-stable wheeled manipulator, this is the control- and
learning-theory foundation the VLA/diffusion/RL stack rests on.

## Rough module map (to refine against the online text ToC)

**Dynamics & the systems:**
1. Fully-actuated vs underactuated — the core distinction.
2. The simple pendulum — the atom (energy shaping, phase portraits).
3. Acrobot, cart-pole, quadrotor — canonical underactuated systems + partial
   feedback linearization.
4. Simple walking/running models (passive dynamic walking, limit cycles).

**Optimal control & motion planning:**
5. Dynamic programming & value iteration (→ the RL bridge).
6. Lyapunov analysis (stability certificates, regions of attraction).
7. Trajectory optimization (direct collocation, shooting, iLQR).
8. Linear optimal control — **LQR** (and time-varying LQR along trajectories).
9. Feedback motion planning — LQR-trees / funnels.
10. Motion planning as search + optimization.

**Learning:**
11. Reinforcement learning (policy gradient, actor-critic) grounded in dynamics.

*(Numbering is approximate — align with <https://underactuated.mit.edu> when we
start module 1.)*

## Tooling

- The course uses **Drake** (`pip install drake` / `pydrake`) — worth installing in
  `.venv` for the built-in examples and its excellent trajectory-optimization and
  systems framework.
- We already have **MuJoCo** in `.venv` and a working render/sim setup — many
  toy systems (pendulum, cart-pole, acrobot) are trivial to build there too, so we
  can use whichever fits a given topic (Drake for its optimal-control tooling,
  MuJoCo for quick sims that match our existing pick-place stack).

## How we'll work it

Same as MR: **theory note → discussion → practice** (by-hand for the math that
builds intuition — LQR, value iteration, a collocation setup — and code/sim where a
system needs to be *seen* moving). First up when we start: **module 1 (fully vs
underactuated) + the simple pendulum**, where the whole philosophy — "shape energy,
don't fight it" — first bites.
