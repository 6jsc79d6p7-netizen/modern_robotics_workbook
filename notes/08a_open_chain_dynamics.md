# 8a — Dynamics of Open Chains (the manipulator equation, conceptually)

> Chapter 8. Kinematics (Ch. 3–7) asked *where* the robot is for a given set of
> joint angles. **Dynamics** asks *how forces and motion relate*: push the joints
> with torque `τ`, how does the arm accelerate — and inversely, what torque do I
> need to make it move a desired way? This is the bridge from "geometry of robots"
> to "**controlling** robots," and it's literally what MuJoCo / Isaac compute every
> timestep.
>
> **Tier-2 / conceptual pass** (per `CLAUDE.md`): we **skip the Lagrangian
> derivations** entirely. The goal is to know *what each term means*, *why the
> equation is shaped the way it is*, and *why contact is the hard part simulators
> exist to solve*. The applied payoff — cancelling a robot's own dynamics — already
> has its own note: [[08b_self_dynamics_compensation]] (the haptic teaser).

---

## 1. The big picture — two questions, one equation

Dynamics is about the relationship between **force/torque** and **motion**
(acceleration). For an open-chain robot it collapses to a single vector equation,
the **manipulator equation** (stated fully in [[08b_self_dynamics_compensation]] §2):

$$ \tau \;=\; \underbrace{M(\theta)\,\ddot\theta}_{\text{inertia}} \;+\; \underbrace{C(\theta,\dot\theta)\,\dot\theta}_{\text{Coriolis/centrifugal}} \;+\; \underbrace{g(\theta)}_{\text{gravity}} \;+\; \underbrace{f(\dot\theta)}_{\text{friction}} $$

Everything in Chapter 8 is *one of two ways to read this equation*:

- **Inverse dynamics** — *given* a desired motion `(θ, θ̇, θ̈)`, solve for the
  torque `τ` that produces it. The equation is already in this form: plug in the
  right-hand side, read off `τ`. **This is what a controller does** — "I want this
  acceleration, what torque do I command?" It's the basis of computed-torque
  control (Ch. 11) and of the gravity/inertia *compensation* in 08b.

- **Forward dynamics** — *given* the applied torque `τ` (and current `θ, θ̇`),
  solve for the resulting acceleration `θ̈`. Rearrange:

  $$ \ddot\theta \;=\; M(\theta)^{-1}\big(\tau - C(\theta,\dot\theta)\dot\theta - g(\theta) - f(\dot\theta)\big). $$

  **This is what a simulator does** — apply torques and contact forces, get
  accelerations, integrate to step the world forward. MuJoCo / Isaac *are* forward-
  dynamics engines.

Same equation, solved for different unknowns. Inverse dynamics is the *easy*
direction (just evaluate the right side). Forward dynamics needs the matrix
**inverse** `M⁻¹`, which is the expensive part and why efficient algorithms matter.

> **Why care, for the north star:** an RL/IL policy never sees this equation
> directly — but it sits *on top* of a controller that does. The policy outputs a
> target (EE pose, joint delta), inverse dynamics + gravity comp turn it into
> motor torque, and forward dynamics (the sim) turns torque into the next state the
> policy observes. Knowing the four terms is knowing what the substrate underneath
> a learned policy is actually doing.

---

## 2. The four terms, by their *shape*

The whole equation is **nonlinear** in the state — that's the headline. Let's see
*where* the nonlinearity lives.

### `g(θ)` — gravity (depends on pose only)
The torque each motor must supply just to **hold the arm up** at configuration `θ`.
Hold a heavy arm straight out → big wrist/shoulder torque; tuck it in → less.
Depends on `θ` only (not how fast you move). Most important term to cancel in
practice (08b §3). Vector, length = #joints.

### `M(θ) θ̈` — inertia (the mass matrix)
`M(θ)` is the **mass matrix**: the arm's effective inertia *as felt at the joints*.
`M θ̈` is the torque needed to **accelerate** the joints — "it resists being sped
up." Three things to internalize about `M`:

1. **It's a matrix, not a scalar**, because the joints are **coupled**:
   accelerating joint 2 reacts back on joint 1 (the off-diagonal entries `M_{ij}`).
   A single rigid body has a scalar/inertia-tensor; a chain of bodies has a *matrix*
   tying all the joints together.
2. **It depends on configuration `θ`.** A stretched-out arm has far more rotational
   inertia about the shoulder than a folded one — same physical links, different
   leverage. (Hold a broom by the end vs the middle.) So `M` is recomputed every
   pose.
3. **It's symmetric positive-definite** (see LA aside). Geometrically that's an
   **inertia ellipsoid** — the dynamics cousin of the manipulability ellipsoid from
   5b. Long axis = direction the arm is "heavy"/hard to accelerate; short axis =
   "light."

### `C(θ,θ̇) θ̇` — Coriolis & centrifugal (the velocity term)
These are the **fictitious forces** that show up *because the mass matrix changes as
the arm moves*. Two flavors, both **quadratic in velocity**:

- **Centrifugal**: ∝ `θ̇ᵢ²` (one joint moving). The outward "fling" — spin the
  shoulder and the outstretched forearm wants to fly outward.
- **Coriolis**: ∝ `θ̇ᵢ θ̇ⱼ` (two joints moving together). The sideways coupling
  force when two joints move at once.

Key intuition: **zero at rest, negligible when slow, dominant when fast** (quadratic
growth). For a slow haptic device you can ignore it; for a fast arm or a legged
robot it's essential. (The `C` matrix isn't unique — it's built from derivatives of
`M` called *Christoffel symbols*; you'll never hand-compute these past a 2R, and we
won't. Just know `C θ̇` *is* the velocity-induced torque.)

### `f(θ̇)` — friction (the un-physics-y term)
Viscous (∝ speed) + Coulomb (constant drag while moving) at each joint. Real,
significant, and the **hardest to model** (nonlinear, hysteretic, temperature-
dependent). MR's idealized dynamics often drop it; real controllers fight it with a
mix of model + learned disturbance observer. Not part of the "clean" rigid-body
dynamics — bolted on.

> **The one-line summary of the structure:** gravity = f(pose), inertia =
> M(pose)·accel, Coriolis = quadratic in velocity, friction = messy. The only term
> that touches `θ̈` is `M(θ)`, which is why forward dynamics = "invert `M`."

---

## 3. Linear algebra you need here

Two ideas, both geometric.

### (a) Quadratic forms & positive-definiteness — the mass matrix is "energy"
A **quadratic form** is `xᵀ A x` — feed a vector in twice, get a scalar out. For the
mass matrix, the quadratic form is **kinetic energy**:

$$ \text{KE} \;=\; \tfrac12\,\dot\theta^{T} M(\theta)\,\dot\theta. $$

This is the matrix generalization of the scalar `½mv²`. `M` plays the role of "mass,"
`θ̇` the role of "velocity," but because motion is multi-dimensional, "mass" is a
matrix.

**Positive-definite** means `θ̇ᵀ M θ̇ > 0` for every nonzero `θ̇` — i.e. *any*
motion costs positive kinetic energy, never zero or negative. Physically obvious
(a moving arm has energy); mathematically it guarantees `M` is **always
invertible**, so forward dynamics `θ̈ = M⁻¹(…)` always has a unique solution. (No
dynamic singularities the way kinematics had — a real arm always has *some* inertia
in every direction.) **Symmetric** (`M = Mᵀ`) because the joint-coupling is mutual:
joint 1's reaction on 2 equals 2's on 1.

Geometrically a positive-definite symmetric matrix *is* an **ellipsoid** (`θ̇ᵀMθ̇ =
const` traces one). That's the inertia ellipsoid from §2 — same math object as the
manipulability ellipsoid (5b), different physical meaning.

### (b) Change of "frame" for inertia — why `M` is configuration-dependent
Each link has a fixed **spatial inertia** in *its own* body frame (mass + a 3×3
inertia tensor, packed into a 6×6 matrix `𝒢ᵢ` — the `<inertial>` block of a URDF).
The mass matrix is, conceptually,

$$ M(\theta) \;=\; \sum_i J_i(\theta)^{T}\, \mathcal G_i\, J_i(\theta), $$

where `Jᵢ` is the Jacobian mapping joint velocities to link `i`'s twist. You've seen
this `Jᵀ(\cdot)J` sandwich before — it's a **change of coordinates** for a quadratic
form (like `5b`'s `JJᵀ`). The link inertias `𝒢ᵢ` are *constant*; all the `θ`-
dependence enters through the **Jacobians**, which change as the arm reconfigures.
That's the precise reason `M` depends on pose: the *links* don't change, but *how
joint motion projects onto each link's motion* does. (You don't need to evaluate
this sum by hand — but it explains the shape.)

---

## 4. The two ways to *get* the equation (gist only — we derive neither)

Two classical roads to the same `M, C, g`:

- **Lagrangian (energy method).** Write kinetic minus potential energy
  `ℒ = KE − PE` as a function of `(θ, θ̇)`, turn the crank of the Euler–Lagrange
  equation, and `M, C, g` fall out. **Elegant, no internal forces to track, brutal
  to do by hand past 2 joints.** *We skip this entirely* (project rule). Worth knowing
  it exists and that `KE = ½θ̇ᵀMθ̇`, `PE = potential → g(θ) = ∂PE/∂θ`.

- **Newton–Euler (recursive force/moment).** Walk **outward** along the chain
  propagating velocities/accelerations (base → tip), then **inward** propagating
  forces/moments (tip → base), applying `F = ma` and its rotational twin to each
  link. It's bookkeeping, not calculus. Crucially it's **O(n)** in the number of
  joints and doesn't require forming `M` explicitly — so it's the **algorithm real
  simulators and controllers actually run** for inverse dynamics. (MuJoCo's
  `mj_inverse`, the "RNEA" you'll meet in robotics libraries.)

Takeaway: **Lagrangian = good for understanding the *form*; Newton–Euler = good for
*computing* fast.** You'll never need the algebra of either at our level — the
simulator does it. But Newton–Euler is worth understanding *as a picture*, because
it's literally the algorithm a real robot runs — so it gets its own section next.

---

## 5. The Newton–Euler algorithm — the two-sweep picture (no algebra)

You don't need to *derive* this, and we won't write a single index-juggling
equation. But the **shape** of the algorithm is genuinely intuitive, and since it's
what both MuJoCo *and* a real robot's control PC actually run, it's worth seeing.

### The setup: a robot is a stack of rigid bodies, each obeying two laws
Forget the chain for a second. **One** rigid body floating in space obeys exactly
two laws — Newton's and his rotational twin, Euler's:

- **Newton (translation):** `f = m·a`. Net force = mass × acceleration of the center
  of mass. The law you already know.
- **Euler (rotation):** `m = 𝓘·α + ω×(𝓘ω)`. Net moment = rotational inertia ×
  angular acceleration, **plus** a "gyroscopic" term `ω×(𝓘ω)`. That second term is
  the weird one: a *spinning* body resists being reoriented and throws a sideways
  moment even at constant spin (why a spinning top precesses, why a bike wheel
  fights you). **It's velocity-dependent and quadratic in `ω`** — at the joint level,
  summed over all links, *this is exactly what becomes the Coriolis/centrifugal `C`
  term* from §2. Same physics, different bookkeeping level.

A robot is just `n` of these bodies, each bolted to the next through a joint. The
*only* complication versus one free body: **each link rides on top of the moving
link before it, and each link carries the reaction of everything after it.** Newton–
Euler handles those two couplings with **two sweeps along the chain.**

### Sweep 1 — OUTWARD (base → tip): "how is each link moving?"
Motion flows *outward*. The base is known (it's bolted down, or you know how it
moves). Then:

> link 1's motion = base's motion **+** what joint 1 adds
> link 2's motion = link 1's motion **+** what joint 2 adds
> … and so on to the tip.

You're given the desired `(θ, θ̇, θ̇̇)` (joint angles, rates, accelerations), so each
joint's contribution is known, and you **accumulate** velocity and acceleration
link-by-link from base to tip. Think of a **relay passing a baton outward**, each
runner adding their own joint's spin to whatever they received. By the end of the
sweep you know the **velocity and acceleration of every link** (including its center
of mass).

*Why outward?* Because a child's motion *depends on* its parent — you cannot know how
the forearm is accelerating until you know how the upper arm is. Kinematics flows
from the base out.

### Sweep 2 — INWARD (tip → base): "what force/torque does each joint need?"
Now you know how every link is accelerating, so Newton+Euler tell you the **net
force and moment** each link requires (to accelerate that way *and* hold up against
gravity). But each link is pushed by **two** neighbors: the joint *outboard* of it
(toward the tip) and the joint *inboard* (toward the base). So you solve from the
**tip inward**, where there's nothing further out to worry about:

> tip link: only gravity + any external load + its own inertia act on it
>   → solve for the force/moment its joint must supply.
> next link in: gravity + inertia + **the reaction pushed back by the link beyond it**
>   → solve for *its* joint.
> … accumulate inward to the base.

Think of **people standing on each other's shoulders**: to find the force in the
*bottom* person's legs you start at the *top* and add weights downward — the base
bears everyone. Same here: each joint's load = everything outboard of it, summed up.
The shoulder torque has to account for the whole arm; the wrist torque only for the
hand. Forces **accumulate inward**.

### The payoff line
For each joint, take the moment Sweep 2 computed and read off **its component along
the joint axis** — *that scalar is the motor torque `τ` for that joint.* Done. You
fed in desired motion `(θ, θ̇, θ̇̇)` and got out required torque `τ` — that's
**inverse dynamics** (§1), computed without ever forming or inverting the big matrix
`M`. Two passes over `n` joints ⇒ **O(n)**, which is why it's fast enough to run at
1 kHz on a real robot's controller (and why it's "RNEA" — *Recursive* Newton–Euler).

### How you still get `M`, `C`, `g` out of it (the clever trick)
The two-sweep algorithm hands you the *combined* torque, not the separate terms. But
you can extract them by feeding it **special inputs** — each run isolates one term:

- Run it with `θ̇ = 0, θ̇̇ = 0, gravity on` → the only thing left is **`g(θ)`**.
- Run it with `gravity off, θ̇ = 0, θ̇̇ = eᵢ` (accelerate only joint *i*) → you get
  **column *i* of the mass matrix `M`**. Do it for each joint → the whole `M`.
- Run it with `θ̇ ≠ 0, θ̇̇ = 0, gravity off` → the leftover is **`C(θ,θ̇)θ̇`**.

So the same recursion both *is* inverse dynamics directly and *builds* the named
terms when you want them. That's the entire engine under `pinocchio.rnea`,
`mj_inverse`, and Franka's `gravity()` / `mass()` API calls.

**The whole thing in one breath:** *outward to find how every link moves, inward to
find what every joint must push, project onto the joint axis to get `τ`.* No
calculus, no matrix inverse — just two relay races along the arm.

---

## 6. A small worked example — the single link (1-DOF), then a hint of 2R

The smallest non-trivial case: **one revolute joint, one uniform rod** of mass `m`,
length `L`, joint at one end, swinging in a vertical plane (a motorized pendulum).
Angle `θ` measured from straight-down.

The manipulator equation collapses to a scalar:

$$ \tau \;=\; \underbrace{\left(\tfrac{1}{3} m L^2\right)}_{M}\ddot\theta \;+\; \underbrace{\tfrac12 m g L \sin\theta}_{g(\theta)}. $$

Read off the structure:
- **`M = (1/3) m L²`** — the rod's moment of inertia about the end (a fixed scalar
  here; no configuration dependence because one link can't reconfigure its own
  leverage). With `m = 1 kg`, `L = 1 m`: `M = 0.333 kg·m²`.
- **`g(θ) = ½ m g L sinθ`** — gravity torque, maximal at `θ = 90°` (arm horizontal,
  worst leverage), **zero** at `θ = 0°` and `180°` (hanging straight down/up). With
  `m=1, L=1, g=9.81`: at horizontal, `g = 0.5·1·9.81·1 = 4.9 N·m` to hold it up; at
  straight down, `0`. *That number is exactly what gravity-compensation feeds
  forward (08b).*
- **No `C` term** — a single joint moving alone produces only centrifugal force
  *along the link*, which the joint can't feel (it's radial). Coriolis needs ≥2
  moving joints. **This is why you need a 2R to see `C` at all.**

**The 2R hint (no derivation).** Add a second link and the scalars become
matrices/vectors. The mass matrix gains **off-diagonal coupling** `M₁₂(θ₂)` that
depends on the elbow angle `θ₂` (folded vs straight changes how shoulder and elbow
react on each other), and a genuine **Coriolis term** appears: swinging both joints
throws a velocity-dependent torque that neither joint commands. That coupling — "I
moved the elbow and the shoulder felt a kick" — *is* the thing that makes
multi-joint dynamics hard and linear control insufficient. (The full 2R `M, C, g`
is MR Example 8.7; we won't grind it, but we can read it together in the exercises
if you want.)

---

## 7. Why **contact** makes dynamics hard (the part simulators exist for)

Everything above is **smooth** rigid-body dynamics — nice continuous functions of
`(θ, θ̇, θ̈)`. The moment the robot **touches something**, that breaks:

- **Contact is non-smooth / discontinuous.** A finger approaching a cup feels *zero*
  force, then *suddenly* a normal force the instant it touches — a step change, not a
  smooth curve. Velocities can jump (impact). Standard ODE integration hates this.
- **Unilateral + complementarity.** Contact can **push but not pull** (normal force
  ≥ 0), and force is nonzero *only* when touching (gap = 0). "Either the gap is zero
  or the force is zero, never both nonzero" is a **complementarity condition** —
  this turns each step into a small constrained problem (an LCP, linear
  complementarity problem), not a plain matrix solve.
- **Friction at contacts** (Coulomb cones) adds more non-smooth, set-valued
  constraints — stick vs slip.
- **Many simultaneous contacts** (a grasped object, a foot, clutter) couple together
  and must be solved *jointly* and consistently.

This is *exactly* the problem MuJoCo and Isaac are built to solve: they replace the
hard rigid complementarity with a **soft, regularized contact model** (a slightly
springy/relaxed contact that's smooth enough to integrate fast and differentiate).
That trade — physical exactness for smooth, fast, stable, *differentiable* contact —
is **why these simulators exist** and why they're the substrate for RL/IL of
**contact-rich manipulation** (the north-star pick-and-place). When people say
"contact is the hard part of robotics," *this* is the math they mean.

---

## 8. Where the model comes from: sim vs. real robot

`M, C, g` are fixed functions of each link's **mass, center of mass, and inertia
tensor**. The question is *who supplies those numbers and who evaluates the terms.*
Crucially: **MuJoCo isn't doing anything magic — it runs Newton–Euler (§5). A real
robot runs the exact same algorithm on its own control PC.** The only difference is
the *parameter source* and the *sensing*.

### In simulation (given for free)
- The link parameters come from the **`<inertial>` blocks of the MJCF/URDF**.
- **MuJoCo exposes the terms directly:** `qfrc_bias` = `C(θ,θ̇)θ̇ + g(θ)` (the whole
  velocity+gravity bias), the mass matrix via `mj_fullM`, inverse dynamics via
  `mj_inverse`, forward dynamics via `mj_forward`/`mj_step`. "Gravity-comp in sim" is
  literally *read `qfrc_bias`, feed it back* (08b).

### On a real robot (you have to acquire the model) — two paths

**Path 1 — compute from a model.** Run the same Newton–Euler recursion (§5) yourself,
every tick (~1 kHz), with a dynamics library: **Pinocchio** (the standard — fast,
gives derivatives), RBDL, KDL, Drake. You feed it the URDF; it returns `M, C, g`
exactly like MuJoCo. Where the *parameters* come from, best-effort-first:
1. **CAD** — the manufacturer's model gives per-link mass/COM/inertia, often shipped
   in the URDF. Decent first cut, never exact (cables, motors, real mass spread).
2. **Manufacturer-calibrated** — high-end arms ship an *identified* model. Franka's
   `libfranka` and KUKA iiwa expose `mass()`, `coriolis()`, `gravity()` **directly
   through their API** — on those robots it's as easy as MuJoCo.

**Path 2 — system identification (measure them).** For accuracy you *identify* the
parameters. The enabling fact: **the dynamics are linear in the inertial parameters**,

$$ \tau = Y(\theta,\dot\theta,\ddot\theta)\,\pi, $$

where `π` stacks all the unknown masses/first-moments/inertia entries and `Y` (the
"regressor") is built from measured motion. So: drive the robot through rich
**exciting trajectories**, record `(θ, θ̇, θ̈)` and `τ`, and **least-squares solve**
`π = Y⁺ τ`. That `Y⁺` is the **pseudo-inverse from 6b** — system ID *is* a big
least-squares fit (your LA tools earning their keep on hardware).

### The sensing problem (what sim doesn't have)
To run either path you need state the sim just *knows*:
- **`θ`** — encoders (easy, precise). **`θ̇`** — differentiate + filter.
- **`θ̈`** — differentiate *twice* → **very noisy**; usually avoided/heavily filtered.
- **`τ`** — either **current sensing** (`τ = Kt·I`, cheap, most robots) or dedicated
  **joint torque sensors** (Franka, iiwa — accurate, expensive; let you close a
  torque loop and lean *less* on a perfect model).

### The honest practical picture
The four terms are **not equally obtainable** on hardware:

| term | real-robot difficulty |
|---|---|
| `g(θ)` gravity | **easy, dominant, well-identified** — gravity comp works great everywhere |
| `M(θ)` inertia | moderate — needs decent CAD or ID |
| `C(θ,θ̇)` Coriolis | moderate, and needs noisy `θ̇` |
| `f(θ̇)` friction | **the nightmare** — nonlinear, hysteretic, drifts with temp/wear |

So most robots *without* torque sensing run **gravity comp + PD** (cheap, ~90% of the
benefit, 08b §3), not full computed torque. Full computed-torque/impedance is for
robots that either ship a calibrated model **and** torque sensors (Franka, iiwa —
then it's sim-easy) or where someone did careful system ID. And where the model is
too crude — especially **friction and contact** — that's the seam where **learned /
residual dynamics** take over: the model handles the well-behaved 95% (`g`, `M`),
learning patches the unmodeled 5%. Very north-star.

---

## 9. Gotchas / intuition checks

- **`M` depends on `θ`, never on `θ̇`.** Inertia is about configuration (leverage),
  not speed. Speed lives in the `C` term.
- **Gravity is velocity-independent; Coriolis is velocity-*quadratic*; inertia is
  acceleration-linear.** Each term keys off a *different* derivative of motion — that
  separation is the whole point of writing it as four terms.
- **The equation is nonlinear** even though it *looks* linear in `θ̈`. `M`, `C`, `g`
  all depend nonlinearly on `θ` (and `C` on `θ̇`). That nonlinearity is why naive
  per-joint PID control struggles and why "cancel the model, then control the
  leftover" (computed torque, Ch. 11) is the move.
- **Inverse dynamics is cheap (evaluate RHS); forward dynamics needs `M⁻¹`** (the
  costly bit). Simulators do forward; controllers do inverse.
- **No dynamic singularities of the kinematic kind** — `M` is always positive-
  definite ⇒ always invertible. (Don't confuse with *kinematic* Jacobian
  singularities from 5b/7b — different object, different failure mode.)
- **The hard part isn't the chain, it's the contact.** The smooth manipulator
  equation is "solved"; contact is the open, simulator-defining problem.

---

## 10. From policy to motor torque — where this chapter becomes real control (discussion)

A discussion thread worth banking: how does the manipulator equation actually reach
a motor, and why is feedforwarding it the whole game? Three layers.

### (a) The manipulation stack — policy → torque
```
policy ──► EE action ──► IK ──► joint targets ──► low-level controller ──► τ ──► robot/sim
 (~10 Hz)   (pose Δ or    (θ_des)                  (PD / computed-torque,    (motor   (forward
            absolute)                               ~500–1000 Hz)            torque)  dynamics)
```
- The **action space varies**: EE pose delta (most VLAs / diffusion policy / ACT),
  absolute EE pose, joint targets (then *skip IK*), or raw joint torque (RL
  locomotion; skips everything).
- **Two clocks:** the policy is slow (~10 Hz), the controller is a fast inner loop
  (~1 kHz) that tracks the target many times per policy tick.
- The **controller box is where this chapter lives.** The rightmost box (`τ → θ̈ →
  integrate`) *is* forward dynamics = MuJoCo/Isaac.

### (b) What you actually command a motor
A motor is fundamentally a **current → torque** device: `τ = Kt·I`, and you meter the
current with **PWM voltage** (`I = (V − back-EMF)/R`, back-EMF ∝ ω). The causal chain
is fixed: **PWM → voltage → current → torque → (dynamics) → accel → ω → θ.** Nothing
commands `θ` or `ω` directly in the physics — you get them by controlling torque over
time through **nested feedback loops** inside the driver:

| You command | Driver mode | Internal loop | Rate |
|---|---|---|---|
| current (≈ torque) | torque | PWM to hit current | ~10–20 kHz |
| ω_des | velocity | velocity PID → current | ~1 kHz |
| θ_des | position | position PID → velocity/current | ~100 Hz–1 kHz |

So "command a position" = the driver's *internal* PD computes `τ = Kp(θ_des−θ) +
Kd(θ̇_des−θ̇)` → current → PWM. **Computed-torque / impedance control needs *torque
mode*** — the only way to inject the inverse-dynamics feedforward below, and the only
way to be compliant for contact (the north-star reason). Hobby servos (position mode)
seal the torque loop away; ODrive / Franka / industrial drives let you pick the mode.

### (c) Computed-torque control — feedforward the model, PD fights only linear error
The payoff of inverse dynamics (§1). Command:

$$ \tau = \underbrace{M(\theta)\ddot\theta_{des} + C(\theta,\dot\theta)\dot\theta + g(\theta)}_{\text{inverse-dynamics feedforward}} + \underbrace{M(\theta)\big(K_p e + K_d \dot e\big)}_{\text{PD on error, wrapped in }M}, \quad e = \theta_{des} - \theta. $$

Plug into the real dynamics `τ = Mθ̈ + Cθ̇ + g`: the `Cθ̇ + g` cancel, the `M`
cancels (invertible, §3), and *all* the nonlinear coupling collapses to a clean,
**decoupled, linear, constant-coefficient** error equation — one spring-mass-damper
per joint:

$$ \boxed{\;\ddot e + K_d\,\dot e + K_p\,e = 0\;} $$

So the PD never sees the nonlinear `M, C, g` — only linearized error, which means
**one gain choice works across the whole workspace** (vs a naive PD whose effective
plant `M(θ)⁻¹` changes with pose, so gains tuned folded go bad stretched).

**The caveat — model accuracy.** The cancellation is only as good as `M, C, g` (from
the URDF). Wrong payload / unmodeled friction ⇒ terms don't fully cancel and the
leftover `M⁻¹·(model error)` becomes a disturbance the PD mops up. Hence the integral
term (PID), **adaptive** control (estimate the wrong params online), why **sim-to-real
is hard** (the sim's perfect model ≠ the real robot), and the seam where **learned
residual policies** live: computed-torque handles the modeled 95%, a small learned
policy corrects the unmodeled 5% (friction, contact, payload). Very north-star.

---

## 11. Worked exercise — bilateral haptic teleop is Chapter 8 in action

Instead of a §8.8 book problem, we practiced Chapter 8 by building the **control
loop of a surgical haptic teleop system** (the master = delta + wrist from
[[07c_haptic_teleop_delta]], the slave = the patient-side robot). Every term in this
note shows up. The architecture is **bilateral teleoperation**: *position flows
forward* (master → slave), *force flows back* (slave → master).

### Master (delta haptic device) — render force, stay transparent
The master must let the surgeon feel *only* the tissue, not the device. So command
its own self-dynamics compensation (08b) **plus** the rendered wrench:

$$ \tau_m = \underbrace{g_m(\theta) + C_m\dot\theta + M_m\ddot\theta}_{\text{transparency (cancel own dynamics)}} + \underbrace{J_m^{T} F_{env}}_{\text{render slave's felt wrench}} $$

In practice you cleanly cancel `g_m` (+ friction); cancelling `M_m, C_m` needs `θ̈`
(noisy, §8) so it's the hard tier — but the **delta's low, near-constant moving
inertia** (07a §7) makes `M_m` small, so gravity-comp + `J_m^{T}F` is already
near-transparent.

### Slave (robot) — track the master's pose via computed torque
The slave receives the master's EE pose, runs **IK** → `θ_des`, and tracks it with
computed-torque control (§9c / §10). The load-bearing subtlety: **cancel `g` and
`C`, but *use* `M`** (you can't cancel real inertia — you feedforward it and wrap the
PD inside it):

$$ \tau_s = M(\theta)\big(\ddot\theta_{des} + K_p e + K_d \dot e\big) + C\dot\theta + g, \qquad e=\theta_{des}-\theta $$

which gives the clean `ë + K_d ė + K_p e = 0` error dynamics.

### Closing the loop — sensorless force estimation (the Ch. 8 payoff)
Where does `F_env` (the wrench the master renders) come from? Either a **wrist F/T
sensor**, or — better — **estimate it from the model**. Add the contact term
`J_s^{T}F_{env}` to the slave's true dynamics and solve for the wrench:

$$ \boxed{\;F_{env} = (J_s^{T})^{+}\big[\,\tau_{meas} - (M\ddot\theta + C\dot\theta + g)\,\big]\;} $$

The bracket is the **residual**: *the part of the measured joint torque the `M,C,g`
model can't explain must be external contact.* The robot **feels through its own
dynamics — no force sensor needed.** (This is also how real arms do collision
detection.)

Three things to get right here, all LA:
- It's **`(Jᵀ)⁺`, not `J⁻¹`** — a wrench maps to joint torques via `τ = JᵀF`
  (statics, 5b/7b), so inverting that relation inverts the *transpose*; and since
  `J_s` is rarely square/nonsingular, it's the **pseudo-inverse** (6b).
- Equivalently `F_env = (J_s^{T})^{+}M(a_{cmd}-\ddot\theta)` — `M` times the
  **commanded-vs-actual acceleration mismatch**: if the arm isn't accelerating as
  told, something is pushing it.
- **Friction is the killer:** unmodeled friction looks *exactly* like an external
  push (both are "torque the model didn't predict"), so stiction shows up as a
  phantom contact force. Plus `θ̈` is noisy (→ **momentum observers** avoid it) and
  `(Jᵀ)⁺` ill-conditions near singularities (7b).

**The takeaway:** one teleop loop exercises the whole chapter — inverse dynamics as
feedforward (master transparency + slave tracking), the `M`-vs-`C,g` distinction,
computed-torque linearization, and the residual force estimator — all riding on the
same `M, C, g` model.

---

## 12. Connections & forward pointers

- **[[08b_self_dynamics_compensation]]** — the *applied* companion: use inverse
  dynamics as feedforward to cancel a robot's own gravity/inertia/friction →
  "transparent" haptics. Read this note for the *why-it's-shaped-this-way*, that one
  for the *what-you-do-with-it*.
- **Mass matrix `M(θ)` ↔ manipulability ellipsoid (5b)** — both configuration-
  dependent symmetric-PD quadratic forms; `M` is the *dynamic* (inertia) version.
- **Ch. 11 Robot Control** — computed-torque control = "feedforward inverse dynamics
  to linearize, then PD on the leftover." This note is its prerequisite. Impedance/
  compliance control (the contact-rich, north-star-relevant part) also lives there.
- **MuJoCo / Isaac** — these *are* forward dynamics + the soft-contact model of §7.
  The bridge from this chapter to building. RL/IL policies ride on top of the
  controller this chapter describes.
