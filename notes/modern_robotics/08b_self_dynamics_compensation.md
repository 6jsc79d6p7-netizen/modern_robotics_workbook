# 8 — How a Robot Cancels Its Own Dynamics (gravity / inertia / friction)

> **Chapter 8 teaser, written early** because the haptic design
> ([[07c_haptic_teleop_delta]]) needs it: a haptic master must let the surgeon
> feel *only* the rendered tissue force — **not the device's own weight, inertia,
> and friction.** Making a robot "transparent" to its own dynamics is the first
> real payoff of Chapter 8. Conceptual only — per the project's plan we **skip the
> Lagrangian derivations** and keep the intuition + the one equation that matters.

---

## 1. The problem, physically

Hold a robot arm's motors at zero torque and let go: it **falls** under gravity,
**resists** when you try to accelerate it (inertia), and **sticks/drags** as you
move it (friction). So if a haptic device just commanded `τ = JᵀF` (the ideal
render), the hand would feel `F` **plus** all of that parasitic junk. The surgeon
would feel the master's arm weight and stiction, not the tissue.

**Self-dynamics compensation** = command *extra* motor torque that exactly cancels
the device's own gravity/inertia/friction, so the only thing left for the hand to
feel is the wrench you *meant* to render. "Make the robot behave as if it were
massless, weightless, and frictionless."

---

## 2. The one equation — the manipulator dynamics

Every open-chain robot's dynamics collapse to one vector equation (this *is*
Chapter 8):

$$ \boxed{\,\tau = M(\theta)\,\ddot\theta \;+\; \underbrace{C(\theta,\dot\theta)\,\dot\theta}_{\text{Coriolis/centrifugal}} \;+\; \underbrace{g(\theta)}_{\text{gravity}} \;+\; \underbrace{f(\dot\theta)}_{\text{friction}} \,} $$

What each term *means* (skip the derivation, keep the picture):

- **`g(θ)` — gravity torque.** The motor torque needed just to *hold the arm up*
  against gravity at configuration `θ`. Depends only on pose. **The single most
  important term to cancel** — it's what makes the arm fall.
- **`M(θ) θ̈` — inertia.** `M(θ)` is the **mass matrix**: the arm's effective
  inertia *seen at the joints*, configuration-dependent (a stretched-out arm has
  more rotational inertia than a folded one). `M θ̈` is the torque to *accelerate*
  the arm. This is the "it resists being sped up" feeling. (`M` is symmetric
  positive-definite — geometrically, an inertia ellipsoid, cousin of the
  manipulability ellipsoid from 5b.)
- **`C(θ,θ̇) θ̇` — Coriolis & centrifugal.** Velocity-dependent torques that appear
  because the inertia *changes as the arm moves* (a spinning, reconfiguring body
  throws sideways forces). Quadratic in `θ̇` — negligible at slow haptic speeds,
  matters when things move fast.
- **`f(θ̇)` — friction.** Viscous (drag ∝ speed) + Coulomb (constant stick) at each
  joint. Physically real, **hardest to model** (nonlinear, hysteretic).

---

## 3. How cancellation works — feedforward the model, motors do the rest

The trick is **inverse dynamics as feedforward**: compute what torque the *parasitic*
terms demand at the current state, and add it to your command. To render an
external wrench `F` *and* make the device transparent:

$$ \tau_{\text{cmd}} \;=\; \underbrace{J^T F}_{\text{render}} \;+\; \underbrace{g(\theta)}_{\text{cancel gravity}} \;+\; \underbrace{M(\theta)\,\ddot\theta + C(\theta,\dot\theta)\,\dot\theta + f(\dot\theta)}_{\text{cancel inertia/Coriolis/friction}}. $$

Plug this into the manipulator equation and the device's own dynamics terms
**cancel** — the hand is left feeling exactly `J^T F`, i.e. the wrench `F`. The
motors are now doing two jobs: *fighting the device's physics* and *delivering the
intended force*.

In practice the tiers, cheapest-first:

1. **Gravity compensation only** (`τ = JᵀF + g(θ)`). Cancels the dominant term —
   the arm **floats**, weightless, holding pose with no effort. This alone gets you
   90% of "transparency" for a slow haptic device. *The most common and most
   important.*
2. **+ Friction feedforward** (`+ f(θ̇)`). Cancels stick/drag so motion feels free.
   Often paired with a disturbance observer since friction is poorly modeled.
3. **+ Full inverse dynamics** (`+ Mθ̈ + Cθ̇`). Needed only when the device moves
   fast enough that inertia/Coriolis are felt — high-performance haptics, or
   **computed-torque control** of a fast arm.

---

## 4. Where the model `(M, C, g, f)` comes from

- **Analytically / from the URDF.** `g(θ)`, `M(θ)`, `C(θ,θ̇)` are fixed functions of
  the link **masses, centers of mass, and inertia tensors** — exactly the
  `<inertial>` data in a URDF/MJCF. MR gives the recursive Newton–Euler algorithm
  to evaluate them; you'll rarely hand-derive past a 2R.
- **From the simulator (the north-star way).** MuJoCo / Isaac *are* dynamics
  engines: they compute these terms internally every step. MuJoCo exposes
  `qfrc_bias` (= `Cθ̇ + g`, the gravity+velocity torque) and the mass matrix via
  `mj_fullM` / `mj_inverse`. So "cancel gravity" in sim is literally *read
  `qfrc_bias` and feed it back*. This is the bridge to RL/IL: policies often output
  on top of a gravity-compensated, inverse-dynamics base controller.
- **Friction** is the weak link — model what you can, learn/observe the rest.

---

## 5. Why deltas make especially good haptic masters (tie-back)

A parallel **delta** keeps its motors on the **fixed base** → tiny moving mass →
small, nearly **configuration-independent inertia** `M`. That means: (a) less
inertia to cancel, and (b) what's left is easy to model. Combined with gravity
compensation, the delta gets very close to *ideal transparency* — the surgeon
feels the tissue, not the machine. (The same low-moving-mass property that makes
deltas fast in pick-and-place, 7a §7, makes them excellent haptic devices.)

---

## 6. Connections & forward pointers

- **[[08a_open_chain_dynamics]]** — the core Chapter 8 theory note this teaser
  anticipated: the *why-it's-shaped-this-way* behind `M, C, g, f`, forward vs
  inverse dynamics, Newton–Euler, and why contact is hard. Read that for the
  structure; this note is the applied payoff (compensation).
- This is the **`τ=JᵀF` of [[07c_haptic_teleop_delta]] §1 made honest** — the real
  command is `JᵀF` *plus* the compensation terms here.
- The mass matrix `M(θ)` is the dynamics cousin of the kinematic **manipulability
  ellipsoid** (5b) — both are configuration-dependent quadratic forms describing
  "how the arm responds along each direction."
- **Full Chapter 8 (later, conceptual):** the manipulator equation in detail,
  Newton–Euler vs Lagrangian, the mass matrix's structure, and why **contact**
  makes dynamics hard (the part MuJoCo/Isaac exist to solve). This note is the
  applied entry point; the rest of Ch. 8 fills in the terms.
