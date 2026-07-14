# 7c — Applied: Haptic Teleoperation with a Delta + Wrist Master

> A north-star design exercise (not a book section): **build the haptic feedback
> for a teleoperated surgery rig.** An open-chain surgical robot (the *slave*)
> does the cutting and carries a force/torque sensor; the surgeon drives it
> through a **delta robot + 3R spherical wrist** (the *master* haptic device).
> Goal: the surgeon should *feel* the forces the tool meets.
>
> This ties together everything: Ch. 5 statics `τ=JᵀF`, Ch. 6 spherical-wrist
> decoupling, Ch. 7 closed-chain Jacobian via `H`, and Ch. 3 adjoints/frames.
> It also surfaces what MR *doesn't* cover (frame-matching, loop stability, the
> device cancelling its own dynamics → [[08b_self_dynamics_compensation]]).

---

## 1. The one equation: haptic display is `τ = Jᵀ F`

To make the hand *feel* a wrench `F` at the handle, command the device's motors

$$ \boxed{\,\tau = J^T F\,} $$

This is exactly the Ch. 5b statics duality, run in reverse: there we computed the
joint torques needed to *resist* an external wrench; here we *manufacture* a
wrench on the hand by commanding those same torques. The slave's F/T sensor reads
the wrench the environment applies to the tool — that's what the surgeon should
feel — and `Jᵀ` maps it to the master's motor torques. Everything else is
plumbing: getting the *right* `J` and the *right* `F`, in consistent frames.

---

## 2. Which Jacobian — the delta's `J_a`, built via `H`

`τ = Jᵀ F` needs the **forward actuated Jacobian** `J_a` (the one with
`V = J_a q̇_a`, motor rates → handle twist) — **not** `J⁻¹`. For the closed-chain
delta you build `J_a` through the constraint Jacobian (7b §2, Eq. 7.21):

1. `H(q) q̇ = 0` → split `[H_a | H_p]` → `q̇_p = −H_p⁻¹ H_a q̇_a` (passive rates).
2. `J_a = J_1 [e₁ᵀ; g₂ᵀ; g₃ᵀ; …]` — push the reconstructed full joint rates
   through any one leg's Jacobian.

> *Shortcut:* each delta forearm is a **parallelogram = two-force member** (ball
> joints both ends → force only along its length), so `J_a⁻ᵀ` can be read off the
> forearm directions geometrically, Stewart-style (7b §3). The `H` route is more
> general; the shortcut is the elegant special case.

---

## 3. Delta can't render torques → add the wrist (and the split is clean)

A pure delta has **3 translational DOF** — it can push the handle linearly, so it
displays the **force** `f` but **never a moment** `m`. To feel torques you need 3
more *rotational* DOF: a **3R spherical wrist** on the platform. Delta + wrist =
6-DOF device → full 6-DOF wrench. (This is a real architecture — Force Dimension's
**omega.3** is a delta for force; **omega.6 / sigma.7** add an active wrist for
torque.)

**Does the split "delta→force, wrist→torque" hold exactly?** Yes — *if the grip
point is the wrist center.* The delta platform never rotates, so delta motors give
**pure translation** (`v` only); the wrist gives **pure rotation about its center**
(`ω` only, plus `ω×r` if the grip is offset). With grip = wrist center the device
Jacobian (space form, twist `V=(ω,v)`) is **block-antidiagonal**:

$$ \begin{bmatrix}\omega\\ v\end{bmatrix}
 = \begin{bmatrix} 0 & J_w \\ J_d & 0 \end{bmatrix}
 \begin{bmatrix}\dot q_{\text{delta}}\\ \dot q_{\text{wrist}}\end{bmatrix}
 \;\Longrightarrow\;
 \tau = J^T F = \begin{bmatrix} J_d^T\, f \\ J_w^T\, m\end{bmatrix}. $$

So `τ_delta = J_deltaᵀ f` and `τ_wrist = J_wristᵀ m` fall out exactly — the design
intuition is confirmed. `J_w` is just the ordinary open-chain 3R Jacobian; `J_d`
from the `H` route. This is the **same decoupling as the Ch. 6 spherical wrist**:
position joints and orientation joints separate when the wrist center is the
reference point. Grip ≠ wrist center → the zero blocks fill with `ω×r` coupling
and the split is only approximate.

---

## 4. Frame matching slave → master (the step MR skips)

The F/T sensor reads in the **slave tool frame**; the surgeon feels in the
**master handle frame**; these are *two different robots* plus a teleop alignment.
You cannot feed slave-frame numbers into the master Jacobian. Transport the wrench:

$$ F_{\text{master}} = [\mathrm{Ad}_{T}]^{T}\, F_{\text{slave}}, $$

where `T` is the alignment between the slave tool frame and master handle frame
(the "clutch" orientation map). Then `τ = J_masterᵀ F_master`. Two notes:

- **Wrench moment subtlety:** a spatial wrench is `F=(m,f)` with the space-frame
  moment carrying the `p×f` term (the Ch. 5 Ex 5.3 gotcha). Change frames with the
  **adjoint-transpose**, never by copying components. Since the sensor reads
  locally, **body form is often the simpler choice** (see §5).
- **Scaling:** real systems scale forces (amplify a 0.1 N microsurgical force so
  the surgeon feels it) — a deliberate gain inserted here.

---

## 5. Body vs space form of the delta Jacobian

If you keep the slave wrench in **body form** (master handle frame `{b}`), you need
the **body** actuated Jacobian `J_b`, since `τ = J_sᵀ F_s = J_bᵀ F_b`. Two routes:

**Route A — convert.** Same rule as open chains (Ch. 5b): body and space Jacobians
of the *same* mechanism differ by the adjoint of the **end-effector pose** —
*not* an arbitrary link:

$$ J_b = [\mathrm{Ad}_{T_{bs}}]\, J_s, \qquad T_{bs} = T_{sb}^{-1}, $$

`T_sb` = the handle/platform frame relative to the delta base (the device's FK,
which you know from its configuration). It's the end-effector transform because
the twist you care about lives **at the handle**.

**Route B — build native (cleaner).** The `H` passive-velocity step is
**frame-independent**: the constraint `V_1=V_2=V_3` ("all legs share the platform
twist") holds in any common frame, so `H_a, H_p`, and the `g_i` relations are
identical in space or body form. The frame enters *only* at the final assembly —
so just use leg 1's **body** Jacobian `J_{1b}` in Eq. 7.21 and `J_b` comes out
directly, reusing the same `H` solve. No conversion needed.

| | Route A (convert) | Route B (native) |
|---|---|---|
| 1 | build `J_s` via `H` | compute `g_i` via `H` (frame-free) |
| 2 | `J_b = [Ad_{T_bs}] J_s` | assemble with leg 1's **body** Jacobian |
| adjoint of | `T_bs` (handle pose) | — none |

> **Two distinct adjoints — keep them apart.** (i) Slave→master *transport* uses
> the adjoint of the **teleop alignment** (two different robots). (ii) Space↔body
> of the master's own `J_a` uses the adjoint of the master's **`T_sb`** (handle vs
> delta base). Both are `[Ad]` of an **end-effector-level** transform — never of a
> random intermediate link.

---

## 6. Gotchas & what MR doesn't cover

- **Two singularity sets.** Near a **delta singularity** (`J_d` rank-deficient,
  7b §5) forces in some direction can't be rendered and motor torques blow up;
  the **wrist** adds gimbal-lock singularities (Ch. 6). Safe workspace = both
  regions' intersection.
- **Loop stability / passivity.** `τ=JᵀF` is the *ideal* render. A real discrete
  feedback loop can inject energy and buzz/go unstable — needs ~**1 kHz** rate,
  virtual coupling, passivity control. (Beyond MR.)
- **The device must cancel its own dynamics.** Otherwise the surgeon feels the
  *device's* weight, inertia, and friction instead of the tissue. Gravity
  compensation + low moving inertia (a key reason deltas make good haptic masters)
  → a "transparent" device. **How this works → [[08b_self_dynamics_compensation]].**

---

## 7. The corrected pipeline

```
slave F/T sensor  (slave tool / body frame)
   │  transport to master handle frame:  F_b = [Ad_T]ᵀ F_slave   (+ alignment, + force scaling)
   ▼
F_b = (m, f)  at the master wrist center, master handle frame {b}
   ├─ f ─▶ τ_delta = J_deltaᵀ f      (J_delta via H-route, native body form = Route B)
   └─ m ─▶ τ_wrist = J_wristᵀ m       (J_wrist = 3R open-chain body Jacobian)
   ▼
apply τ   (≈1 kHz, after cancelling the device's own gravity/inertia/friction)
   ▼
surgeon feels the tissue wrench
```

**Verdict on the design:** the physics is right — `τ=JᵀF`, `H`-route for
`J_delta`, force/torque split via delta+wrist (clean at grip = wrist center). The
pieces MR doesn't hand you are the **slave→master frame transport** (with `p×f`
adjoint care), the **two-singularity** workspace, **loop stability**, and the
**self-dynamics cancellation** — the last of which is Chapter 8.
