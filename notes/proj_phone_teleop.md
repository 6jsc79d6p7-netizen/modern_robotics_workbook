# Project note — Phone as a 6-DoF teleop controller

> **Status:** design decided, not yet built. This is the **data-collection
> interface** for the pick-and-place build (Phase 0–1: collect human demos in
> MuJoCo → train a conditioned flow-matching policy, Rung 4 → π0, Rung 5). It is
> also the single best hands-on application of **Ch 3 (SE(3), frames, adjoints)**
> in the whole project — the core of it is a transform tree.

---

## 1. The decision (what we're building and why)

**Build a full 6-DoF phone teleop from the start.** You wave your phone in the
air; the simulated gripper mirrors its motion; a screen button toggles the
gripper. Recorded episodes become imitation-learning demos.

The choices, and the reasoning:

- **Why phone, not SpaceMouse or keyboard?** A 3D SpaceMouse is the "proper" tool
  but costs money; keyboard EE-nudging is unnatural and slow. A phone you already
  own has the sensors to be a real 6-DoF spatial controller.
- **Why full 6-DoF immediately (not orientation-first staging)?** Because **you
  are the feedback loop** (see §2) — inaccuracy is correctable live, so there's no
  need to de-risk with a translation-free version first. Go straight for the real
  thing.
- **Why it's acceptable to be inaccurate?** Teleoperation is *closed-loop through
  the human*: you watch the sim and correct continuously, exactly like steering a
  car. The phone only needs to be *responsive and consistent*, not metrically
  precise.
- **Both phones available** → platform is a free choice; see §6 (lean iPhone +
  ARKit).

---

## 2. Why "inaccurate is fine" — you are the feedback controller

The key conceptual point, and the reason this works:

> A teleop device does **not** need to be accurate in any absolute sense. The
> operator (you), watching the robot, *is* a feedback controller — you observe the
> error between where the gripper is and where you want it, and you move your hand
> to cancel it, many times per second. Drift, scale error, and jitter get absorbed
> by this human visual-servoing loop.

What the device *does* need:
- **Low latency** (so your corrections land before the error grows) — feel, not
  precision.
- **Consistency / monotonicity** (phone-right always → gripper-right; the mapping
  doesn't flip or warp), so your motor intuition stays calibrated.
- **A clutch** (§5) so you can reposition your hand without dragging the robot —
  this also quietly cancels accumulated VIO drift on every re-engage.

This is why we can tolerate ARKit/ARCore VIO drift, an un-calibrated motion scale,
and a roughly-aligned coordinate frame: the loop is closed on *your eyes*, not on
sensor accuracy. (Contrast: an *open-loop* replay of a recorded trajectory would
need real accuracy — but that's not what teleop is.)

---

## 3. How it works — VIO gives pose, we stream and map it

The phone's **ARKit (iOS) / ARCore (Android)** already runs **visual-inertial
odometry (VIO)** — fusing camera + IMU into a drift-bounded 6-DoF pose in a fixed
world frame. *That is the "SLAM" you wanted; we consume it, we don't build it.*

Data flow:

```
 ┌─────────────┐  6-DoF pose T_ar_p(t)        ┌──────────────────────────────┐
 │  PHONE app  │  + button states             │            MAC               │
 │ ARKit/ARCore│ ───── WiFi (UDP/websocket) ─► │ listener → frame-map (§4)    │
 │  VIO pose   │      ~60–90 Hz               │   → EE target → IK (Ch 6)    │
 │  + buttons  │ ◄──── (optional haptics) ──── │   → MuJoCo step → record     │
 └─────────────┘                              └──────────────────────────────┘
```

The app emits, per frame: phone pose `T_ar_p(t)` (position + orientation quat),
**clutch/engage** button, **gripper** button, maybe a **scale** toggle. The Mac
maps that to an end-effector target and drives the sim.

---

## 4. The MR core — the transform tree (this is the Ch 3 payoff)

The whole problem reduces to: *"the phone moved like this in its space; make the
gripper move correspondingly in the robot's space, as it looks to me on screen."*
That is a **change-of-frame** problem — exactly SE(3) + adjoints from Ch 3.

**Frames involved:**

| frame | meaning |
|---|---|
| `{ar}` | phone's AR world frame (ARKit origin, fixed at app start) |
| `{p}` | phone device frame (moves with the phone); ARKit gives `T_ar_p(t)` |
| `{b}` | robot base frame (MuJoCo world) |
| `{e}` | end-effector frame; current pose `T_b_e(t)` |
| `{v}` | **operator view frame** — the virtual camera you watch the sim through |

**Clutched *relative* mapping** (the standard, drift-robust scheme):

1. On **engage** (clutch press) at `t₀`, store the phone pose `P₀ = T_ar_p(t₀)` and
   the current EE pose `E₀ = T_b_e(t₀)`.
2. Each frame while engaged, compute the phone's **relative motion since engage**:
   ```
   ΔT = P₀⁻¹ · T_ar_p(t)   ∈ SE(3),   ΔT = (ΔR, Δp)
   ```
   (the "subscript-cancellation" move from Ch 3 — `T_p₀_p(t)`, displacement in the
   engage frame).
3. Re-express that displacement in the **operator's view frame** and apply it to
   the EE's engaged pose:
   ```
   p_target = p(E₀) + s · R_align · Δp           # s = translation scale
   R_target = R_align · ΔR · R_alignᵀ · R(E₀)    # rotate the EE like the phone rotated
   ```
   - `R_align` is a **fixed rotation** that maps phone-device axes → operator-view
     axes. Getting it right is what makes *"move phone left ⇒ gripper goes left on
     screen."* It's a pure **change of basis / adjoint** — the most concrete Ch-3
     exercise you'll do. (`R_alignᵀ` conjugating `ΔR` re-expresses the phone's
     rotation in view axes — same `R[ω]Rᵀ` similarity-transform idea from 3a.)
   - `s` is a tunable **motion scale** (phone wave of 20 cm → robot moves `s·20`
     cm). Lets a small desk-space hand motion cover the robot workspace.
4. **Drive the sim from `T_b_e^target`:** two options —
   - **explicit IK (reuses Ch 6):** numerical IK → joint targets → position
     actuators. More control; this is the "policy → IK → controller" stack.
   - **MuJoCo mocap weld (easiest):** weld a mocap body to the EE, set its pose to
     the target, let MuJoCo's solver drag the arm. Fast to stand up.

Why **relative/clutched** instead of absolute (`T_b_e = R_align·T_ar_p` directly)?
Because it (a) decouples your hand's *starting* position from the robot's, (b)
cancels VIO drift on every re-engage, and (c) lets you "ratchet" — engage, move,
release, recenter your hand, re-engage — to cover a big workspace from a small
desk.

---

## 5. Controls (the buttons)

- **Clutch / engage** (hold): robot follows the phone only while held. Release to
  freeze and reposition your hand. *Required* — it's also the drift reset.
- **Gripper** (tap toggle): open/close. Binary is fine to start; could be analog
  later.
- **Scale toggle** (optional): coarse/fine `s` for gross reach vs delicate
  insertion.
- (Optional later) **record start/stop** + a **language tag** picker so each demo
  is auto-labeled ("pick up the apple") for the conditioning channel.

---

## 6. Platform & build approach

- **Recommended: iPhone + ARKit** — the most robust VIO and the smoothest API.
  ARCore (Android) is a fine fallback; you have both, so default to iPhone unless
  iOS dev friction (Xcode/provisioning) annoys you.
- **App:** a small native app — Swift + ARKit (`ARWorldTrackingConfiguration`,
  read `frame.camera.transform`) or Kotlin + ARCore. It streams pose + button
  state over **UDP** (lowest latency) or a websocket to the Mac's IP on the LAN.
  - *Shortcut to de-risk:* some off-the-shelf "ARKit pose streamer / OSC" apps
    broadcast 6-DoF pose already — try one first to validate the Mac-side mapping
    before writing our own app.
- **Mac side:** a small listener (Python, in the project venv) that parses the
  stream, runs §4, and talks to MuJoCo. No GPU; pure CPU/Mac.
- **Protocol payload (per packet):** `[timestamp, px,py,pz, qx,qy,qz,qw,
  clutch, gripper]`. Quaternion for orientation (no gimbal lock; see the parked
  quaternion note — this is where it becomes practically necessary).

---

## 7. What we record (the imitation dataset)

Each timestep of an engaged demo logs a tuple:

- **observation:** camera image(s) from sim camera(s) + **proprioception**
  (joint pos/vel, current EE pose, gripper state);
- **action:** the commanded target — **EE-pose delta** *or* joint target (decide
  the **action space** up front; ties to the FK/IK FAQ — EE-space leans on the IK
  layer, joint-space doesn't);
- **language label:** the task string for conditioning;
- **gripper command.**

This is exactly the `(observation, action-chunk, language)` data a conditioned
flow-matching / Diffusion / ACT policy consumes (Rung 4). Human teleop gives
*natural, multimodal* demos — including an intuitive solution to the planning
challenge (§8).

> **See also** [`proj_data_source_spectrum.md`](proj_data_source_spectrum.md) for
> where this teleop-in-sim data sits vs. egocentric-human data (UMI etc.) — our
> phone rig is the same `SE(3)` retargeting idea, at the zero-embodiment-gap end.

---

## 8. How it plugs into the bigger pick-and-place build

- **Phase 0:** MuJoCo env — Franka + gripper + objects + a **walled bin** placed
  so a straight-line place would hit the wall (the planning challenge). All CPU.
- **Phase 1:** collect demos. **You teleoperating** naturally **arc over the bin
  wall** — the avoidance is baked into the data with zero planning code. (A
  scripted privileged-state expert with hand via-points, or RRT\* for clutter, is
  the *volume* complement; teleop is the *quality* layer.)
- **Phase 2+:** train the conditioned flow-matching policy on this data (GPU,
  cloud); later layer SAM/AnyGrasp/cuRobo and π0 (Rung 5).

So this note is the **front door** of the data pipeline. Build it well and demo
collection is pleasant; build it badly and everything downstream starves.

---

## 9. Risks / open questions

- **`R_align` calibration** — the make-or-break for "feels natural." Likely a
  quick interactive tune (move phone +X, see which way the gripper goes, pick the
  rotation that matches the view). Pure Ch-3 frame algebra.
- **Latency feel** — WiFi + VIO + sim step. Target < ~50 ms end-to-end; UDP over
  websocket helps. May need light EMA smoothing on the pose (trade lag vs jitter).
- **Drift over a long demo** — bounded by VIO + reset on every clutch re-engage;
  keep demos short (a few seconds) anyway.
- **Scale tuning `s`** — find a value where a comfortable desk-space hand motion
  spans the workspace without feeling twitchy.
- **Gripper-button reliability / debounce** — a missed close ruins a demo.
- **Action-space choice** (EE-delta vs joint) — decide before logging, since it
  fixes the dataset format and whether an IK layer is in the deploy loop.
- **iOS dev friction** — Xcode, free provisioning profile re-signs every 7 days;
  acceptable for a personal build.

---

## 10. Build checklist (next steps — no code yet)

1. Pick platform (default **iPhone/ARKit**) and confirm a streaming approach
   (off-the-shelf pose-streamer first, custom app if needed).
2. Reinstall MuJoCo + `mujoco_menagerie` (Franka) into `.venv` (the rebuild
   dropped it). Stand up a static Franka + table scene.
3. Mac-side UDP/websocket listener that prints incoming phone pose — *validate the
   stream before any mapping.*
4. Implement §4 mapping (start with **mocap weld** to skip IK) and tune
   `R_align` + `s` interactively.
5. Add clutch + gripper buttons; verify "feels natural" in the empty scene.
6. Add objects + walled bin; do a few hand-collected pick-place demos.
7. Add recording (the §7 tuple) + a language tag; collect a small dataset.

---

## MR learning payoff (why this is worth the detour)

This single component exercises, *for real*: SE(3) poses and the matrix
quaternion/rotation representations (Ch 3 + the parked quaternion note), **frame
changes and the adjoint / `R[ω]Rᵀ` similarity transform** (`R_align`), the
**subscript-cancellation `P₀⁻¹·T`** relative-pose trick (Ch 3), and the **EE-pose
→ IK → controller** stack (Ch 4–6). If the transform tree here clicks, the rest of
the project's pose bookkeeping (camera extrinsics, grasp poses, the policy action
space) will feel like the same muscle.
