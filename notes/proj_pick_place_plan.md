# Project note — Pick-and-place build: execution plan & checklist

> **What this is.** The concrete, sequenced build plan for the
> language-conditioned pick-and-place project (the north star of this repo — see
> `CLAUDE.md`). It turns the design in
> [`proj_phone_teleop.md`](proj_phone_teleop.md) and
> [`proj_data_source_spectrum.md`](proj_data_source_spectrum.md) into an ordered
> checklist. All code lives in `pick_place/`.
>
> **Status:** plan agreed; executing **Step 1**.

---

## North star

Natural-language task → robot executes it. Concretely: collect
`(camera obs, EE-pose action, gripper, language)` demos in MuJoCo → train a
conditioned **flow-matching / diffusion** policy → run it at inference. This is
**Phase 0–2** of the roadmap; the MR chapters (SE(3), FK, Jacobian, IK) are the
substrate every layer sits on.

---

## Locked design decisions

These are settled — don't relitigate without a reason.

- **Action space = EE-pose delta.** Portable, matches UMI/π0, keeps IK as a clean
  reusable layer. Cost: IK-in-the-loop at deploy (accepted).
- **Controller = pure target-follower.** Always the same job: *target EE pose this
  tick → numerical IK (Ch 6) → position actuators*. It never knows the target's
  source. Includes a **built-in Cartesian rate/velocity limiter** (single-step
  online clamp toward target) as a safety net for dropped packets / clutch
  re-engage. **No computed-torque control.**
- **Gravity compensation** via MuJoCo per-body `gravcomp="1"` — free, no explicit
  inverse dynamics.
- **Target-source abstraction** sits *above* the controller; two implementations
  emit the **identical** `(EE-delta, gripper)` stream so the policy sees one
  action distribution:
  - `TeleopSource` — passthrough; the 60–90 Hz phone stream is already dense.
  - `ScriptSource` — sparse waypoints → **trajectory generation** → dense stream.
- **Trajectory generation = script-side layer only** (time-scaled Cartesian/screw
  interpolation between via-points). Exists to make scripted data *look like*
  teleop data. **Not** a controller flag; teleop simply doesn't attach it.
- **Embodiment = Franka Panda + gripper** from `mujoco_menagerie`.
- **Viewer = native `mujoco.viewer.launch_passive`.** Operator watches the
  computer screen; phone is controller-only → **no browser viewer needed.**
- **Teleop = WebXR `immersive-ar` on Android Chrome**, reusing the existing
  well-tested server↔browser combo. Native Android app only if a real improvement
  is needed. Clutched *relative* mapping per `proj_phone_teleop.md` §4.
  Clutch = toggle, gripper = button/slider.
- **Haptics = two distinct, edge-triggered channels** (fire on rising edge, never
  sustained): **vibration = warning (arm↔bin collision)**, **sound = confirmation
  (grasp closure)**. `navigator.vibrate` (Android). Swappable if the other feel
  reads faster. Contacts come from MuJoCo `data.contact`.
- **Data format = LeRobot, from day 1.** No custom format → Steps 7–8 are "point
  trainer at dataset," not "write a converter."
- **Recorded tuple** (per `proj_phone_teleop.md` §7): camera obs + proprioception
  (joint pos/vel, EE pose, gripper state) · EE-delta action · gripper cmd ·
  language instruction.
- **Data mix:** teleop = quality slice, scripted privileged expert = volume. Fix
  the ratio *after* setting an episode budget (revisit the initial 30/70 — teleop
  more likely ~10–20%; keep enough human multimodality for the diffusion policy).

---

## Execution sequence (checklist)

### Step 1 — Controller + viewer (dummy target)  ✅ *done*
- [x] Create `pick_place/` package.
- [x] Pull Franka Panda + gripper from `mujoco_menagerie`; scene builds via MjSpec
      (Franka + floor + table + TCP site + mocap target marker).
- [x] Numerical IK (`ik.py` DLSIK, reuses `mr.so3.matrix_log3` — Ch 5 + Ch 6):
      EE target pose → joint targets.
- [x] Controller (`controller.py`): target EE pose → Cartesian rate limiter → IK
      (seeded from internal reference) → position actuators; `gravcomp=1` on arm
      bodies (no sag, no torque code).
- [x] `TargetSource` ABC + `DummyOrbitSource` (scripted orbit, **no teleop**).
- [x] `mujoco.viewer.launch_passive` viewer + headless self-test
      (`~9 mm` mean TCP tracking error → PASS). *Confirm the live viewer on your
      Mac.*
- Gotcha logged: a large dummy orbit drives elbow joint 4 into its limit /
  singularity near home → keep dummy motion in the dexterous workspace.

### Step 2 — Teleop  🔨 *built; pending live phone tuning*
- [x] Self-contained copy of the simple_slam server↔browser combo in
      `pick_place/teleop/` (aiohttp WSS in a daemon thread + WebXR phone page),
      SLAM stripped. `run_teleop.py --validate` prints the phone stream (validate
      before mapping).
- [x] `TeleopSource` (`teleop_source.py`): clutched relative mapping (§4) →
      absolute EE target; `R_align` + scale `s` params (unit-tested: phone-right→
      robot-right, phone-up→robot-up, ratchet freeze/re-engage). *Tune `R_align`/`s`
      to feel on the phone.*
- [x] Clutch = **toggle** button, gripper = **slider** (+ open/close) on the AR
      dom-overlay.
- [x] Haptics: plumbing end-to-end + the **warning channel wired now** —
      `CollisionMonitor` (`haptics.py`) edge-triggers a distinct vibrate pattern on
      arm/gripper↔table/floor contact (fires once per collision, not every tick).
      The **confirmation channel** (grasp closure → sound) comes with objects in
      Step 3.
- [x] VIO robustness (loop-closure / relocalization yanks the target): **jump
      rejection** — if the phone pose steps >`pos_jump`/`ang_jump` in one frame or
      tracking drops, re-latch without moving the robot (jump invisible to arm);
      **HOME recover button** — snaps target to the home pose + disengages clutch.
- [ ] Live: open the page on the Android phone, verify pose drives the arm and it
      "feels natural"; tune `R_align`/`s` (and `pos_jump`/`ang_jump` if needed).

### Step 3 — Pick-and-place environment  🔨 *built; pending live run*
- [x] `env.py` `PickPlaceEnv`: 3 graspable free-body objects (shape × color) +
      2 walled open-top **bins as mocap bodies** (immovable, collidable).
- [x] Randomized non-overlapping, reachable placement per `reset()` (rejection
      sampling; objects settle onto the table).
- [x] Walled bins (`BIN_WALL_H`) so a straight-line place clips the rim — the
      planning challenge for teleop/expert to arc over.
- [x] Per-reset **instruction** ("put the red box into the purple bin"); success =
      target object settled inside target bin → **auto-reset** (+ success chime).
- [x] `scene` + `wrist` cameras (for Step 4 observations).
- [x] Unlocked the **grasp-confirmation sound** haptic (finger↔object) that was
      deferred from Step 2; two-channel `CollisionMonitor` (vibrate warn / sound
      grasp), both edge-triggered.
- [ ] Live: `mjpython -m pick_place.run_teleop` — teleop a pick→place, confirm the
      instruction, grasp chime, collision buzz, and auto-reset all fire.

### Step 4 — Scripted privileged expert
- [ ] Uses ground-truth state → grasp/lift/arc/place via-points → `ScriptSource`
      trajectory generation → same dense `(EE-delta, gripper)` stream as teleop.
- [ ] Validates the whole pick-place loop end-to-end with **no human in it**.

### Step 5 — Recording pipeline (LeRobot)
- [ ] Log the recorded tuple in **LeRobot dataset format** (parquet + mp4 +
      episode index); both teleop and script feed it.
- [ ] Sanity: reload a recorded episode, replay actions, confirm fidelity.

### Step 6 — Data generation
- [ ] Collect scripted-expert volume + teleop quality slice; label each episode
      with its instruction. Fix the mix ratio here.

### Step 7 — Train the policy
- [ ] Train a conditioned flow-matching / diffusion policy on the LeRobot dataset
      (GPU / cloud).

### Step 8 — Inference
- [ ] Run the trained policy in the sim; display it executing from a language
      instruction.

---

## MR learning payoff (why this build *is* the curriculum)

SE(3) poses & frames (Ch 3) · FK (Ch 4) · Jacobian/velocity kinematics (Ch 5) ·
numerical IK (Ch 6) · trajectory generation (Ch 9, the script layer) · and the
modern learned replacement for planning/grasping (Ch 9/10/12 SOTA). The teleop
transform tree is the single richest Ch-3 exercise in the repo — see
`proj_phone_teleop.md` "MR learning payoff."
