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

### Step 4 — Scripted privileged expert  ✅ *done*
- [x] `expert.py` `ScriptedExpert`: ground-truth poses → rise/over/descend/grasp
      (gripper yawed **across** the object axis) /lift/**arc-over-wall**/place
      waypoints → drives the same `EEController`, records via the same `Recorder`.
- [x] `run_script.py`: headless (fast, no viewer/realtime), per-episode
      randomization for multimodality, **auto-discards failures** (only successes
      saved). ~**75–85%** success (box~100, cyl~90, capsule~77 — capsule is the
      hard grasp; discard keeps the data clean).
- [x] **Generated 100 demos** → `pick_place/data/pick_place` (8.5k frames @ 15 Hz,
      all 6 task variants, bins balanced 50/50). Command:
      `python -m pick_place.run_script --episodes 100`.
- Gotcha logged: the pre-grasp must **rise then descend vertically** — a low
      horizontal approach sweeps the open fingertips through tabletop objects
      (knocked the long capsule most).

### Step 5 — Recording pipeline (LeRobot)  🔨 *built (teleop path); pending live run*
- [x] `recorder.py` `Recorder`: writes **LeRobot v2 (0.6.0)** — scene+wrist videos,
      `observation.state` [ee_pos, ee_rot6d, gripper_width, joint_pos] (17),
      `action` base-frame EE-delta-to-next-pose [Δpos, Δrot_rotvec, gripper] (7),
      `task` = instruction. Structured target fields → sidecar `episode_meta.jsonl`.
      See [`proj_lerobot_format.md`](proj_lerobot_format.md).
- [x] Downsampled ~15 Hz from the 500 Hz loop; one-frame-lag for the delta action.
- [x] Wired into `run_teleop --record`: **auto-save on success**, phone **DISCARD**
      button to drop a botched attempt (verified it excludes the episode).
- [x] **Offline load + merge solved**: `recorder.finalize()` (auto-called) makes
      the dataset valid; `dataset.load_dataset()` loads locally (pyav backend, no
      Hub); `Recorder` **create-or-resumes** so scripts + teleop pool into one
      dataset. Verified create/resume/offline-load/action-chunking.
- [x] **300 scripted demos generated** → 25.8k frames @ 15 Hz, all 6 variants,
      loads offline, `(16,7)` action chunks. (bins 177/123 orange/purple — mild
      imbalance; rebalance later if the policy shows bin bias.)
- [ ] Live: `mjpython -m pick_place.run_teleop --record` to *append* teleop demos
      to the same dataset (reactively, against observed policy weaknesses).

### Step 6 — Data generation  ✅ *scripted done; teleop ready*
- [x] **300 scripted-expert demos** → one LeRobot dataset (source-tagged `script`,
      all 6 variants, bins 177/123).
- [ ] Teleop **quality slice** appended reactively (`run_teleop --record`
      auto-resumes the dataset, tags `teleop`); fix the script/teleop mix against
      observed policy weaknesses (~10–20% teleop is plenty).

### Step 7 — Train the policy  🔨 *smolvla on Mac MPS*
- **Policy = `smolvla`** (SmolVLM-500M + flow-matching action expert): the small,
  **language-conditioned** flow-matching VLA — a mini-π0. Chosen because vanilla
  Diffusion Policy in LeRobot **ignores language** (0 text encoding), so it can't
  do our conditioned task; language conditioning ⇒ a flow-matching VLA.
- **Verified smolvla trains on MPS** (loss 3.5→1.85 by step 200, ~1.06 s/step @ batch 4).
- Setup gotchas (all handled): extras `pip install 'lerobot[dataset,training,smolvla]'`;
  `--dataset.video_backend=pyav`; finetuning `lerobot/smolvla_base` fails on a
  **camera-name/count mismatch** (base expects camera1/2/3, we have scene/wrist) →
  `--policy.type=smolvla` (adapts to our 2 cameras) or `--rename_map`.
- **Run:**
  ```
  lerobot-train --dataset.repo_id=local/pick_place \
    --dataset.root=pick_place/data/pick_place --dataset.video_backend=pyav \
    --policy.type=smolvla --policy.device=mps --policy.push_to_hub=false \
    --batch_size=4 --steps=10000 --save_freq=2000 --num_workers=0 \
    --wandb.enable=false --output_dir=outputs/smolvla_pickplace
  ```
  Cloud = same, `--policy.device=cuda` + bigger batch/steps (+ finetune smolvla_base).
- [ ] Watch loss; test a checkpoint at inference (Step 8).

### Step 8 — Inference  🔨 *deploy loop built; awaiting a trained checkpoint*
- [x] `run_infer.py`: load policy+processors from a checkpoint → per decision
      build obs (render cams + 17-d state + instruction) →
      `preprocessor → select_action → postprocessor` → **integrate EE-delta** onto
      the current pose → same `EEController`. `--viz live` (mjpython) or headless.
      Pipeline validated on the step-2k checkpoint (runs clean; places nothing yet).
- **No policy 'done' signal** — reactive; WE terminate on `env.success()` (GT) or
  a `--max-decisions` timeout. Post-success behaviour is OOD (hover/drift).
- [ ] Run `--viz live` on a later checkpoint; measure success rate; check it
      actually *reads the instruction* (same scene, different target → different
      action). If it ignores language / is brittle → more steps, or add teleop.

---

### Step 8.5 — grounding diagnosis → mask pivot  🔨 *data path built; retrain pending*
- **Finding (DiT, 3-cam, CLIP-text conditioning):** `run_infer --probe-language`
  (same scene, vary only the instruction) → **0/3 reached the named object**.
  Language *shifts* behaviour (3 different endpoints) but grounding is ~random,
  and the arm never gets closer than ~10 cm to any object. Root cause: 300 demos
  can't teach language→object **spatial grounding** on a from-scratch head over
  CLIP features. This is the checkpoint's ceiling, not a deploy-loop knob.
- **Two paths:** (a) end-to-end **smolvla** (pretrained VLM *supplies* grounding),
  or (b) **factor grounding out of the policy** — resolve the target *outside* the
  net and hand the DiT an already-grounded view. Pursuing (b) first as a clean
  ablation ("is the DiT's *control* fine and only grounding broken?").
- **Chosen (b): dimmed-RGB target mask + 2 cameras.**
  - Cut cameras 3→2 (`scene` third-person + `wrist`); dropped the redundant
    `front` view (CLIP×3cams×2obs was the batch bottleneck; fewer dims = better at
    300 demos). `front` camera stays in the model, just off the obs list.
  - On the `scene` view only, **dim everything except the target object + target
    bin** (`env.dim_except`, MuJoCo segmentation, `MASK_DIM=0.35`); wrist stays
    clean for the grasp. Grounding is privileged (sim seg) now; the *same* mask
    channel comes from SAM / Grounding-DINO on hardware → sim→real transfers.
  - New dataset root **`pick_place/data/pick_place_masked`** (original 3-cam data
    left intact). Recorder + `run_infer` apply the identical mask (train/deploy
    parity). `--probe-language` now varies the *highlight*, not the sentence.
- **Deploy-loop knob also added:** `--n-action-steps` (default 8; trained 24) —
  execute fewer chunk actions open-loop before re-observing (reactivity).
- [ ] Regenerate ~300 masked demos → retrain DiT (`outputs/dit_flow_masked`) →
      re-run `--probe-language`. Success signal: **0/3 → ~3/3**. If it grounds,
      the control stack was fine all along and grounding was the whole problem.

### Step 8.6 — deploy-loop bug: measured-pose vs running-reference integration  ✅ *fixed*
- **Symptom:** DiT, smolvla, AND ACT all trained to low loss (ACT 8.8→0.38) yet
  *every* policy failed at deploy the same way — a tell that the fault is **below
  the model**, in the shared deploy loop, not the net.
- **Decisive diagnostic — `pick_place/replay_gt.py`:** feed the scripted expert's
  *own* ground-truth EE-delta stream through the exact deploy loop.
  - GT expert: **5/6** success. `openloop_drift = 0.0 mm` (Σdeltas reconstructs the
    path exactly — the action representation is perfect; **no compounding error**).
  - **measured-pose integration (old `run_infer`): 0/6**, tracking error 126–247 mm
    and *growing*.
  - **running-reference integration (fix): 5/6**, error 14–28 mm and *constant* —
    matches GT exactly.
- **Root cause:** the position servo trails its command by a ~constant lag *L*
  (that's what `EEController.q_ref`/`cmd_pos` exist to decouple). `_integrate`
  rebuilt each target as `measured_TCP + delta`; since `measured_TCP` already lags
  by *L*, the lag is **re-injected every decision and compounds** to `k·L`. The
  expert never hit this — it commanded *absolute distant waypoints*.
- **Fix (`run_infer.py`, both `rollout` + `probe_language`):** seed a running
  reference at the start pose and integrate each delta **onto the reference**
  (`ref ← ref ⊕ delta`), not onto the measured TCP. Arm then trails by a constant,
  in-tolerance ~15–28 mm. This is the standard delta-action deployment pattern.
- **Reframes Step 8.5:** the 0/3 grounding verdict was measured through the broken
  loop, so it's **not trustworthy** — re-run `--probe-language` on the fixed loop
  before concluding anything about grounding. Control was likely fine all along.
- *Open item:* pure running-reference never re-syncs to measured, so a genuinely
  stuck arm (contact/unreachable) would let the reference run away. Add gentle
  re-sync only if that shows up; not needed for free-space motion.

## MR learning payoff (why this build *is* the curriculum)

SE(3) poses & frames (Ch 3) · FK (Ch 4) · Jacobian/velocity kinematics (Ch 5) ·
numerical IK (Ch 6) · trajectory generation (Ch 9, the script layer) · and the
modern learned replacement for planning/grasping (Ch 9/10/12 SOTA). The teleop
transform tree is the single richest Ch-3 exercise in the repo — see
`proj_phone_teleop.md` "MR learning payoff."
