# pick_place

Language-conditioned pick-and-place build (repo north star). Full plan &
checklist: [`../notes/proj_pick_place_plan.md`](../notes/proj_pick_place_plan.md).

## Status

| Step | State |
|---|---|
| 1 controller + viewer | ✅ EE-pose follower (IK + rate limiter) |
| 2 phone WebXR teleop | ✅ clutched relative mapping, haptics, wrist view |
| 3 pick-place env | ✅ objects/bins, instructions, success/reset |
| 4 scripted expert | ✅ **300 demos generated** |
| 5 recording (LeRobot) | ✅ offline load + resume/merge + source tags |
| 6 data generation | ✅ 300 scripted (teleop-augment ready) |
| 7 training (smolvla) | 🔨 trains on Mac MPS |
| 8 inference rollout | ✅ deploy loop built |

## Step 1 — EE-pose target-follower

A Franka Panda in MuJoCo that follows an end-effector target pose, driven by a
swappable `TargetSource`. This is the controller substrate every later data
source (teleop, scripted expert) plugs into.

```
TargetSource ──(target TCP pose + gripper)──▶ EEController ──▶ MuJoCo
  DummyOrbitSource (bring-up, no teleop)         rate limit
  TeleopSource   (Step 2, phone)                 → DLS IK  (mr Jacobian/log)
  ScriptSource   (Step 4, expert)                → position ctrl[0:7]
                                                 → gripper ctrl[7]
```

### Modules
- `scene.py` — builds the scene with **MjSpec**: Franka `panda.xml` + floor +
  table + a **TCP site** on the hand + a mocap target marker; sets `gravcomp=1`
  on the arm (gravity handled by MuJoCo, no inverse dynamics).
- `ik.py` — `DLSIK`: damped-least-squares differential IK on the TCP site
  (`mj_jacSite`), orientation error via `mr.so3.matrix_log3`. Ch 5 + Ch 6.
- `controller.py` — `EEController`: pure target-follower. Cartesian **rate
  limiter** (safety net, *not* trajectory generation) → IK seeded from an
  internal reference config → position actuators.
- `target_source.py` — `TargetSource` ABC + `DummyOrbitSource`.
- `run.py` — live viewer / headless self-test.

### Run
```bash
# live viewer — on macOS the viewer needs mjpython (Cocoa main-thread loop):
.venv/bin/mjpython -m pick_place.run

# headless self-test (plain python; reports TCP error, writes out/selftest.png):
.venv/bin/python -m pick_place.run --headless
```
Self-test currently: ~9 mm mean / ~15 mm max TCP position error — this is the
position servo's tracking lag on a moving target, the regime where the human (or
policy) closes the loop.

## Step 2 — phone WebXR teleop

`teleop/` is a self-contained copy of the simple_slam server↔browser plumbing
(aiohttp WSS + WebXR pose page), SLAM stripped. The MuJoCo process *is* the
server (ws server in a daemon thread; sim+viewer on the main thread).

```bash
# 1) validate the phone stream first (no sim, plain python):
.venv/bin/python -m pick_place.run_teleop --validate

# 2) full teleop with the viewer (macOS → mjpython):
.venv/bin/mjpython -m pick_place.run_teleop
```
Then on the **Android** phone (same LAN): open `https://<LAN-IP>:8443/`, accept
the self-signed cert warning, **Start AR session**. Toggle **CLUTCH** to engage
motion (tap on → move → tap off to reposition = the ratchet); the **gripper**
slider opens/closes the hand. Watch the arm on the Mac screen.

- `teleop/server.py` — `TeleopServer`: threaded WSS, `SharedState.latest()`
  (phone→sim), `send_haptic()` (sim→phone).
- `teleop_source.py` — `TeleopSource`: clutched **relative** mapping (§4 of
  `proj_phone_teleop.md`): `ΔT = P₀⁻¹·T`, `R_align` change-of-basis, scale `s`.
  - Motion is measured in the phone's **gravity-aligned AR world frame** (not the
    device frame), so tilting/holding the phone differently during a motion
    doesn't corrupt the axes. `R_align` (`build_R_align`) maps that frame → robot
    base using a **calibrated forward** `_f0` and the live sim-view azimuth.
  - **Set Fwd** button: point the phone the way you want "into the screen" and
    tap — sets `_f0` empirically, so no WebXR/MuJoCo axis-convention guessing.
    `scale` (`s`) is the sensitivity knob.
  - **VIO jump rejection**: an ARCore loop-closure/relocalization can teleport the
    pose estimate; if a one-frame step exceeds `pos_jump`/`ang_jump` (or tracking
    drops), we re-latch without moving the robot, so the jump is invisible.
  - **HOME button** snaps the target to the home pose and disengages the clutch —
    the recover-from-anything escape hatch.
- `haptics.py` — `CollisionMonitor`, two edge-triggered channels: arm↔env
  (table/floor/bins) → **vibrate** (warning), fingers↔object → **sound** (grasp
  confirmation).

## Step 3 — pick-and-place task (`env.py`)

`PickPlaceEnv` extends the Franka scene with 3 graspable objects (box, capsule,
cylinder × color) and 2 walled open-top bins (mocap → immovable, collidable).
Grasp reliability (diagnosed the hard way):
- Objects: friction `[3.0, 0.1, 0.05]` + **`condim=6`** (torsional + rolling
  friction; default `condim=3` is sliding-only).
- Gripper **stiffened 4×** (`GRIP_STIFFEN`) — the stock Franka servo grips at only
  ~2 N, too weak to hold under lateral motion.
- **No sphere**: a ball can't be held by parallel jaws (low grip slips, high grip
  ejects it — geometric, not friction). Replaced with a **capsule**, which grasps
  like a cylinder when the gripper approaches across its short axis.
`reset()` randomizes
non-overlapping reachable placement, picks a target object + bin, and returns a
language **instruction** ("put the red box into the purple bin"). `success()` is
true when the target object settles inside the target bin → `run_teleop`
auto-resets with a success chime. Adds `scene` + `wrist` cameras for Step 4.

Teleop now runs the full task:
```bash
.venv/bin/mjpython -m pick_place.run_teleop     # prints the instruction; auto-resets on success
```
**Wrist camera view:** the loop renders the `wrist` cam (~30 Hz) and the server
streams it as MJPEG — open `https://localhost:8443/wrist` in a browser on the Mac
next to the sim window for the close-up grasp view. The image is **rotated so
control-forward points up** (`φ = atan2(fh·camₓ, fh·cam_y)` from the live view
azimuth, then centre-cropped) — otherwise it spins with the gripper yaw and
inverts your controls. Rendering is guarded by `_WristStreamer`; an offscreen-GL
conflict disables the stream with a warning instead of crashing teleop.
- `teleop/static/` — the AR page as a **full-screen control surface** (the camera
  passthrough isn't needed — you watch the Mac). Shows the **objective** up top
  (pushed by `server.push_instruction` on each reset, with late-joiner catch-up)
  and giant **OPEN/CLOSE** + clutch + home/set-fwd buttons.

## Step 5 — recording (`recorder.py`, LeRobot)

`Recorder` writes teleop demos as a **LeRobot v2 dataset**. Record while teleoperating:
```bash
.venv/bin/mjpython -m pick_place.run_teleop --record        # → pick_place/data/pick_place
```
- Per timestep (~15 Hz, downsampled from 500 Hz): scene+wrist **videos**,
  `observation.state` (ee_pos, ee_rot6d, gripper_width, joint_pos = 17),
  `action` = base-frame **EE-delta to next pose** (Δpos, Δrot_rotvec, gripper = 7),
  `task` = the instruction. Structured target fields → sidecar `episode_meta.jsonl`.
- **Auto-saves on success**; the phone **✕ DISCARD** button drops a botched attempt.
- The wrist obs is the **raw** mounted camera (not the operator-aligned live view).
- Saves are synchronous (brief stall between episodes) so files are flushed.
- Deps: `pip install 'lerobot[dataset]'`.
- **Create-or-resume**: recording into an existing dataset root **appends** (so
  scripts + teleop pool into one dataset). `rm -rf` the root to start fresh.
- **`recorder.finalize()`** (called automatically at the end of `run_script` /
  `run_teleop`) flushes writers — required for the dataset to be valid.
- **Source tags**: each episode's sidecar records `source` (`script`|`teleop`) so
  you can weight/filter the teleop quality slice vs scripted volume at train time.

### Loading the dataset (offline)

```python
from pick_place.dataset import load_dataset
ds = load_dataset()                                   # local, no HF Hub
ds = load_dataset(delta_timestamps={"action": [i/15 for i in range(8)]})  # (H,7) chunks
```
Two 0.6.0 gotchas handled by `dataset.load_dataset` / `Recorder`:
- LeRobot only hits the Hub when the **local metadata is incomplete** — so a
  `finalize()`d dataset loads fully offline from `root`.
- **Video decode backend**: torchcodec (LeRobot's default) needs a *system*
  FFmpeg — `brew install ffmpeg` (it doesn't bundle one, unlike pyav). With
  ffmpeg present it works and is the default; pass `video_backend="pyav"` for the
  self-contained fallback (no system dependency, slightly slower).

## Step 4 — scripted privileged expert (`expert.py`, `run_script.py`)

`ScriptedExpert` reads ground-truth object/bin poses and plans a pick→place as
waypoint segments — **rise → over-object → descend → grasp → lift → arc over the
bin wall → release** — driving the *same* `EEController` and recording via the
*same* `Recorder`. Runs headless (no viewer/realtime), so it's fast; failures are
**auto-discarded** (only successes saved). Per-episode randomization (heights,
grasp yaw, drop) injects multimodality.

```bash
# measure success rate only:
.venv/bin/python -m pick_place.run_script --episodes 20 --dry-run
# record N successful demos (auto-resumes an existing dataset):
.venv/bin/python -m pick_place.run_script --episodes 300
```
- ~75–85% success (box ~100, cylinder ~90, **capsule ~77** — the hard grasp).
- Grasp gotcha: the gripper is **yawed across** an object's axis (the capsule can
  only be grasped perpendicular to its long axis), and the approach **rises then
  descends vertically** — a low horizontal approach sweeps the open fingertips
  through tabletop objects and knocks them.
- ⚠ It uses waypoints + the controller's **rate limiter** for interpolation, *not*
  real trajectory generation (no time-scaling / screw motion). Fine for data;
  smoother demos would want a Ch-9 trajgen layer.

## Step 6 — training (`smolvla`, LeRobot)

**Policy = `smolvla`** — a small **language-conditioned flow-matching VLA**
(SmolVLM-500M + flow action expert), i.e. a mini-π0. Vanilla Diffusion Policy in
LeRobot has *no* language encoding, so it can't do the conditioned task; language
conditioning ⇒ a flow-matching VLA.

```bash
pip install 'lerobot[dataset,training,smolvla]'   # + brew install ffmpeg (for torchcodec)
lerobot-train --dataset.repo_id=local/pick_place \
  --dataset.root=pick_place/data/pick_place \
  --policy.path=lerobot/smolvla_base --policy.device=mps --policy.push_to_hub=false \
  --policy.optimizer_lr=1e-5 \
  --batch_size=4 --steps=10000 --save_freq=2000 --num_workers=0 \
  --wandb.enable=false --output_dir=outputs/smolvla_finetune
```
- **Trains on Mac MPS** (verified; ~1.2 s/step). Cloud = same with
  `--policy.device=cuda`, bigger batch/steps.
- **Finetune `smolvla_base`** (pretrained expert) — needs the dataset to use its
  3-camera names (`camera1/2/3`); our env/recorder now produce those (`CAMERAS` in
  `env.py`). Use `--policy.type=smolvla` instead to train the expert from scratch.
- **`--policy.optimizer_lr=1e-5`**: the default `1e-4` *diverges* when finetuning
  the pretrained (cross-embodiment) expert on a small batch — lower it.
- Video decode uses **torchcodec** by default (`brew install ffmpeg`); add
  `--dataset.video_backend=pyav` for the self-contained fallback.

## Step 8 — inference rollout (`run_infer.py`)

Runs a trained checkpoint in the sim. Deploy loop: build obs (render scene+wrist,
17-d state, instruction) → `preprocessor → policy.select_action → postprocessor`
→ **integrate the EE-delta** onto the current pose → same `EEController` → step.

```bash
# watch it (macOS → mjpython), on a decent checkpoint:
.venv/bin/mjpython -m pick_place.run_infer --viz live
# headless success rate (CPU avoids fighting an MPS training run):
.venv/bin/python -m pick_place.run_infer --viz none --device cpu --episodes 10 \
  --checkpoint outputs/smolvla_pickplace/checkpoints/last/pretrained_model
```
- **The policy has no 'done' signal** — it's reactive. Episodes terminate on
  `env.success()` (privileged ground truth) or `--max-decisions` timeout; after
  success the policy would just hover/drift (out-of-distribution).
- Decisions run at ~15 Hz (each EE-delta applied over ~33 sim steps); smolvla
  re-plans its 50-step action chunk internally.

### Design notes / gotchas
- **Franka arm actuators are position servos** (`ctrl[0:7]` = desired joint
  angle); `ctrl[7]` is the gripper, range `[0, 255]` (0 closed, 255 open).
- **Seed IK from the internal reference, not physical `qpos`.** Seeding from the
  lagging physical state makes the redundant 7-DoF IK jump between solutions.
- **The dummy orbit is deliberately gentle.** A larger orbit drives the elbow
  (joint 4) into its limit / through a singularity around home; that's a
  workspace fact, not an IK bug. Real targets (teleop/expert) must also respect
  the dexterous workspace.
- Run as `-m pick_place.run` from the repo root so `import mr` resolves. The
  local `mujoco/` FK-demo folder does **not** shadow the installed package.
