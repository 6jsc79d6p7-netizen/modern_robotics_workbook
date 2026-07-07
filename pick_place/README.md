# pick_place

Language-conditioned pick-and-place build (repo north star). Full plan &
checklist: [`../notes/proj_pick_place_plan.md`](../notes/proj_pick_place_plan.md).

## Status — Step 1 done: EE-pose target-follower

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
next to the sim window for the close-up grasp view. (Rendering happens in the sim
loop guarded by `_WristStreamer`; if the offscreen GL context ever conflicts with
the viewer it disables itself with a warning instead of crashing teleop.)
- `teleop/static/` — the AR page (clutch toggle, gripper slider, haptics).

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
