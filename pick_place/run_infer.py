"""Step 8 — run the trained policy in the sim (inference rollout).

Deploy loop (the "policy → EE-delta → integrate → same controller" stack):

    obs (rendered cams + 17-d state + instruction)
      → preprocessor (normalize + tokenize language)
      → policy.select_action  → postprocessor (un-normalize)  → EE-delta (7,)
      → integrate onto the CURRENT EE pose → IK → EEController → step

The policy has **no 'done' signal** (reactive; it never stops). So WE terminate an
episode on `env.success()` (privileged ground truth) or a max-step timeout — after
success the policy would just hover/drift (out-of-distribution).

    # live viewer (macOS → mjpython):
    .venv/bin/mjpython -m pick_place.run_infer --viz live
    # headless success rate (plain python, e.g. CPU to avoid MPS contention while training):
    .venv/bin/python -m pick_place.run_infer --viz none --device cpu --episodes 10
"""
import argparse
import os
import queue
import threading

import numpy as np
import mujoco
import mujoco.viewer
import torch

from mr.so3 import matrix_exp3
from .env import PickPlaceEnv, CAMERAS, render_obs, OBJECTS, BINS
from .controller import EEController
from .scene import tcp_pose
from .target_source import TargetCommand
from .dataset import load_dataset
from .recorder import MASK_ROOT
from .run import _update_marker

DEFAULT_CKPT = "outputs/dit_flow_masked/checkpoints/last/pretrained_model"


def load_policy(ckpt, device, n_action_steps=None, dataset_root=MASK_ROOT,
                temporal_ensemble_coeff=None, noise_scheduler_type=None,
                num_inference_steps=None):
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import make_policy, make_pre_post_processors
    cfg = PreTrainedConfig.from_pretrained(ckpt)
    cfg.pretrained_path = ckpt
    cfg.device = device
    # diffusion-only: swap the *sampler* at inference (same weights, no retrain).
    # DDIM ~10 steps (fast, deterministic); DDPM ~num_train_timesteps steps
    # (slower, stochastic). Scheduler is built in the policy from cfg, so set here.
    if noise_scheduler_type is not None and hasattr(cfg, "noise_scheduler_type"):
        cfg.noise_scheduler_type = noise_scheduler_type
    if num_inference_steps is not None and hasattr(cfg, "num_inference_steps"):
        cfg.num_inference_steps = num_inference_steps
    if noise_scheduler_type is not None:
        print(f"[infer] sampler -> {noise_scheduler_type} "
              f"({num_inference_steps or 'default'} steps)")
    # How many chunk actions to execute open-loop before re-observing. The DiT
    # trains with n_action_steps=24 (of a 32-horizon) — a LOT of blind motion for
    # a task that needs visual servoing. Lower it (~8) to re-look ~3x as often;
    # this is a deploy-time knob, no retraining. See notes/proj_pick_place_plan.md.
    if temporal_ensemble_coeff is not None:
        # ACT temporal ensembling (paper default coeff 0.01): re-query every
        # decision and EMA-blend the overlapping chunk predictions for smoother,
        # more accurate control — no retraining. LeRobot requires n_action_steps=1
        # for this, and the ensembler is built from cfg in the policy __init__, so
        # this MUST be set before make_policy. See modeling_act.ACTTemporalEnsembler.
        cfg.temporal_ensemble_coeff = temporal_ensemble_coeff
        cfg.n_action_steps = n_action_steps = 1
    elif n_action_steps is not None:
        cfg.n_action_steps = n_action_steps
    ds = load_dataset(root=dataset_root)                  # for feature shapes + stats
    policy = make_policy(cfg=cfg, ds_meta=ds.meta)
    if n_action_steps is not None:
        policy.config.n_action_steps = n_action_steps     # in case make_policy re-read it
        # chunk length is `horizon` on the DiT, `chunk_size` on ACT.
        chunk = getattr(policy.config, "horizon",
                        getattr(policy.config, "chunk_size", "?"))
        te = f", temporal_ensemble={temporal_ensemble_coeff}" if temporal_ensemble_coeff else ""
        print(f"[infer] n_action_steps -> {n_action_steps} (chunk {chunk}){te}")
    policy.eval()
    pre, post = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=ckpt,
        preprocessor_overrides={"device_processor": {"device": device}})
    return policy, pre, post


def _state(model, data, info, finger_qadr):
    # MUST match recorder._observe channel-for-channel (see recorder.STATE_NAMES):
    # ee_pos, rot6d, gripper_width(measured), gripper_cmd(commanded ctrl), joints.
    p, R = tcp_pose(model, data, info)
    rot6d = R[:, :2].T.reshape(6)
    gw = float(sum(data.qpos[a] for a in finger_qadr))
    gcmd = float(data.ctrl[info.gripper_act_id])          # commanded gripper (0..255)
    jp = data.qpos[info.arm_qpos_ids]
    return np.concatenate([p, rot6d, [gw], [gcmd], jp]).astype(np.float32)


def _images(renderer, seg_renderer, data, keep_geoms):
    """Build the camera-obs dict via the SAME render+mask path as the recorder
    (env.render_obs): target-highlighted, shadow-free masked view; clean wrist."""
    obs = {}
    for key, cam in CAMERAS:
        im = render_obs(renderer, seg_renderer, data, cam, keep_geoms)
        obs[key] = torch.from_numpy(im).permute(2, 0, 1).float().div(255)[None]
    return obs


def _integrate(ee_p, ee_R, action):
    """Apply base-frame EE-delta (Δpos, Δrot_rotvec, gripper) to the current pose."""
    dpos, drot, grip = action[:3], action[3:6], float(action[6])
    theta = np.linalg.norm(drot)
    Rd = matrix_exp3(drot / theta, theta) if theta > 1e-8 else np.eye(3)
    return ee_p + dpos, Rd @ ee_R, grip


class _VideoRec:
    """Capture a clean (unmasked) third-person render to an MP4 for a demo/README.
    Frames grabbed every `stride` sim steps so playback at `fps` is ~real-time."""
    def __init__(self, model, cam="scene", fps=30, w=640, h=480):
        self.r = mujoco.Renderer(model, h, w)
        self.cam, self.fps, self.w, self.h = cam, fps, w, h
        self.stride = max(1, round((1.0 / fps) / model.opt.timestep))
        self.frames, self.i = [], 0

    def maybe(self, data):
        if self.i % self.stride == 0:
            self.r.update_scene(data, camera=self.cam)       # marker (group 4) hidden → clean
            self.frames.append(self.r.render().copy())
        self.i += 1

    def save(self, path, won):
        import cv2
        out = path.replace(".mp4", f"_{'ok' if won else 'fail'}.mp4")
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (self.w, self.h))
        for f in self.frames:
            vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))      # mujoco RGB → cv2 BGR
        vw.release()
        print(f"[infer] saved {out}  ({len(self.frames)} frames @ {self.fps}fps)")


def rollout(env, policy, pre, post, controller, renderer, seg_renderer, finger_qadr,
            device, max_decisions=160, sim_per_decision=33, on_step=None,
            record_path=None, record_cam="scene"):
    model, data, info = env.model, env.data, env.info
    instruction = env.reset()
    keep = env.target_keep_geoms()                           # highlight target obj+bin
    controller.reset(data)
    policy.reset()
    ref_p, ref_R = tcp_pose(model, data, info)               # running target reference
    rec = _VideoRec(model, record_cam) if record_path else None
    print(f"[infer] task: {instruction}")
    won = False
    for k in range(max_decisions):
        obs = _images(renderer, seg_renderer, data, keep)
        obs["observation.state"] = torch.from_numpy(
            _state(model, data, info, finger_qadr))[None]
        obs["task"] = [instruction]
        with torch.no_grad():
            action = post(policy.select_action(pre(obs)))
        action = action.squeeze(0).detach().cpu().numpy()
        # Integrate the delta onto the RUNNING REFERENCE, not the measured TCP:
        # the position servo trails its command by a constant lag, and rebuilding
        # the target from the measured (lagging) pose re-injects that lag every
        # decision so it compounds (→ 150 mm+ drift). Advancing an internal
        # reference keeps the arm at a constant, in-tolerance trail. See
        # pick_place/replay_gt.py (measured 0/6 vs reference 5/6).
        tp, tR, grip = _integrate(ref_p, ref_R, action)
        ref_p, ref_R = tp, tR
        cmd = TargetCommand(pos=tp, R=tR, gripper=grip)
        _update_marker(model, data, info, cmd)               # green marker = policy target
        for _ in range(sim_per_decision):
            controller.step(data, cmd)
            mujoco.mj_step(model, data)
            env.track()                        # lift height gates env.success() below
            if rec is not None:
                rec.maybe(data)
            if on_step:
                on_step()
        if env.success():
            print(f"[infer] SUCCESS at decision {k}")
            won = True
            break
    if not won:
        print("[infer] timeout — not placed")
    if rec is not None:
        rec.save(record_path, won)
    return won


def rollout_async(env, policy, pre, post, controller, renderer, seg_renderer, finger_qadr,
                  device, max_decisions=160, sim_per_decision=33, lookahead=2, on_step=None,
                  record_path=None, record_cam="scene"):
    """Same rollout, but inference runs in a background thread so it OVERLAPS sim
    stepping instead of freezing it (kills the move-stop-move stutter). A producer
    thread calls select_action to fill a small action queue; the main thread renders
    the latest obs, pops an action, and steps the sim. Torch inference and mj_step
    both release the GIL, so they genuinely run in parallel.

    `lookahead` = queue depth = how many actions the producer runs ahead. This is
    the accuracy/smoothness knob: SMALL (1-3) keeps each chunk planned from a nearly
    fresh observation (accuracy ≈ sync) and still hides most latency; LARGE runs a
    whole chunk ahead, so chunks get planned from chunk-stale observations and the
    policy overshoots the grasp. Default small on purpose.

    NOTE: torch inference from a non-main thread is reliable on CPU; on MPS it usually
    works but is less battle-tested — if it errors/hangs, drop --async-infer."""
    model, data, info = env.model, env.data, env.info
    instruction = env.reset()
    keep = env.target_keep_geoms()
    controller.reset(data)
    policy.reset()
    ref_p, ref_R = tcp_pose(model, data, info)
    rec = _VideoRec(model, record_cam) if record_path else None
    print(f"[infer] task: {instruction}  (async)")

    def observe():
        obs = _images(renderer, seg_renderer, data, keep)          # main thread only (GL ctx)
        obs["observation.state"] = torch.from_numpy(_state(model, data, info, finger_qadr))[None]
        obs["task"] = [instruction]
        return obs

    latest = {"obs": observe()}
    lock = threading.Lock()
    act_q = queue.Queue(maxsize=max(1, lookahead))    # small = fresh obs = accurate
    stop = threading.Event()
    err = {}

    def producer():
        try:
            while not stop.is_set():
                with lock:
                    obs = latest["obs"]
                with torch.no_grad():
                    a = post(policy.select_action(pre(obs))).squeeze(0).detach().cpu().numpy()
                while not stop.is_set():
                    try:
                        act_q.put(a, timeout=0.1); break
                    except queue.Full:
                        continue
        except Exception as e:                       # surface, don't hang the main loop
            err["e"] = e; stop.set()

    worker = threading.Thread(target=producer, daemon=True)
    worker.start()

    won = False
    try:
        for k in range(max_decisions):
            with lock:
                latest["obs"] = observe()            # keep the producer's obs fresh
            while True:                              # wait for an action (or producer death)
                try:
                    a = act_q.get(timeout=0.1); break
                except queue.Empty:
                    if stop.is_set():
                        raise RuntimeError(f"inference thread died: {err.get('e')}")
            tp, tR, grip = _integrate(ref_p, ref_R, a)
            ref_p, ref_R = tp, tR
            cmd = TargetCommand(pos=tp, R=tR, gripper=grip)
            _update_marker(model, data, info, cmd)
            for _ in range(sim_per_decision):
                controller.step(data, cmd)
                mujoco.mj_step(model, data)
                env.track()
                if rec is not None:
                    rec.maybe(data)
                if on_step:
                    on_step()
            if env.success():
                print(f"[infer] SUCCESS at decision {k}")
                won = True
                break
        else:
            print("[infer] timeout — not placed")
    finally:
        stop.set()
        worker.join(timeout=1.0)
    if rec is not None:
        rec.save(record_path, won)
    return won


def _snapshot(data):
    return dict(qpos=data.qpos.copy(), qvel=data.qvel.copy(),
                mocap_pos=data.mocap_pos.copy(), mocap_quat=data.mocap_quat.copy(),
                act=data.act.copy(), ctrl=data.ctrl.copy(), time=float(data.time))


def _restore(model, data, s):
    data.qpos[:] = s["qpos"]; data.qvel[:] = s["qvel"]
    data.mocap_pos[:] = s["mocap_pos"]; data.mocap_quat[:] = s["mocap_quat"]
    data.act[:] = s["act"]; data.ctrl[:] = s["ctrl"]; data.time = s["time"]
    mujoco.mj_forward(model, data)


def probe_language(env, policy, pre, post, controller, renderer, seg_renderer,
                   finger_qadr, decisions=40, sim_per_decision=33):
    """Grounding test: same scene, change ONLY which object is the target (its
    mask highlight + matching instruction). If the gripper follows the highlighted
    object, grounding works; if it always heads for the same one, it's ignoring
    the target signal."""
    model, data, info = env.model, env.data, env.info
    env.reset()
    env.target_bin = 0
    snap = _snapshot(data)
    obj_xy0 = [env.obj_pos(j)[:2].copy() for j in range(len(OBJECTS))]
    nears = []
    for i, (color, shape) in enumerate(OBJECTS):
        env.target_obj = i                                   # drives BOTH mask + text
        keep = env.target_keep_geoms()
        instr = f"put the {color} {shape} into the {BINS[env.target_bin]} bin"
        _restore(model, data, snap)
        controller.reset(data)
        policy.reset()
        ref_p, ref_R = tcp_pose(model, data, info)           # running target reference
        for k in range(decisions):
            obs = _images(renderer, seg_renderer, data, keep)
            obs["observation.state"] = torch.from_numpy(
                _state(model, data, info, finger_qadr))[None]
            obs["task"] = [instr]
            with torch.no_grad():
                action = post(policy.select_action(pre(obs)))
            action = action.squeeze(0).detach().cpu().numpy()
            tp, tR, grip = _integrate(ref_p, ref_R, action)  # onto reference, not measured
            ref_p, ref_R = tp, tR
            cmd = TargetCommand(pos=tp, R=tR, gripper=grip)
            for _ in range(sim_per_decision):
                controller.step(data, cmd)
                mujoco.mj_step(model, data)
        ee_xy = tcp_pose(model, data, info)[0][:2]
        # measure against ORIGINAL object positions (target may have been moved)
        dists = [float(np.linalg.norm(ee_xy - p)) for p in obj_xy0]
        nearest = int(np.argmin(dists))
        nears.append(nearest)
        print(f"[probe] target='{color} {shape}'  EE_xy={ee_xy.round(3)}  "
              f"dist_to_objs={[round(d, 3) for d in dists]}  "
              f"nearest=obj{nearest}({OBJECTS[nearest][0]} {OBJECTS[nearest][1]})  "
              f"{'<-- CORRECT' if nearest == i else '<-- WRONG'}")
    correct = sum(n == i for i, n in enumerate(nears))
    if len(set(nears)) == 1:
        print("[probe] VERDICT: same object for all 3 targets -> grounding IGNORED")
    else:
        print(f"[probe] VERDICT: target shifts behavior; {correct}/3 reached the "
              f"highlighted object")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=DEFAULT_CKPT)
    ap.add_argument("--device", default="mps")
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--max-decisions", type=int, default=160)
    ap.add_argument("--n-action-steps", type=int, default=8,
                    help="chunk actions to execute open-loop before re-observing "
                         "(trained default 24; 8 re-looks ~3x as often). Forced to "
                         "1 when --temporal-ensemble is set.")
    ap.add_argument("--temporal-ensemble", type=float, default=None,
                    metavar="COEFF",
                    help="ACT temporal ensembling coefficient (paper default 0.01). "
                         "Re-queries every decision and EMA-blends overlapping chunk "
                         "predictions; smoother/more accurate, no retraining. ACT only.")
    ap.add_argument("--scheduler", choices=["DDPM", "DDIM"], default=None,
                    help="diffusion only: override the inference sampler (same "
                         "weights). Checkpoint default here is DDIM/10 steps.")
    ap.add_argument("--num-inference-steps", type=int, default=None,
                    help="diffusion denoising steps (DDIM ~10; DDPM ~100 = "
                         "num_train_timesteps). Fewer = faster, rougher.")
    ap.add_argument("--async-infer", action="store_true",
                    help="run inference in a background thread so it overlaps sim "
                         "stepping (removes the move-stop stutter of slow samplers). "
                         "Reliable on CPU; usually works on MPS.")
    ap.add_argument("--async-lookahead", type=int, default=2,
                    help="async only: actions the producer runs ahead. Small (1-3) "
                         "= fresher obs = accuracy≈sync; larger = smoother but the "
                         "policy plans on staler obs and overshoots.")
    ap.add_argument("--record", action="store_true",
                    help="save a clean third-person MP4 per episode (for a README/demo); "
                         "filename gets _ok/_fail so you can grab a successful one.")
    ap.add_argument("--record-dir", default="outputs/infer_videos",
                    help="where to write the recorded MP4s")
    ap.add_argument("--record-cam", choices=["scene", "front", "wrist"], default="scene",
                    help="camera to record (scene = elevated 3/4 view; unmasked)")
    ap.add_argument("--viz", choices=["live", "none"], default="live")
    ap.add_argument("--probe-language", action="store_true",
                    help="same scene, vary only the target highlight; report "
                         "whether the policy reaches the highlighted object")
    ap.add_argument("--dataset-root", default=MASK_ROOT,
                    help="dataset root for feature shapes + norm stats (must match "
                         "the one the checkpoint was trained on)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    env = PickPlaceEnv(seed=args.seed)
    model, info = env.model, env.info
    controller = EEController(model, info)
    fj = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
          for n in ("finger_joint1", "finger_joint2")]
    finger_qadr = [model.jnt_qposadr[j] for j in fj if j >= 0]
    renderer = mujoco.Renderer(model, height=256, width=256)
    seg_renderer = mujoco.Renderer(model, height=256, width=256)
    seg_renderer.enable_segmentation_rendering()

    print(f"[infer] loading {args.checkpoint} on {args.device} …")
    policy, pre, post = load_policy(args.checkpoint, args.device, args.n_action_steps,
                                    dataset_root=args.dataset_root,
                                    temporal_ensemble_coeff=args.temporal_ensemble,
                                    noise_scheduler_type=args.scheduler,
                                    num_inference_steps=args.num_inference_steps)

    if args.probe_language:
        probe_language(env, policy, pre, post, controller, renderer, seg_renderer,
                       finger_qadr)
        return

    roll = rollout_async if args.async_infer else rollout
    extra = {"lookahead": args.async_lookahead} if args.async_infer else {}

    def run_all(on_step=None):
        wins = 0
        for ep in range(args.episodes):
            rp = os.path.join(args.record_dir, f"ep{ep:02d}.mp4") if args.record else None
            wins += roll(env, policy, pre, post, controller, renderer, seg_renderer,
                         finger_qadr, args.device, args.max_decisions,
                         on_step=on_step, record_path=rp, record_cam=args.record_cam, **extra)
        print(f"\n[infer] SUCCESS RATE: {wins}/{args.episodes}")

    if args.viz == "live":
        try:
            vc = mujoco.viewer.launch_passive(model, env.data)
        except RuntimeError as e:
            raise SystemExit(f"{e}\nUse mjpython for --viz live: .venv/bin/mjpython -m pick_place.run_infer")
        with vc as viewer:
            viewer.opt.geomgroup[4] = 1
            run_all(on_step=viewer.sync)
    else:
        run_all()


if __name__ == "__main__":
    main()
