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
                num_inference_steps=None, num_integration_steps=None, fp16=False):
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
    # flow-matching-only (multi_task_dit objective=flow_matching): the sampler
    # takes `num_integration_steps` euler steps, each a FULL denoiser forward pass.
    # Trained default 100 → ~590 ms/chunk on MPS and it dominates deploy latency;
    # flow matching stays accurate at ~10-20 steps (100→10 ≈ 4.7x faster, ~126 ms).
    # Read at sample time from config, so this is a pure deploy knob, no retrain.
    if num_integration_steps is not None and hasattr(cfg, "num_integration_steps"):
        trained_steps = cfg.num_integration_steps
        cfg.num_integration_steps = num_integration_steps
        print(f"[infer] num_integration_steps -> {num_integration_steps} "
              f"(flow matching; trained {trained_steps})")
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
    if num_integration_steps is not None and hasattr(policy.config, "num_integration_steps"):
        policy.config.num_integration_steps = num_integration_steps   # in case make_policy re-read it
    if n_action_steps is not None:
        policy.config.n_action_steps = n_action_steps     # in case make_policy re-read it
        # chunk length is `horizon` on the DiT, `chunk_size` on ACT.
        chunk = getattr(policy.config, "horizon",
                        getattr(policy.config, "chunk_size", "?"))
        te = f", temporal_ensemble={temporal_ensemble_coeff}" if temporal_ensemble_coeff else ""
        print(f"[infer] n_action_steps -> {n_action_steps} (chunk {chunk}){te}")
    policy.eval()
    if fp16:
        # Run the denoiser/encoder forward passes in fp16 (the DP UNet is 256M
        # params → compute-bound, ~1.4x on MPS). autocast keeps the scheduler
        # math + norm stats in fp32; we cast the action back to fp32 so the
        # (fp32) postprocessor un-normalizer matches. Deploy knob, no retrain.
        dev_type = "cuda" if str(device).startswith("cuda") else str(device)
        _orig_select = policy.select_action

        def _select_fp16(batch, *a, **k):
            with torch.autocast(device_type=dev_type, dtype=torch.float16):
                out = _orig_select(batch, *a, **k)
            return out.float() if torch.is_tensor(out) else out

        policy.select_action = _select_fp16
        print(f"[infer] fp16 autocast on ({dev_type})")
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


def _gen_chunk(policy, pre, post, obs_list):
    """Generate ONE full action chunk (un-normalized, base-frame EE-deltas).

    `obs_list` is the last `n_obs_steps` *consecutive* observations (one decision
    apart). We feed them through the policy's own obs-history queue exactly like
    select_action does (pop ACTION so populate_queues doesn't push a None into the
    action queue; stack the per-camera images into OBS_IMAGES), then call
    predict_action_chunk to sample the whole chunk in one shot. Returns (L, adim)
    where L = horizon - (n_obs_steps-1) *iff* the caller widened
    policy.config.n_action_steps to the horizon (rollout_chunked does), so there's
    an overlap tail beyond the executed actions for the RTC boundary blend.

    Only the producer thread calls this, so it is the sole owner of policy._queues.
    """
    from lerobot.utils.constants import ACTION, OBS_IMAGES
    from lerobot.policies.utils import populate_queues
    b = None
    for ob in obs_list:
        b = dict(pre(ob))
        b.pop(ACTION, None)
        if policy.config.image_features:
            b[OBS_IMAGES] = torch.stack(
                [b[k] for k in policy.config.image_features], dim=-4)
        policy._queues = populate_queues(policy._queues, b)
    chunk = policy.predict_action_chunk(b)                    # (1, L, adim) normalized
    return np.stack([post(chunk[:, t]).squeeze(0).float().detach().cpu().numpy()
                     for t in range(chunk.shape[1])])


def rollout_chunked(env, policy, pre, post, controller, renderer, seg_renderer, finger_qadr,
                    device, max_decisions=160, sim_per_decision=33, rtc_overlap=0,
                    on_step=None, record_path=None, record_cam="scene"):
    """Chunk-granularity pipeline (the right async design when sample ≫ per-decision
    sim, i.e. the ratio that makes the per-action `--async-infer` need lookahead ≈ a
    whole chunk). While the arm executes the current chunk's `n_action_steps`
    actions, a background thread generates the NEXT full chunk from a fresh 2-frame
    observation taken at chunk start. Because one chunk executes in
    n_action_steps · T_dec while a sample takes ~one sample-time, the next chunk is
    ready before the current one is consumed → the sample latency is hidden with
    exactly ONE chunk of obs-staleness (deterministic, no jitter).

    Two wins vs the sync/per-action loops:
      1. renders only the `n_obs_steps` frames the policy actually consumes per
         chunk (sync renders every decision but evicts all but the last 2 before
         each generation — ~14/16 wasted).
      2. overlaps generation with execution on a background thread.

    `rtc_overlap` > 0 enables an RTC-style soft boundary: the first `rtc_overlap`
    actions of the new chunk are crossfaded with the overlapping tail the previous
    chunk already predicted (horizon > n_action_steps), ramping new-chunk weight
    0→1 so the fresh-but-stale plan doesn't snap the arm's velocity at the seam.
    """
    model, data, info = env.model, env.data, env.info
    instruction = env.reset()
    keep = env.target_keep_geoms()
    controller.reset(data)
    policy.reset()
    ref_p, ref_R = tcp_pose(model, data, info)
    rec = _VideoRec(model, record_cam) if record_path else None
    n_obs = int(policy.config.n_obs_steps)
    na = int(policy.config.n_action_steps)                   # actions to EXECUTE per chunk
    H = int(getattr(policy.config, "horizon",
                    getattr(policy.config, "chunk_size", na)))
    policy.config.n_action_steps = H                         # widen slice → keep the RTC tail
    print(f"[infer] task: {instruction}  (chunk-pipeline exec={na} horizon={H} "
          f"n_obs={n_obs} rtc_overlap={rtc_overlap})")

    def observe():
        obs = _images(renderer, seg_renderer, data, keep)
        obs["observation.state"] = torch.from_numpy(
            _state(model, data, info, finger_qadr))[None]
        obs["task"] = [instruction]
        return obs

    def exec_action(a):
        nonlocal ref_p, ref_R
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

    # background producer: sole owner of policy state (no races)
    req = {"obs": None}
    out = {"chunk": None}
    have_req, have_chunk, stop = threading.Event(), threading.Event(), threading.Event()
    err = {}

    def producer():
        try:
            while not stop.is_set():
                if not have_req.wait(timeout=0.1):
                    continue
                have_req.clear()
                with torch.no_grad():
                    out["chunk"] = _gen_chunk(policy, pre, post, req["obs"])
                have_chunk.set()
        except Exception as e:
            err["e"] = e
            stop.set()
            have_chunk.set()

    worker = threading.Thread(target=producer, daemon=True)
    worker.start()

    def request(obs_list):
        req["obs"] = obs_list
        have_chunk.clear()
        have_req.set()

    def await_chunk():
        while not have_chunk.wait(timeout=0.1):
            if stop.is_set():
                raise RuntimeError(f"producer died: {err.get('e')}")
        if out["chunk"] is None:
            raise RuntimeError(f"producer died: {err.get('e')}")
        return out["chunk"]

    # bootstrap: n_obs consecutive frames (hold pose so they differ by one decision) → first chunk
    boot = []
    for _ in range(n_obs):
        boot.append(observe())
        exec_action(np.zeros(7))                             # zero delta = hold; advances sim one decision
    request(boot)
    cur = await_chunk()

    won = False
    k = 0
    try:
        while k < max_decisions and not stop.is_set():
            obs_list, prev = [], cur
            for i in range(na):
                if k >= max_decisions:
                    break
                if i < n_obs:                                # render only the frames the policy consumes
                    obs_list.append(observe())
                exec_action(cur[i])
                k += 1
                if i == n_obs - 1:                           # kick next chunk from fresh consecutive frames
                    request(obs_list)
                if env.success():
                    won = True
                    break
            if won or k >= max_decisions:
                break
            nxt = await_chunk()
            if rtc_overlap > 0:                              # RTC soft boundary
                m = min(rtc_overlap, len(nxt), max(0, len(prev) - na))
                for i in range(m):
                    w = (i + 1) / (m + 1)
                    nxt[i] = (1 - w) * prev[na + i] + w * nxt[i]
            cur = nxt
    finally:
        stop.set()
        worker.join(timeout=1.0)
        policy.config.n_action_steps = na                    # restore (we widened it for the tail)
    if won:
        print(f"[infer] SUCCESS at decision {k - 1}")
    else:
        print("[infer] timeout — not placed")
    if rec is not None:
        rec.save(record_path, won)
    return won


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
    ap.add_argument("--num-integration-steps", type=int, default=None,
                    help="flow-matching (multi_task_dit) euler steps per chunk. "
                         "Trained default 100 ≈ 590 ms/chunk on MPS and dominates "
                         "latency; 10-20 is usually as accurate (100->10 ≈ 4.7x "
                         "faster). Deploy-time knob, no retraining.")
    ap.add_argument("--fp16", action="store_true",
                    help="run the policy forward passes in fp16 autocast "
                         "(~1.4x on MPS for the 256M-param DP UNet; scheduler + "
                         "norm stats stay fp32). Deploy knob, no retraining.")
    ap.add_argument("--async-infer", action="store_true",
                    help="run inference in a background thread so it overlaps sim "
                         "stepping (removes the move-stop stutter of slow samplers). "
                         "Reliable on CPU; usually works on MPS.")
    ap.add_argument("--async-lookahead", type=int, default=2,
                    help="async only: actions the producer runs ahead. Small (1-3) "
                         "= fresher obs = accuracy≈sync; larger = smoother but the "
                         "policy plans on staler obs and overshoots.")
    ap.add_argument("--chunk-pipeline", action="store_true",
                    help="chunk-granularity async: generate the NEXT full chunk on a "
                         "bg thread while executing the current one (one chunk of "
                         "obs-staleness, no jitter). Renders only n_obs_steps frames "
                         "per chunk. Right design when sample-time ≈ chunk-exec-time.")
    ap.add_argument("--rtc-overlap", type=int, default=0, metavar="N",
                    help="chunk-pipeline only: RTC soft boundary — crossfade the first "
                         "N actions of each new chunk with the previous chunk's "
                         "overlapping tail (needs horizon>n_action_steps). 0 = off.")
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
                                    num_inference_steps=args.num_inference_steps,
                                    num_integration_steps=args.num_integration_steps,
                                    fp16=args.fp16)

    if args.probe_language:
        probe_language(env, policy, pre, post, controller, renderer, seg_renderer,
                       finger_qadr)
        return

    if args.chunk_pipeline:
        roll = rollout_chunked
        extra = {"rtc_overlap": args.rtc_overlap}
    elif args.async_infer:
        roll = rollout_async
        extra = {"lookahead": args.async_lookahead}
    else:
        roll = rollout
        extra = {}

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
