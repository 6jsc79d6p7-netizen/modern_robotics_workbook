"""Ground-truth replay diagnostic — is the DEPLOY loop even capable of success?

The failure signature (great training loss, every architecture fails identically)
points below the model, at the shared action representation + deploy loop. This
isolates that: it feeds *perfect* (ground-truth) actions through the exact deploy
stack and asks whether they succeed.

Procedure, per episode:
  1. reset the scene; snapshot the initial sim state.
  2. run the scripted expert, capturing the SAME 15 Hz base-frame EE-delta stream
     the recorder would store (`_gt_delta`, identical to `Recorder._delta`).
  3. restore the initial state, then replay those deltas through the DEPLOY loop
     (`run_infer._integrate -> EEController -> mj_step`, sim_per_decision=33) —
     i.e. exactly `run_infer.rollout`, but actions come from the GT stream, not a
     policy.

Readouts:
  * replay success rate  — does the deploy stack, fed perfect actions, place it?
  * closed-loop tracking error — how well the controller follows GT targets.
  * open-loop integration drift — chain the deltas in pose-space only (no sim,
    no re-observation), like a policy executing a chunk blindly. Shows how far
    PERFECT deltas drift when integrated open-loop over a whole episode — the
    compounding-error term a policy also pays.

Interpretation:
  * replay FAILS  -> the deploy pipeline (IK / rate limiter / cadence / integrate)
    is broken; no policy can work on top of it. Fix here first.
  * replay SUCCEEDS -> pipeline is sound; the policy failure is learning /
    representation (covariate shift + proprioceptive shortcut), not the loop.

Run:  .venv/bin/python -m pick_place.replay_gt --episodes 5
"""
import argparse

import numpy as np
import mujoco

from .env import PickPlaceEnv
from .controller import EEController
from .expert import ScriptedExpert
from .scene import tcp_pose
from .target_source import TargetCommand
from .run_infer import _integrate, _snapshot, _restore   # the REAL deploy helpers
from mr.so3 import matrix_log3, matrix_exp3


def _reached(model, data, info, pos, tol=0.012):
    p, _ = tcp_pose(model, data, info)
    return np.linalg.norm(p - np.asarray(pos, float)) < tol


def _gt_delta(pose0, pose1, grip):
    """Base-frame EE delta, identical to recorder.Recorder._delta."""
    (p0, R0), (p1, R1) = pose0, pose1
    axis, theta = matrix_log3(R1 @ R0.T)
    return np.concatenate([p1 - p0, axis * theta, [grip]]).astype(np.float32)


def capture_expert(env, controller, expert, rng, record_every):
    """Run the expert; return the 15 Hz (pos, R, gripper_cmd) stream it produces —
    the exact frames the recorder samples."""
    model, data, info = env.model, env.data, env.info
    seq, gtick = [], 0
    for pos, R, grip, mn, mx in expert.plan(rng):
        cmd = TargetCommand(pos=np.asarray(pos, float), R=R, gripper=grip)
        for t in range(mx):
            controller.step(data, cmd)
            mujoco.mj_step(model, data)
            if gtick % record_every == 0:
                p, Rr = tcp_pose(model, data, info)
                seq.append((p.copy(), Rr.copy(), float(grip)))
            gtick += 1
            if t >= mn and _reached(model, data, info, pos):
                break
    return seq


def replay(env, controller, actions, sim_per_decision, mode="measured"):
    """run_infer.rollout, but actions come from the GT stream. Returns achieved
    TCP positions after each decision.

    mode='measured'  — integrate delta onto the MEASURED TCP each decision
                       (exactly what run_infer does today).
    mode='reference' — integrate delta onto a RUNNING internal reference pose
                       seeded at the start; the servo lag never feeds back in.
    """
    model, data, info = env.model, env.data, env.info
    achieved = []
    ref_p, ref_R = tcp_pose(model, data, info)          # reference seed
    for a in actions:
        if mode == "measured":
            base_p, base_R = tcp_pose(model, data, info)
        else:
            base_p, base_R = ref_p, ref_R
        tp, tR, grip = _integrate(base_p, base_R, a)
        ref_p, ref_R = tp, tR                            # advance the reference
        cmd = TargetCommand(pos=tp, R=tR, gripper=grip)
        for _ in range(sim_per_decision):
            controller.step(data, cmd)
            mujoco.mj_step(model, data)
        p, _ = tcp_pose(model, data, info)
        achieved.append(p.copy())
    return achieved


def open_loop_drift(seq, actions):
    """Chain the deltas in pose-space only (no sim, no re-observation) from the
    first GT pose, and report the endpoint gap vs the true final GT pose. This is
    the pure representation drift a policy pays over an open-loop chunk/episode."""
    p, R = seq[0][0].copy(), seq[0][1].copy()
    for a in actions:
        dpos, drot = a[:3], a[3:6]
        theta = np.linalg.norm(drot)
        Rd = matrix_exp3(drot / theta, theta) if theta > 1e-8 else np.eye(3)
        p, R = p + dpos, Rd @ R
    return float(np.linalg.norm(p - seq[-1][0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--fps", type=int, default=15)
    args = ap.parse_args()

    env = PickPlaceEnv(seed=args.seed)
    model = env.model
    controller = EEController(model, env.info)
    expert = ScriptedExpert(env)
    rng = np.random.default_rng(args.seed)
    sim_per_decision = round((1.0 / model.opt.timestep) / args.fps)

    gt_ok = 0
    results = {"measured": 0, "reference": 0}
    for ep in range(args.episodes):
        instr = env.reset()
        controller.reset(env.data)
        snap = _snapshot(env.data)

        seq = capture_expert(env, controller, expert, rng, sim_per_decision)
        gt_success = env.success()
        gt_ok += gt_success

        if len(seq) < 2:
            print(f"ep {ep}: expert produced <2 frames — skipped ({instr})")
            continue
        actions = [_gt_delta(seq[i][:2], seq[i + 1][:2], seq[i][2])
                   for i in range(len(seq) - 1)]

        targets = np.array([seq[i + 1][0] for i in range(len(actions))])
        drift = open_loop_drift(seq, actions)
        line = (f"ep {ep}: frames={len(seq):3d}  GT={'OK ' if gt_success else 'FAIL'}"
                f"  openloop_drift={drift*1000:4.1f}mm")
        for mode in ("measured", "reference"):
            _restore(model, env.data, snap)
            controller.reset(env.data)
            achieved = replay(env, controller, actions, sim_per_decision, mode=mode)
            ok = env.success()
            results[mode] += ok
            err = np.linalg.norm(np.array(achieved) - targets, axis=1)
            line += (f"  |{mode[:3]}: {'OK ' if ok else 'FAIL'} "
                     f"track_mean={err.mean()*1000:5.1f}mm")
        print(line + f"   ({instr})")

    n = args.episodes
    print(f"\nGT expert success:        {gt_ok}/{n}")
    print(f"REPLAY (measured-pose):   {results['measured']}/{n}   <- what run_infer does now")
    print(f"REPLAY (running-ref):     {results['reference']}/{n}   <- proposed fix")


if __name__ == "__main__":
    main()
