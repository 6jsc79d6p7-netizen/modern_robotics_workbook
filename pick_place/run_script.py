"""Generate scripted-expert demos headlessly (Step 4).

    # measure success rate only (fast, no recording):
    .venv/bin/python -m pick_place.run_script --episodes 20 --dry-run

    # record N successful demos into a LeRobot dataset:
    .venv/bin/python -m pick_place.run_script --episodes 100

Runs with no viewer / no realtime pacing, so it's fast. Failed attempts are
auto-discarded; it keeps going until `--episodes` successes are saved.
"""
import argparse

import numpy as np
import mujoco

from .env import PickPlaceEnv
from .controller import EEController
from .scene import tcp_pose
from .target_source import TargetCommand
from .expert import ScriptedExpert


def _reached(model, data, info, pos, tol=0.012):
    p, _ = tcp_pose(model, data, info)
    return np.linalg.norm(p - np.asarray(pos, float)) < tol


def _run_plan(env, model, data, info, controller, plan, recorder, record_every, gtick):
    for pos, R, grip, mn, mx in plan:
        cmd = TargetCommand(pos=np.asarray(pos, float), R=R, gripper=grip)
        for t in range(mx):
            controller.step(data, cmd)
            mujoco.mj_step(model, data)
            env.track()                       # accumulate lift height for success()
            if recorder and gtick[0] % record_every == 0:
                recorder.record_step(data, grip)
            gtick[0] += 1
            if t >= mn and _reached(model, data, info, pos):
                break


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50, help="successful demos to save")
    ap.add_argument("--dry-run", action="store_true", help="no recording; just success rate")
    ap.add_argument("--repo-id", default="local/pick_place")
    ap.add_argument("--root", default=None)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-attempts", type=int, default=0, help="0 = until target met")
    args = ap.parse_args()

    env = PickPlaceEnv(seed=args.seed)
    model, data, info = env.model, env.data, env.info
    controller = EEController(model, info)
    expert = ScriptedExpert(env)
    rng = np.random.default_rng(args.seed)

    recorder = None
    record_every = 1
    if not args.dry_run:
        from .recorder import Recorder, DEFAULT_ROOT
        recorder = Recorder(env, repo_id=args.repo_id, root=args.root or DEFAULT_ROOT,
                            fps=args.fps)
        record_every = max(1, round((1.0 / model.opt.timestep) / args.fps))
        print(f"[rec] recording to {recorder.root} @ {args.fps} Hz")

    saved = attempts = 0
    cap = args.max_attempts or 10 ** 9
    try:
        while saved < args.episodes and attempts < cap:
            instruction = env.reset()
            controller.reset(data)
            if recorder:
                recorder.start_episode(instruction)
            _run_plan(env, model, data, info, controller, expert.plan(rng),
                      recorder, record_every, gtick=[0])
            attempts += 1
            # placed in the bin AND held cleanly the whole way (no empty grasp / no
            # mid-carry drop-that-luckily-landed-in-bin). Both are needed for a clean
            # demo; either alone let contamination through (see env.held_cleanly).
            placed, clean = env.success(), env.held_cleanly()
            ok = placed and clean
            if ok:
                saved += 1
                if recorder:
                    color, shape = env.env.obj_desc[env.target_obj]
                    recorder.save_episode(color, shape, env.env.bin_desc[env.target_bin],
                                          success=placed, grasp_verified=env.grasp_verified(),
                                          held_cleanly=clean, source="script")
            elif recorder:
                recorder.discard_episode()
            if attempts % 5 == 0 or ok:
                print(f"attempt {attempts:4d}  {'OK  ' if ok else 'fail'}  "
                      f"saved {saved}/{args.episodes}  ({instruction})")
    finally:
        if recorder:
            recorder.finalize()          # flush writers → dataset valid + loadable

    print(f"\nDONE: {saved} saved / {attempts} attempts  "
          f"(success rate {saved/max(attempts,1)*100:.0f}%)")


if __name__ == "__main__":
    main()
