"""Step 1 entry point: drive the Franka to follow a TargetSource.

    # live viewer (run on your Mac; needs a display):
    .venv/bin/python -m pick_place.run

    # headless self-test (no display): runs the loop, reports TCP tracking
    # error, writes pick_place/out/selftest.png
    .venv/bin/python -m pick_place.run --headless
"""
import argparse
import os
import time

import numpy as np
import mujoco

from .scene import build_scene, tcp_pose
from .controller import EEController
from .target_source import DummyOrbitSource

HERE = os.path.dirname(os.path.abspath(__file__))


def setup():
    model, info = build_scene()
    data = mujoco.MjData(model)
    data.qpos[:] = info.home_qpos
    data.ctrl[:] = info.home_ctrl
    mujoco.mj_forward(model, data)

    controller = EEController(model, info)
    controller.reset(data)
    source = DummyOrbitSource()
    source.reset(model, data, info)
    return model, data, info, controller, source


def _update_marker(model, data, info, cmd):
    """Move the green mocap marker to the commanded target pose."""
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, cmd.R.flatten())
    data.mocap_pos[info.target_mocap_id] = cmd.pos
    data.mocap_quat[info.target_mocap_id] = quat


def _tcp_errors(model, data, info, cmd):
    from mr.so3 import matrix_log3
    p, R = tcp_pose(model, data, info)
    pos_err = np.linalg.norm(cmd.pos - p)
    _, ang_err = matrix_log3(cmd.R @ R.T)
    return pos_err, ang_err


def run_live():
    import mujoco.viewer
    model, data, info, controller, source = setup()
    try:
        viewer_cm = mujoco.viewer.launch_passive(model, data)
    except RuntimeError as e:
        raise SystemExit(
            f"{e}\nOn macOS run the live viewer under mjpython:\n"
            "    .venv/bin/mjpython -m pick_place.run")
    with viewer_cm as viewer:
        viewer.opt.geomgroup[4] = 1        # show the target marker (hidden group)
        while viewer.is_running():
            tic = time.time()
            cmd = source(data.time, model, data, info)
            controller.step(data, cmd)
            _update_marker(model, data, info, cmd)
            mujoco.mj_step(model, data)
            viewer.sync()
            # real-time pacing
            dt = model.opt.timestep - (time.time() - tic)
            if dt > 0:
                time.sleep(dt)


def run_headless(duration=8.0):
    model, data, info, controller, source = setup()
    renderer = mujoco.Renderer(model, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.4, 0.0, 0.5]
    cam.distance, cam.azimuth, cam.elevation = 1.6, 130, -20

    n = int(duration / model.opt.timestep)
    pos_errs, ang_errs = [], []
    for _ in range(n):
        cmd = source(data.time, model, data, info)
        controller.step(data, cmd)
        _update_marker(model, data, info, cmd)
        mujoco.mj_step(model, data)
        # skip the first 0.5 s of settling before scoring tracking
        if data.time > 0.5:
            pe, ae = _tcp_errors(model, data, info, cmd)
            pos_errs.append(pe)
            ang_errs.append(ae)

    pos_errs, ang_errs = np.array(pos_errs), np.array(ang_errs)
    print(f"steps: {n}   sim time: {data.time:.2f}s")
    print(f"TCP position error  (m):   mean {pos_errs.mean()*1000:6.2f} mm   "
          f"max {pos_errs.max()*1000:6.2f} mm")
    print(f"TCP orientation err (rad): mean {ang_errs.mean():6.4f}      "
          f"max {ang_errs.max():6.4f}")

    outdir = os.path.join(HERE, "out")
    os.makedirs(outdir, exist_ok=True)
    renderer.update_scene(data, cam)
    img = renderer.render()
    import imageio
    path = os.path.join(outdir, "selftest.png")
    imageio.imwrite(path, img)
    print("wrote", path)

    ok = pos_errs.max() < 0.02 and ang_errs.max() < 0.1
    print("SELFTEST:", "PASS" if ok else "FAIL (tracking error too high)")
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true",
                    help="run without a viewer; report tracking error + PNG")
    ap.add_argument("--duration", type=float, default=8.0)
    args = ap.parse_args()
    if args.headless:
        run_headless(args.duration)
    else:
        run_live()
