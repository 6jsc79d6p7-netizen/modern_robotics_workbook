"""Step 2 entry point: drive the Franka from the phone (WebXR teleop).

    # 1) validate the phone stream first (no sim, plain python):
    .venv/bin/python -m pick_place.run_teleop --validate
    #    open https://<LAN-IP>:8443/ on the phone, Start AR, move it,
    #    toggle clutch / gripper — watch the values print here.

    # 2) full teleop with the live viewer (macOS needs mjpython):
    .venv/bin/mjpython -m pick_place.run_teleop
"""
import argparse
import io
import time

import numpy as np
import mujoco
from PIL import Image

from .env import PickPlaceEnv
from .controller import EEController
from .teleop_source import TeleopSource
from .teleop.server import TeleopServer
from .haptics import CollisionMonitor
from .run import _update_marker

PORT = 8443
SUCCESS_HOLD = 150          # control ticks (~0.3 s) the target must stay in-bin
WRIST_EVERY = 16            # render wrist cam every N ticks (~30 Hz at 500 Hz loop)


def banner(server):
    print("=" * 60)
    print("  pick_place · phone teleop")
    print("-" * 60)
    print(f"  Phone (WebXR): https://{server.ip}:{server.port}/")
    print(f"  (this Mac)   : https://localhost:{server.port}/")
    print(f"  Wrist cam    : https://localhost:{server.port}/wrist   (open on the Mac)")
    print("  Accept the self-signed cert warning once, then Start AR.")
    print("  CLUTCH toggles motion; gripper slider opens/closes the hand.")
    print("-" * 60)
    print("  CALIBRATE ONCE: point the phone the way you want 'into the screen'")
    print("  and tap  ⟳ SET FWD.  Then CLUTCH and move — right→right, up→up,")
    print("  push forward→into the screen. Re-tap SET FWD anytime it feels off.")
    print("  Mapping is gravity-aligned + follows the sim view (orbit freely).")
    print("=" * 60)


def run_validate(server):
    """Print the incoming phone pose/buttons; no sim. Validate before mapping."""
    print("[validate] waiting for phone pose… (Ctrl-C to stop)")
    while True:
        m = server.latest()
        if m:
            p = np.array(m["p"])
            print(f"p={p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f} m  "
                  f"clutch={'ON ' if m.get('clutch') else 'off'}  "
                  f"grip={m.get('gripper', '-'):>3}  "
                  f"{'6-DoF' if m.get('tracked') else '3-DoF(no pos!)'}",
                  end="\r", flush=True)
        time.sleep(0.1)


class _WristStreamer:
    """Render the hand-mounted wrist cam and publish JPEGs to /wrist, **rotated
    so the control-forward direction points up** — the wrist image otherwise
    spins with the gripper yaw and inverts your controls. We roll it by
    φ = atan2(fh·camₓ, fh·cam_y), where fh is the control-forward from the live
    sim-view azimuth; then centre-crop to hide the rotation's black corners.
    Guarded: any offscreen-render failure disables the stream instead of killing
    teleop."""
    def __init__(self, model, info, server, render_px=420, crop_px=290):
        self.info = info
        self.server = server
        self.crop = crop_px
        self.wid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
        self.ok = True
        try:
            self.renderer = mujoco.Renderer(model, height=render_px, width=render_px)
        except Exception as e:
            print(f"[wrist] renderer unavailable ({e}); wrist view disabled")
            self.ok = False

    def push(self, data, azimuth):
        if not self.ok:
            return
        try:
            self.renderer.update_scene(data, camera="wrist")
            img = self.renderer.render()
            a = np.radians(azimuth)
            fh = np.array([np.cos(a), np.sin(a), 0.0])          # control-forward
            R = data.cam_xmat[self.wid].reshape(3, 3)
            phi = np.degrees(np.arctan2(float(fh @ R[:, 0]), float(fh @ R[:, 1])))
            im = Image.fromarray(img).rotate(phi, resample=Image.BILINEAR)
            o = (im.width - self.crop) // 2
            im = im.crop((o, o, o + self.crop, o + self.crop))
            buf = io.BytesIO()
            im.save(buf, "JPEG", quality=70)
            self.server.push_frame(buf.getvalue())
        except Exception as e:
            print(f"[wrist] render failed ({e}); wrist view disabled")
            self.ok = False


def _emit_haptics(server, events):
    for ev in events:
        if ev == "vibrate":                               # warning: arm↔env
            server.send_haptic("vibrate", pattern=[50, 40, 50])
        elif ev == "sound":                               # confirmation: grasp
            server.send_haptic("sound", freq=660, ms=90)


def run_teleop(server):
    import mujoco.viewer
    env = PickPlaceEnv()
    model, data, info = env.model, env.data, env.info
    instruction = env.reset()
    print(f'\n[task] {instruction}')
    controller = EEController(model, info)
    controller.reset(data)
    source = TeleopSource(server)
    source.reset(model, data, info)
    collisions = CollisionMonitor(model)
    success_ticks = 0
    wrist = _WristStreamer(model, info, server)
    try:
        viewer_cm = mujoco.viewer.launch_passive(model, data)
    except RuntimeError as e:
        raise SystemExit(f"{e}\nRun under mjpython: .venv/bin/mjpython -m pick_place.run_teleop")
    with viewer_cm as viewer:
        viewer.opt.geomgroup[4] = 1        # show the target marker third-person only
        tick = 0
        while viewer.is_running():
            tic = time.time()
            source.view_azimuth = viewer.cam.azimuth      # map matches on-screen view
            cmd = source(data.time, model, data, info)
            controller.step(data, cmd)
            _update_marker(model, data, info, cmd)
            mujoco.mj_step(model, data)
            _emit_haptics(server, collisions.poll(data))
            if tick % WRIST_EVERY == 0:
                wrist.push(data, viewer.cam.azimuth)
            tick += 1

            # auto-reset once the target object has settled in the target bin
            success_ticks = success_ticks + 1 if env.success() else 0
            if success_ticks >= SUCCESS_HOLD:
                server.send_haptic("sound", freq=880, ms=250)   # success chime
                instruction = env.reset()
                print(f'[task] done ✓  next: {instruction}')
                controller.reset(data)
                source.reset(model, data, info)
                success_ticks = 0

            viewer.sync()
            dt = model.opt.timestep - (time.time() - tic)
            if dt > 0:
                time.sleep(dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", action="store_true",
                    help="print the phone stream without running the sim")
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    server = TeleopServer(port=args.port)
    server.start()
    banner(server)
    try:
        if args.validate:
            run_validate(server)
        else:
            run_teleop(server)
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
