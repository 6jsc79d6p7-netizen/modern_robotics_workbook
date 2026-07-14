"""Chapter 13 mecanum base: forward kinematics (H), odometry (F=H^+), and drift.

We drive the base KINEMATICALLY with the maps from notes/13a & 13c:
    u   = H(0) V_b            wheel speeds from a commanded body twist
    V_b = F  Delta-theta      odometry: body twist from wheel-angle increments
and integrate the body twist with the planar SCREW exponential (note 13c, eq 13.35).

Ground truth integrates the *commanded* twist. Odometry integrates the twist
reconstructed from *noisy* (slipping) wheel readings -> the estimate drifts.

Outputs:
    mujoco/mecanum_demo.gif                  true base (blue) + odometry ghost (orange)
    mujoco/mecanum_base.png                  hero still
    notes/modern_robotics/figures/13c_mecanum_drift.png   true vs odometry xy-path (embed in note)

Run:  .venv/bin/python mujoco/mecanum_kinematics.py
"""
import os
import numpy as np
import mujoco
import imageio.v2 as imageio
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ----- geometry (matches mecanum_base.xml) -----
r = 0.05          # wheel radius
L = 0.30          # half length (center -> wheel along x_b)
W = 0.20          # half width  (center -> wheel along y_b)
Z = 0.05          # deck height

# ----- the wheel matrix H(0): u = H(0) V_b, V_b = (wz, vx, vy)  (note 13a, eq 13.10) -----
lw = L + W
H = (1.0 / r) * np.array([
    [-lw, 1.0, -1.0],
    [ lw, 1.0,  1.0],
    [ lw, 1.0, -1.0],
    [-lw, 1.0,  1.0],
])
# odometry map F = H^+ (Moore-Penrose pseudoinverse, note 13c eq 13.33)
F = np.linalg.pinv(H)


def delta_qb(twist_dt):
    """Planar screw exponential (note 13c, eq 13.35).
    twist_dt = (dw, dx, dy) = body twist * dt. Returns body-frame displacement
    (dphi, dxb, dyb). Reduces to straight-line motion as dw -> 0."""
    dw, vx, vy = twist_dt
    if abs(dw) < 1e-9:
        return np.array([0.0, vx, vy])
    dxb = (vx * np.sin(dw) + vy * (np.cos(dw) - 1.0)) / dw
    dyb = (vy * np.sin(dw) + vx * (1.0 - np.cos(dw))) / dw
    return np.array([dw, dxb, dyb])


def integrate(q, twist_dt):
    """Advance planar pose q=(phi,x,y) by a body twist*dt, rotating body delta to world."""
    phi, x, y = q
    dphi, dxb, dyb = delta_qb(twist_dt)
    c, s = np.cos(phi), np.sin(phi)
    x += c * dxb - s * dyb
    y += s * dxb + c * dyb
    return np.array([phi + dphi, x, y])


def build_trajectory(dt):
    """A body-twist command program (wz, vx, vy) that shows off all 3 DOF."""
    seg = []
    def hold(T, w, vx, vy):
        for _ in range(int(T / dt)):
            seg.append((w, vx, vy))
    hold(1.4, 0.0, 0.6, 0.0)     # drive forward
    hold(1.4, 0.0, 0.0, 0.6)     # strafe left  (the mecanum showcase)
    hold(1.4, 0.0, 0.45, 0.45)   # diagonal
    hold(2.2, 0.8, 0.5, 0.0)     # curve: forward + rotate (arc integration matters here)
    hold(1.4, 0.0, 0.0, -0.6)    # strafe right
    return seg


def set_free(model, data, jname, q, z=Z):
    """Set a free joint's qpos from planar pose q=(phi,x,y)."""
    adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)]
    phi, x, y = q
    data.qpos[adr:adr + 3] = [x, y, z]
    data.qpos[adr + 3:adr + 7] = [np.cos(phi / 2), 0, 0, np.sin(phi / 2)]


def set_hinge(model, data, jname, angle):
    adr = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)]
    data.qpos[adr] = angle


def main():
    # ---- sanity: F truly inverts the forward map (odometry recovers the twist) ----
    print("H(0) =\n", np.round(H, 2))
    print("F = H^+ =\n", np.round(F, 3))
    print("F @ H (should be I_3) =\n", np.round(F @ H, 6))
    twist = np.array([0.3, 0.5, -0.2])
    u = H @ twist
    print("round-trip  F(H V_b) - V_b =", np.round(F @ u - twist, 12), "\n")

    dt = 1.0 / 60.0
    cmds = build_trajectory(dt)
    rng = np.random.default_rng(0)

    q_true = np.array([0.0, 0.0, 0.0])   # (phi, x, y) ground truth
    q_odom = np.array([0.0, 0.0, 0.0])   # odometry estimate
    wheel_ang = np.zeros(4)              # accumulated wheel angles (for viz)

    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "mecanum_base.xml"))
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=760, width=1000)
    root_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "root")

    cam = mujoco.MjvCamera()
    cam.distance, cam.azimuth, cam.elevation = 3.4, 90, -55
    scene_opt = mujoco.MjvOption()
    scene_opt.frame = mujoco.mjtFrame.mjFRAME_SITE

    frames = []
    true_path, odom_path = [q_true[1:].copy()], [q_odom[1:].copy()]

    for k, (w, vx, vy) in enumerate(cmds):
        # --- ground truth: integrate the COMMANDED twist ---
        q_true = integrate(q_true, np.array([w, vx, vy]) * dt)

        # --- wheels: true driving speeds u = H V_b, accumulate angle for viz ---
        u = H @ np.array([w, vx, vy])
        wheel_ang += u * dt

        # --- odometry: mecanum wheels slip a lot -> noisy measured increments ---
        # multiplicative slip (random) + small systematic bias per wheel (miscalibration)
        slip = 1.0 + rng.normal(0.0, 0.03, size=4) + np.array([0.03, 0.0, -0.02, 0.0])
        dtheta_meas = u * dt * slip
        v_odom = F @ dtheta_meas                    # reconstructed body twist * dt
        q_odom = integrate(q_odom, v_odom)

        true_path.append(q_true[1:].copy())
        odom_path.append(q_odom[1:].copy())

        # --- render every 2nd step (~30 fps) ---
        if k % 2 == 0:
            set_free(model, data, "root", q_true)
            set_free(model, data, "ghost", q_odom)
            for i in range(4):
                set_hinge(model, data, f"w{i+1}", wheel_ang[i])
            mujoco.mj_forward(model, data)
            cam.lookat[:] = data.xpos[root_bid]
            renderer.update_scene(data, cam, scene_option=scene_opt)
            frames.append(renderer.render())

    gif = os.path.join(HERE, "mecanum_demo.gif")
    imageio.mimsave(gif, frames, fps=30, loop=0)
    imageio.imwrite(os.path.join(HERE, "mecanum_base.png"), frames[len(frames) // 3])
    print("wrote", gif)

    # ---- drift plot ----
    true_path = np.array(true_path)
    odom_path = np.array(odom_path)
    drift = np.linalg.norm(true_path[-1] - odom_path[-1])
    fig, ax = plt.subplots(figsize=(7.2, 6.4))
    ax.plot(true_path[:, 0], true_path[:, 1], color="#1d4ed8", lw=2.6, label="ground truth")
    ax.plot(odom_path[:, 0], odom_path[:, 1], color="#ea580c", lw=2.2, ls="--",
            label="odometry estimate")
    ax.scatter(*true_path[0], c="k", s=60, zorder=5, label="start")
    ax.scatter(*true_path[-1], c="#1d4ed8", s=70, zorder=5)
    ax.scatter(*odom_path[-1], c="#ea580c", s=70, zorder=5)
    ax.annotate(f"final drift = {drift*100:.1f} cm", xy=odom_path[-1],
                xytext=(odom_path[-1][0] + 0.15, odom_path[-1][1] - 0.25),
                fontsize=11, color="#7c2d12", fontweight="bold",
                arrowprops=dict(arrowstyle="->", color="0.4"))
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("Mecanum base: odometry drifts under wheel slip\n"
                 "(wheels → V_b=FΔθ → screw-integrate → accumulate)", fontsize=12, fontweight="bold")
    ax.legend(loc="best", fontsize=10); ax.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(ROOT, "notes", "modern_robotics", "figures", "13c_mecanum_drift.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("wrote", out)
    print(f"final ground truth (phi,x,y) = {np.round(q_true,3)}")
    print(f"final odometry     (phi,x,y) = {np.round(q_odom,3)}")
    print(f"final position drift = {drift*100:.2f} cm")


if __name__ == "__main__":
    main()
