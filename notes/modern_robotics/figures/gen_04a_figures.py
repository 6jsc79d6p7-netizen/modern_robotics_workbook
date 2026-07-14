"""Generate figures for notes/04a_forward_kinematics_space.md. Run with project .venv."""
import numpy as np
import matplotlib.pyplot as plt

OUT = __file__.rsplit("/", 1)[0]

L = [1.0, 1.0, 1.0]  # link lengths


def fk_planar(thetas):
    """Return the joint pixel positions (base, j2, j3, tip) for a 3R planar arm."""
    pts = [np.array([0.0, 0.0])]
    ang = 0.0
    for i, th in enumerate(thetas):
        ang += th
        pts.append(pts[-1] + L[i] * np.array([np.cos(ang), np.sin(ang)]))
    return pts


def draw_arm(ax, thetas, color, label, joints=True):
    pts = fk_planar(thetas)
    xs, ys = zip(*pts)
    ax.plot(xs, ys, "-", color=color, lw=4, solid_capstyle="round",
            label=label, zorder=2)
    if joints:
        for i, p in enumerate(pts[:-1]):
            ax.plot(*p, "o", color="white", mec=color, mew=2, ms=12, zorder=3)
            ax.text(p[0], p[1] + 0.12, f"$\\theta_{i+1}$", color=color,
                    fontsize=11, ha="center")
    # end-effector frame (x red, y green) at the tip
    tip = pts[-1]
    ang = sum(thetas)
    R = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    for vec, c in [(R[:, 0], "tab:red"), (R[:, 1], "tab:green")]:
        ax.annotate("", xy=tip + 0.4 * vec, xytext=tip,
                    arrowprops=dict(arrowstyle="->", color=c, lw=2))


fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))

# --- Panel 1: home position (all theta = 0), screw axes labelled -----------
ax = axes[0]
draw_arm(ax, [0, 0, 0], "tab:blue", "home ($\\theta=0$)")
# mark the joint-axis points q_i (all on the x-axis), out-of-page rotation
for i, qx in enumerate([0.0, 1.0, 2.0]):
    ax.plot(qx, 0, "x", color="black", ms=9, mew=2)
    ax.text(qx, -0.28, f"$q_{i+1}=({qx:.0f},0)$", ha="center", fontsize=9)
# end-effector home position
ax.text(3.0, 0.25, "M: tip at $(L_1{+}L_2{+}L_3,0)$\nframe aligned with $\\{s\\}$",
        fontsize=9)
ax.text(0.05, 1.15, r"all joints: $\hat\omega_i=\hat z$ (out of page)"
                    "\n" r"$v_i=-\omega_i\times q_i$", fontsize=10,
        bbox=dict(boxstyle="round", fc="w", ec="0.7"))
ax.set_title("Home position — read off each joint's screw axis $S_i$ in $\\{s\\}$")

# --- Panel 2: a bent configuration -----------------------------------------
ax = axes[1]
draw_arm(ax, [0, 0, 0], "0.8", "home", joints=False)
thetas = [np.deg2rad(60), np.deg2rad(-40), np.deg2rad(70)]
draw_arm(ax, thetas, "tab:purple", r"$\theta=(60°,-40°,70°)$")
ax.set_title(r"$T(\theta)=e^{[S_1]\theta_1}e^{[S_2]\theta_2}e^{[S_3]\theta_3}M$")

for ax in axes:
    ax.axhline(0, color="0.85", lw=0.8, zorder=0)
    ax.axvline(0, color="0.85", lw=0.8, zorder=0)
    ax.set_aspect("equal")
    ax.set_xlim(-1.0, 3.6)
    ax.set_ylim(-1.6, 2.2)
    ax.set_xlabel("$x_s$"); ax.set_ylabel("$y_s$")
    ax.legend(loc="lower left", fontsize=9)
    ax.grid(alpha=0.15)

fig.suptitle("3R planar arm: forward kinematics by Product of Exponentials",
             fontsize=13, fontweight="bold")
fig.tight_layout()
fig.savefig(f"{OUT}/04a_3R_planar.png", dpi=130, bbox_inches="tight")
plt.close(fig)
print("wrote figure to", OUT)
