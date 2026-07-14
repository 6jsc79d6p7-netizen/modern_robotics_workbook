"""Generate figures for notes/03a_rotations.md. Run with the project .venv."""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

OUT = __file__.rsplit("/", 1)[0]


def draw_frame(ax, R, origin=(0, 0, 0), colors=("tab:red", "tab:green", "tab:blue"),
                labels=("x", "y", "z"), style="-", lw=2.5, alpha=1.0, length=1.0):
    o = np.array(origin)
    for i in range(3):
        axis = R[:, i] * length
        ax.quiver(*o, *axis, color=colors[i], linewidth=lw, linestyle=style,
                  alpha=alpha, arrow_length_ratio=0.15)
        tip = o + axis * 1.15
        ax.text(*tip, labels[i], color=colors[i], fontsize=12, fontweight="bold")


def style_axes(ax, lim=1.4, title=""):
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_xlabel("X (space)")
    ax.set_ylabel("Y (space)")
    ax.set_zlabel("Z (space)")
    ax.set_title(title)
    ax.set_box_aspect([1, 1, 1])


# ---------------------------------------------------------------------------
# Figure 1: {s} frame vs {b} frame after a 90deg rotation about space-z
# ---------------------------------------------------------------------------
theta = np.pi / 2
R_sb = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0,              0,             1],
])

fig = plt.figure(figsize=(12, 5.5))

ax1 = fig.add_subplot(1, 2, 1, projection="3d")
draw_frame(ax1, np.eye(3), labels=("x_s", "y_s", "z_s"), style="-", lw=2)
style_axes(ax1, title="Space frame {s}\n(reference, never moves)")

ax2 = fig.add_subplot(1, 2, 2, projection="3d")
# faint space frame for reference
draw_frame(ax2, np.eye(3), colors=("lightcoral", "lightgreen", "lightblue"),
           labels=("x_s", "y_s", "z_s"), style="--", lw=1.2, alpha=0.6)
# body frame, rotated
draw_frame(ax2, R_sb, labels=("x_b", "y_b", "z_b"), style="-", lw=3)
style_axes(ax2, title="Body frame {b} after R_sb\n(90° rotation about z_s)")
ax2.text(-1.3, -1.3, 1.3,
         "x_b -> +y_s\ny_b -> -x_s\nz_b -> z_s  (unchanged)",
         fontsize=10, family="monospace")

for ax in (ax1, ax2):
    ax.view_init(elev=22, azim=-60)

fig.suptitle(
    "R_sb's columns ARE the body axes, drawn in space coordinates",
    fontsize=13, fontweight="bold")
fig.tight_layout()
fig.savefig(f"{OUT}/03a_frame_rotation.png", dpi=130, bbox_inches="tight")
plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: angular velocity omega, a point p, and the cross product omega x p
# ---------------------------------------------------------------------------
fig = plt.figure(figsize=(7, 6.5))
ax = fig.add_subplot(1, 1, 1, projection="3d")

omega = np.array([0, 0, 1.3])          # spin axis = z, length ~ speed
p = np.array([1.0, 0.0, 0.3])          # a point on the body, off-axis
v = np.cross(omega, p)                  # velocity of that point = omega x p

ax.quiver(0, 0, 0, *omega, color="tab:blue", linewidth=3, arrow_length_ratio=0.12)
ax.text(*(omega * 1.1), r"$\omega$  (spin axis)", color="tab:blue", fontsize=12)

ax.quiver(0, 0, 0, *p, color="black", linewidth=2.5, arrow_length_ratio=0.15)
ax.text(*(p * 1.15), "p  (point on body)", color="black", fontsize=11)

ax.quiver(*p, *v, color="tab:red", linewidth=2.5, arrow_length_ratio=0.2)
ax.text(p[0] - 1.9, p[1] - 0.1, p[2] + v[2] + 0.5,
        r"$[\omega]p = \omega \times p$" + "\n(velocity of that point)",
        color="tab:red", fontsize=11)

# a faint circle showing the path that point travels (right-hand rule)
t = np.linspace(0, 1.7 * np.pi, 100)
r = np.linalg.norm(p[:2])
circ = np.stack([r * np.cos(t), r * np.sin(t), np.full_like(t, p[2])])
ax.plot(*circ, "--", color="gray", alpha=0.5, linewidth=1)

style_axes(ax, lim=2.0, title=r"$\omega \times p$ = velocity of a spinning point"
                              "\n(this is exactly what $[\\omega]$ computes)")
ax.view_init(elev=22, azim=-55)
fig.tight_layout()
fig.savefig(f"{OUT}/03a_angular_velocity.png", dpi=130, bbox_inches="tight")
plt.close(fig)

print("wrote figures to", OUT)
