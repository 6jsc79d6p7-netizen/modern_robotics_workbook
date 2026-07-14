"""Generate figures for notes/03b_rigid_motions_twists.md. Run with the project .venv."""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

OUT = __file__.rsplit("/", 1)[0]


def draw_frame(ax, R, origin=(0, 0, 0), colors=("tab:red", "tab:green", "tab:blue"),
                labels=("x", "y", "z"), style="-", lw=2.5, alpha=1.0, length=1.0):
    o = np.array(origin, dtype=float)
    for i in range(3):
        axis = R[:, i] * length
        ax.quiver(*o, *axis, color=colors[i], linewidth=lw, linestyle=style,
                  alpha=alpha, arrow_length_ratio=0.15)
        tip = o + axis * 1.15
        ax.text(*tip, labels[i], color=colors[i], fontsize=11, fontweight="bold")


def style_axes(ax, xlim, ylim, zlim, title=""):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    ax.set_box_aspect([1, 1, 1])


def vec_to_so3(w):
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0],
    ])


def matrix_exp3(w_hat, theta):
    w = vec_to_so3(w_hat)
    return np.eye(3) + np.sin(theta) * w + (1 - np.cos(theta)) * (w @ w)


def G(w_hat, theta):
    w = vec_to_so3(w_hat)
    return np.eye(3) * theta + (1 - np.cos(theta)) * w + (theta - np.sin(theta)) * (w @ w)


# ---------------------------------------------------------------------------
# Figure 1: T_sb = a translated AND rotated body frame {b}
# ---------------------------------------------------------------------------
theta = np.pi / 2
R_sb = matrix_exp3(np.array([0, 0, 1.0]), theta)  # 90 deg about z, same as 3a
p_sb = np.array([2.0, 1.5, 0.8])

fig = plt.figure(figsize=(7, 6.5))
ax = fig.add_subplot(1, 1, 1, projection="3d")

# space frame {s} at the origin
draw_frame(ax, np.eye(3), labels=("x_s", "y_s", "z_s"), style="-", lw=2)

# arrow for p_sb: origin of {s} -> origin of {b}
ax.quiver(0, 0, 0, *p_sb, color="gray", linewidth=2, linestyle="--", arrow_length_ratio=0.06)
mid = p_sb * 0.5
ax.text(*mid, "  p_sb", color="gray", fontsize=11)

# body frame {b}, rotated AND translated
draw_frame(ax, R_sb, origin=p_sb, labels=("x_b", "y_b", "z_b"), style="-", lw=3)

style_axes(ax, (-0.5, 3.2), (-0.5, 3.2), (-0.5, 2.2),
           title=r"$T_{sb}=(R_{sb},\,p_{sb})$: {b} is {s} rotated by $R_{sb}$"
                 "\nand shifted by $p_{sb}$")
ax.view_init(elev=20, azim=-60)
fig.tight_layout()
fig.savefig(f"{OUT}/03b_transform_frame.png", dpi=130, bbox_inches="tight")
plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: a screw motion -- exp([S]theta) traces a helix around the axis
# ---------------------------------------------------------------------------
omega = np.array([0.0, 0.0, 1.0])           # axis direction (unit)
q = np.array([1.0, 0.0, 0.0])               # a point on the axis
h = 0.3                                       # pitch
v = -np.cross(omega, q) + h * omega          # screw axis linear part

thetas = np.linspace(0, 4 * np.pi, 400)
path = np.array([G(omega, t) @ v for t in thetas])

fig = plt.figure(figsize=(7.5, 6.5))
ax = fig.add_subplot(1, 1, 1, projection="3d")

# the screw axis itself (a vertical line through q, direction omega)
ax_t = np.linspace(-0.5, 3.0, 10)
axis_pts = q[None, :] + ax_t[:, None] * omega[None, :]
ax.plot(axis_pts[:, 0], axis_pts[:, 1], axis_pts[:, 2], color="tab:blue", lw=2,
        label="screw axis S")
ax.text(*(q + np.array([0, 0, 3.05])), r"$\hat{s}$", color="tab:blue", fontsize=12)

# helical path traced by the body-frame origin
ax.plot(path[:, 0], path[:, 1], path[:, 2], color="black", lw=2)

# a few frames along the way
for t, style, lw in [(0, "-", 2.5), (np.pi, "-", 2.5), (2 * np.pi, "-", 2.5), (3 * np.pi, "-", 2.5)]:
    p = G(omega, t) @ v
    R = matrix_exp3(omega, t)
    draw_frame(ax, R, origin=p, length=0.5, lw=lw,
               labels=("", "", "") if t != 0 else ("x_b", "y_b", "z_b"))

ax.text(*(path[0] + np.array([0.2, 0.2, -0.3])), r"$\theta=0$", fontsize=10)
ax.text(*(path[100] + np.array([0.2, 0.2, 0.0])), r"$\theta=\pi$", fontsize=10)
ax.text(*(path[200] + np.array([0.2, 0.2, 0.0])), r"$\theta=2\pi$", fontsize=10)

style_axes(ax, (-1.5, 1.5), (-1.5, 1.5), (-0.5, 3.0),
           title=r"A screw motion: $e^{[S]\theta}$ rotates about $\hat{s}$"
                 "\nwhile translating along it (helix)")
ax.view_init(elev=18, azim=-55)
fig.tight_layout()
fig.savefig(f"{OUT}/03b_screw_motion.png", dpi=130, bbox_inches="tight")
plt.close(fig)

print("wrote figures to", OUT)
