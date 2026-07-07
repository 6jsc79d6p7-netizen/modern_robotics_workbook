"""Figures for notes/10_motion_planning.md.

Run with the project venv:
    ./.venv/bin/python notes/figures/gen_10_figures.py
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from pathlib import Path

rng = np.random.default_rng(2)
OUT = Path(__file__).parent


# ---------------------------------------------------------------------------
# Figure 1 (3 panels): (a) C-obstacle = grow obstacle by robot radius;
#                      (b) RRT tree from start to goal among obstacles;
#                      (c) potential field with a local minimum.
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(1, 3, figsize=(14, 4.4))

# --- (a) C-obstacle for a circular robot ---
ax = axes[0]
r_robot = 0.6
obs = Circle((0, 0), 1.0, fc="0.55", ec="k", zorder=2)
cobs = Circle((0, 0), 1.0 + r_robot, fc="none", ec="tab:red", ls="--", lw=2,
              zorder=3)
ax.add_patch(obs)
ax.add_patch(cobs)
# a robot touching the C-obstacle boundary
ax.add_patch(Circle((1.0 + r_robot, 0), r_robot, fc="none", ec="tab:blue",
                     lw=1.8))
ax.plot(1.0 + r_robot, 0, "x", color="tab:blue")
ax.text(0, 0, "obstacle", ha="center", va="center", fontsize=9, color="w")
ax.text(0, 1.0 + r_robot + 0.18, "C-obstacle\n(grown by robot radius)",
        ha="center", fontsize=8.5, color="tab:red")
ax.text(1.0 + r_robot, -0.5, "robot = a point\nat its center",
        ha="center", fontsize=8, color="tab:blue")
ax.set_title("(a) C-space obstacle:\nrobot shrinks to a point, obstacle grows")
ax.set_xlim(-2.2, 3.2)
ax.set_ylim(-2.4, 2.6)
ax.set_aspect("equal")
ax.axis("off")

# --- (b) RRT tree ---
ax = axes[1]
start = np.array([0.5, 0.5])
goal = np.array([9.0, 9.0])
obstacles = [Rectangle((3, 0), 1.2, 6, fc="0.55", ec="k"),
             Rectangle((6, 4), 1.2, 6, fc="0.55", ec="k")]
boxes = [(3, 0, 4.2, 6), (6, 4, 7.2, 10)]


def in_obs(p):
    for x0, y0, x1, y1 in boxes:
        if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
            return True
    return False


def seg_free(a, b, n=12):
    for t in np.linspace(0, 1, n):
        if in_obs(a + t * (b - a)):
            return False
    return True


nodes = [start]
parent = [-1]
for _ in range(1400):
    samp = goal if rng.random() < 0.1 else rng.uniform(0, 10, 2)
    d = [np.linalg.norm(samp - n) for n in nodes]
    i = int(np.argmin(d))
    near = nodes[i]
    direction = samp - near
    L = np.linalg.norm(direction)
    if L < 1e-6:
        continue
    new = near + 0.5 * direction / L
    if not (0 <= new[0] <= 10 and 0 <= new[1] <= 10):
        continue
    if seg_free(near, new):
        nodes.append(new)
        parent.append(i)
        if np.linalg.norm(new - goal) < 0.5:
            break

for ob in obstacles:
    ax.add_patch(ob)
for k, p in enumerate(nodes):
    if parent[k] >= 0:
        q = nodes[parent[k]]
        ax.plot([p[0], q[0]], [p[1], q[1]], color="tab:green", lw=0.5,
                alpha=0.7)
# trace path back from last node
k = len(nodes) - 1
path = [nodes[k]]
while parent[k] >= 0:
    k = parent[k]
    path.append(nodes[k])
path = np.array(path)
ax.plot(path[:, 0], path[:, 1], "tab:red", lw=2.2, label="path found")
ax.plot(*start, "go", ms=9)
ax.plot(*goal, "b*", ms=14)
ax.text(start[0], start[1] - 0.6, "start", fontsize=8)
ax.text(goal[0] - 1.4, goal[1] + 0.1, "goal", fontsize=8)
ax.set_title("(b) RRT: random tree grows through\nfree C-space toward the goal")
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_aspect("equal")
ax.legend(fontsize=8, loc="lower right")
ax.set_xticks([])
ax.set_yticks([])

# --- (c) potential field local minimum ---
ax = axes[2]
gx, gy = np.meshgrid(np.linspace(0, 10, 200), np.linspace(0, 10, 200))
goal2 = np.array([8.5, 8.5])
U_att = 0.06 * ((gx - goal2[0]) ** 2 + (gy - goal2[1]) ** 2)
U_rep = np.zeros_like(gx)
for cx, cy in [(4, 4), (5.3, 5.3), (3, 5.5), (5.5, 3)]:
    d2 = (gx - cx) ** 2 + (gy - cy) ** 2
    U_rep += 3.0 / (d2 + 0.4)
U = U_att + U_rep
cs = ax.contourf(gx, gy, U, levels=30, cmap="viridis")
ax.plot(*goal2, "w*", ms=14)
ax.text(goal2[0] - 1.5, goal2[1] + 0.2, "goal", color="w", fontsize=8)
ax.plot(1.0, 1.0, "ro", ms=8)
ax.annotate("robot rolls\n'downhill'...", (1.0, 1.0), (1.3, 2.6),
            color="w", fontsize=8,
            arrowprops=dict(arrowstyle="->", color="w"))
ax.text(4.2, 4.4, "...but can get\nTRAPPED in a\nlocal minimum",
        color="w", fontsize=8, ha="center")
ax.set_title("(c) Potential field: pulled to goal,\npushed off obstacles "
             "— local minima trap")
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_aspect("equal")
ax.set_xticks([])
ax.set_yticks([])

fig.tight_layout()
fig.savefig(OUT / "10_planning_overview.png", dpi=125, bbox_inches="tight")
print("wrote 10_planning_overview.png")
