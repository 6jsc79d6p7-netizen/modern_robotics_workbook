"""DWA-style local-planner figure for 13d §19.

From the current pose, fan out candidate (v, omega) arcs (the dynamic window),
forward-simulate each, mark colliding arcs red, and highlight the chosen arc
(green) that makes progress along the global path while clearing the obstacle.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch

plt.rcParams["font.size"] = 10

def rollout(v, w, T=2.6, dt=0.05, x0=(1.0, 3.0, 0.0)):
    x, y, th = x0
    xs, ys = [x], [y]
    for _ in range(int(T / dt)):
        x += v * np.cos(th) * dt
        y += v * np.sin(th) * dt
        th += w * dt
        xs.append(x); ys.append(y)
    return np.array(xs), np.array(ys)

fig, ax = plt.subplots(figsize=(11, 6.5))

# obstacle (newly seen / dynamic)
obs_c, obs_r = np.array([4.8, 3.15]), 0.75
robot_r = 0.28
ax.add_patch(Circle(obs_c, obs_r, facecolor="#37474f", edgecolor="k", zorder=4))
ax.text(obs_c[0], obs_c[1] - obs_r - 0.35, "newly-seen /\ndynamic obstacle",
        ha="center", va="top", fontsize=8.5, color="#37474f")

# global path (the suggestion) -> goal
gx = np.array([1.0, 3.0, 5.2, 7.0, 8.2])
gy = np.array([3.0, 3.2, 3.7, 4.0, 4.1])
ax.plot(gx, gy, "--", color="#7a5cc0", linewidth=2.2, zorder=3,
        label="global path (suggestion)")
ax.plot(8.2, 4.1, "*", color="#7a5cc0", markersize=20, zorder=5)
ax.text(8.2, 4.45, "goal", ha="center", color="#5a3fa0", fontsize=9.5)

# candidate arcs (the dynamic window): fixed-ish v, sweep omega
v = 1.25
best_w = 0.34
collide_label = free_label = best_label = True
for w in np.linspace(-0.85, 0.85, 15):
    xs, ys = rollout(v, w)
    d = np.hypot(xs - obs_c[0], ys - obs_c[1])
    hits = np.any(d < obs_r + robot_r)
    if abs(w - best_w) < 1e-6:
        continue  # draw the best one last, on top
    if hits:
        ax.plot(xs, ys, color="#d64545", linewidth=1.3, alpha=0.9, zorder=2,
                label="colliding → discard" if collide_label else None)
        collide_label = False
    else:
        ax.plot(xs, ys, color="#9aa3ad", linewidth=1.1, alpha=0.75, zorder=2,
                label="feasible candidate" if free_label else None)
        free_label = False

# best arc
xs, ys = rollout(v, best_w)
ax.plot(xs, ys, color="#2e8b57", linewidth=3.0, zorder=6,
        label="chosen (v, ω) → command")
ax.plot(xs[-1], ys[-1], "o", color="#2e8b57", markersize=7, zorder=6)

# robot pose
ax.add_patch(Circle((1.0, 3.0), robot_r, facecolor="#c62828", edgecolor="k",
                    zorder=7))
ax.annotate("robot @ current\nstate (x, v, ω)", xy=(1.0, 3.0),
            xytext=(0.7, 1.35), fontsize=9, ha="center",
            arrowprops=dict(arrowstyle="-|>", color="#c62828"))

ax.text(2.15, 4.85,
        "dynamic window = the (v, ω) reachable\n"
        "in the next instant, given accel limits",
        fontsize=8.7, color="#444", style="italic",
        bbox=dict(boxstyle="round,pad=0.3", fc="#f4f2ec", ec="#ccc"))

ax.set_xlim(0, 9)
ax.set_ylim(0.7, 5.4)
ax.set_aspect("equal")
ax.set_xticks([]); ax.set_yticks([])
ax.set_title("Local planner (DWA): sample reachable arcs → simulate → "
             "discard collisions → pick best",
             fontsize=11.5, fontweight="bold")
ax.legend(loc="lower right", fontsize=8.8, framealpha=0.95)

plt.tight_layout()
out = __file__.replace("gen_", "").replace(".py", ".png")
plt.savefig(out, dpi=140, bbox_inches="tight")
print("wrote", out)
