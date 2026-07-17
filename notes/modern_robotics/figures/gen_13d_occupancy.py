"""Occupancy-mapping figure for 13d §18.

Left  : ray-casting on a 2D grid — cells along each beam vote FREE, the hit cell
        votes OCCUPIED, everything beyond stays UNKNOWN. Rays are cast from the
        current *estimated pose*.
Right : log-odds as vote-tallying — a cell observed occupied x3 then free x1,
        with the probability read back through the sigmoid.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

plt.rcParams["font.size"] = 10

UNKNOWN, FREE, OCC = 0, 1, 2
COLS, ROWS = 13, 9
grid = np.full((ROWS, COLS), UNKNOWN, dtype=int)

# a vertical wall (occupied) at col 9, rows 1..7
wall_col = 9
wall_rows = range(1, 8)

sensor = np.array([1.5, 4.5])  # in cell coords (x=col, y=row)

# cast rays from sensor to each wall cell; mark traversed cells free, hit occ
targets = [(wall_col + 0.5, r + 0.5) for r in wall_rows]
rays = []
for tx, ty in targets:
    n = 200
    xs = np.linspace(sensor[0], tx, n)
    ys = np.linspace(sensor[1], ty, n)
    rays.append((tx, ty))
    for x, y in zip(xs, ys):
        ci, ri = int(x), int(y)
        if 0 <= ri < ROWS and 0 <= ci < COLS:
            if ci == wall_col:          # reached the wall -> occupied, stop
                grid[ri, ci] = OCC
                break
            if grid[ri, ci] == UNKNOWN:
                grid[ri, ci] = FREE
for r in wall_rows:
    grid[r, wall_col] = OCC

fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 6),
                               gridspec_kw={"width_ratios": [1.5, 1]})

# ---------- LEFT: ray-casting grid ----------
colors = {UNKNOWN: "#d9dce1", FREE: "#dbeaf7", OCC: "#37474f"}
for r in range(ROWS):
    for c in range(COLS):
        axL.add_patch(Rectangle((c, r), 1, 1, facecolor=colors[grid[r, c]],
                                edgecolor="white", linewidth=1.2))
# rays
for tx, ty in rays:
    axL.plot([sensor[0], tx], [sensor[1], ty], color="#e8792a",
             linewidth=0.9, alpha=0.7, zorder=3)
# sensor / robot pose
axL.add_patch(Circle(sensor, 0.28, facecolor="#c62828", edgecolor="k",
                     zorder=5))
axL.annotate("robot @ estimated\npose  T (SE3)", xy=sensor,
             xytext=(sensor[0] - 0.3, sensor[1] - 2.6),
             fontsize=9, ha="center",
             arrowprops=dict(arrowstyle="-|>", color="#c62828"))

axL.set_xlim(0, COLS)
axL.set_ylim(0, ROWS)
axL.set_aspect("equal")
axL.set_xticks([]); axL.set_yticks([])
axL.set_title("Ray-casting into the grid  (from the current pose)",
              fontsize=11, fontweight="bold")

# legend
leg = [(colors[FREE], "FREE — beam passed through"),
       (colors[OCC], "OCCUPIED — beam hit here"),
       (colors[UNKNOWN], "UNKNOWN — never observed")]
for i, (col, lab) in enumerate(leg):
    y = -0.9 - i * 0.7
    axL.add_patch(Rectangle((0.3, y), 0.6, 0.5, facecolor=col,
                            edgecolor="#888", clip_on=False))
    axL.text(1.1, y + 0.25, lab, va="center", fontsize=9)
axL.text(6.6, -0.65, "one beam  →  a whole column of FREE votes\n"
                     "+ one OCCUPIED vote at the hit; beyond = UNKNOWN",
         fontsize=8.5, va="center", ha="left", color="#444", style="italic")

# ---------- RIGHT: log-odds tally ----------
delta = 0.85
obs = ["occ", "occ", "occ", "free"]
l = 0.0
ls = [0.0]
for o in obs:
    l += delta if o == "occ" else -delta
    ls.append(l)
ps = [1 / (1 + np.exp(-v)) for v in ls]

xs = np.arange(len(ls))
axR.axhline(0, color="#bbb", linewidth=1)
axR.plot(xs, ls, "-o", color="#2e6f9e", linewidth=2, markersize=7, zorder=4)
for x, lv, pv in zip(xs, ls, ps):
    axR.annotate(f"p={pv:.2f}", (x, lv),
                 textcoords="offset points", xytext=(0, 12 if lv >= 0 else -16),
                 ha="center", fontsize=8.5, color="#204d6b")
labels = ["start\n(unknown)"] + obs
for x, lab in zip(xs, labels):
    col = "#c62828" if lab == "free" else ("#37474f" if lab == "occ" else "#666")
    axR.text(x, -3.05, lab, ha="center", fontsize=8.5, color=col)

axR.set_xlim(-0.4, len(ls) - 0.6)
axR.set_ylim(-3.2, 3.2)
axR.set_ylabel("log-odds  l = log(p / (1-p))")
axR.set_xticks([])
axR.set_title("Updating a cell = tallying votes\n"
              "occupied: l += Δ   free: l −= Δ   (Δ=0.85)",
              fontsize=10.5, fontweight="bold")
axR.text(0.05, 2.7, "read p back via the sigmoid:\n p = 1 / (1 + e^(−l))",
         fontsize=8.5, color="#204d6b",
         bbox=dict(boxstyle="round,pad=0.3", fc="#eef4f9", ec="#9cc"))
for s in ["top", "right"]:
    axR.spines[s].set_visible(False)

plt.tight_layout()
out = __file__.replace("gen_", "").replace(".py", ".png")
plt.savefig(out, dpi=140, bbox_inches="tight")
print("wrote", out)
