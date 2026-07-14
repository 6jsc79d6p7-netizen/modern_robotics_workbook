"""Generate figures for notes/03c_wrenches.md. Run with the project .venv."""
import numpy as np
import matplotlib.pyplot as plt

OUT = __file__.rsplit("/", 1)[0]

# ---------------------------------------------------------------------------
# Figure: a force f at point r produces a moment m = r x f about the origin.
# 2D picture (in the x-y plane) -- the lever-arm intuition for wrenches.
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7, 6))

origin = np.array([0, 0])
r = np.array([3.0, 1.0])          # point where the force is applied
f = np.array([0.0, -2.0])         # downward force (like gravity on a held object)

# the body: a little blob at r
ax.plot(*r, "o", color="tab:gray", markersize=22, zorder=3)
ax.text(r[0] + 0.15, r[1] + 0.25, "point r\n(application)", fontsize=10)

# position vector r (lever arm) from origin
ax.annotate("", xy=r, xytext=origin,
            arrowprops=dict(arrowstyle="->", color="tab:blue", lw=2))
ax.text(r[0] / 2 - 0.5, r[1] / 2 + 0.15, "r", color="tab:blue", fontsize=13)

# force f at r
ax.annotate("", xy=r + f, xytext=r,
            arrowprops=dict(arrowstyle="->", color="tab:red", lw=2.5))
ax.text(r[0] + 0.15, r[1] + f[1] / 2, "f", color="tab:red", fontsize=13)

# the {s} frame at the origin
ax.annotate("", xy=(1, 0), xytext=origin,
            arrowprops=dict(arrowstyle="->", color="black", lw=1.5))
ax.annotate("", xy=(0, 1), xytext=origin,
            arrowprops=dict(arrowstyle="->", color="black", lw=1.5))
ax.text(1.05, -0.25, "x_s", fontsize=10)
ax.text(-0.35, 1.05, "y_s", fontsize=10)
ax.plot(*origin, "o", color="black", markersize=5)

# moment m = r x f (out of / into page). Here r x f = 3*(-2) - 1*0 = -6 (into page)
m_z = r[0] * f[1] - r[1] * f[0]
spin = "clockwise (into page)" if m_z < 0 else "counter-clockwise (out of page)"
circ = plt.Circle(origin, 0.55, fill=False, color="tab:green", lw=2)
ax.add_patch(circ)
ax.annotate("", xy=(0.55 * np.cos(-0.5), 0.55 * np.sin(-0.5)),
            xytext=(0.55 * np.cos(0.5), 0.55 * np.sin(0.5)),
            arrowprops=dict(arrowstyle="->", color="tab:green", lw=2))
ax.text(-0.1, -0.95, f"m = r x f\n(m_z = {m_z:.0f}, {spin})",
        color="tab:green", fontsize=10, ha="center")

ax.set_xlim(-1.5, 4.5)
ax.set_ylim(-2.0, 2.5)
ax.set_aspect("equal")
ax.set_title("A wrench F = (m, f): force f at point r,\n"
             "plus the moment m = r x f it makes about the origin")
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(f"{OUT}/03c_wrench.png", dpi=130, bbox_inches="tight")
plt.close(fig)
print("wrote figure to", OUT)
