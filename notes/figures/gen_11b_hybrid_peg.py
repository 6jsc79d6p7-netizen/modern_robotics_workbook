"""11b figure: hybrid motion-force control, peg-in-hole.

Task space splits into directions you POSITION-control and directions you
FORCE-control. For a peg sliding down a hole:
  - along the hole (z): MOTION controlled  -> command a downward velocity/position
  - sideways / rotation (x,y): FORCE controlled -> command ~0 force so the peg
    self-aligns and doesn't fight the walls
You cannot control both position AND force in the same direction at once.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow

fig, ax = plt.subplots(figsize=(6.2, 5.6))

# hole walls (two gray blocks)
ax.add_patch(Rectangle((-2.2, -3), 1.5, 4.5, color="#b0b7bd"))
ax.add_patch(Rectangle((0.7, -3), 1.5, 4.5, color="#b0b7bd"))
# peg
ax.add_patch(Rectangle((-0.6, -1.2, ), 1.2, 3.2, color="#34495e"))
ax.text(0, 0.6, "peg", color="white", ha="center", fontsize=11, weight="bold")

# MOTION-controlled axis (down the hole)
ax.annotate("", xy=(0, -2.6), xytext=(0, -1.4),
            arrowprops=dict(arrowstyle="-|>", color="#2980b9", lw=3))
ax.text(0.15, -2.3, "MOTION controlled\n(command downward\nvelocity/position)",
        color="#2980b9", fontsize=9, ha="left", va="center")

# FORCE-controlled axes (sideways, keep ~0 contact force)
for sx in (-1, 1):
    ax.annotate("", xy=(sx*0.9, 0.2), xytext=(sx*0.6, 0.2),
                arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=2.5))
ax.text(0, 2.5, "FORCE controlled sideways: command ~0 force\n"
                "→ peg self-aligns, doesn't jam against walls",
        color="#c0392b", fontsize=9, ha="center", va="center")

ax.text(-3.4, -2.6, "Rule: in any one direction you get\n"
                    "EITHER position OR force — never both.\n"
                    "Partition the task by which you need.",
        fontsize=8.5, ha="left", va="center",
        bbox=dict(boxstyle="round", fc="#fdf6e3", ec="#b58900"))

ax.set_xlim(-3.6, 3.2)
ax.set_ylim(-3.2, 3.3)
ax.set_aspect("equal")
ax.axis("off")
ax.set_title("Hybrid motion–force control: peg-in-hole", fontsize=12)

fig.tight_layout()
fig.savefig(__file__.replace("gen_", "").replace(".py", ".png"), dpi=130)
print("wrote", __file__.replace("gen_", "").replace(".py", ".png"))
