"""11b figure: converting a position controller into an admittance controller.

Top: a standard position controller — desired pose goes straight into the stiff
inner PD loop.
Bottom: admittance control — insert ONE block (the virtual M_d,B_d,K_d filter)
that reads the force sensor and shifts the setpoint. The inner loop is unchanged.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

fig, (axA, axB) = plt.subplots(2, 1, figsize=(9.6, 5.4))

def box(ax, x, y, w, h, text, fc, ec="#333", fs=9.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.05",
                                fc=fc, ec=ec, lw=1.6))
    ax.text(x + w/2, y + h/2, text, ha="center", va="center", fontsize=fs)

def arrow(ax, x0, y0, x1, y1, color="#333", lw=1.8):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>",
                                 mutation_scale=14, color=color, lw=lw))

for ax in (axA, axB):
    ax.set_xlim(0, 10); ax.set_ylim(0, 2.4); ax.axis("off")

# ---- Top: standard position controller ----
axA.set_title("Standard position controller", fontsize=11, loc="left")
axA.text(0.1, 1.2, r"$x_d$", fontsize=12, va="center")
arrow(axA, 0.55, 1.2, 3.2, 1.2)
box(axA, 3.2, 0.75, 3.0, 0.9, "stiff inner PD\n(IK → position actuators)", "#dfeaf5")
arrow(axA, 6.2, 1.2, 7.2, 1.2)
box(axA, 7.2, 0.8, 1.6, 0.8, "arm", "#e8e8e8")
axA.text(6.55, 1.42, r"$\tau$", fontsize=11)

# ---- Bottom: admittance controller ----
axB.set_title("Admittance controller  =  same loop  +  one filter block", fontsize=11, loc="left")
axB.text(0.1, 1.55, r"$x_d$", fontsize=12, va="center")
arrow(axB, 0.55, 1.55, 2.0, 1.55)
box(axB, 2.0, 1.05, 2.7, 1.0, "ADMITTANCE FILTER\n"
    r"$M_d\ddot d+B_d\dot d+K_d d=\mathcal{F}_{ext}$", "#fdefe0", ec="#c0722c", fs=8.6)
arrow(axB, 4.7, 1.55, 6.0, 1.55)
axB.text(5.15, 1.78, r"$x_r=x_d+d$", fontsize=9.5)
box(axB, 6.0, 1.05, 2.6, 1.0, "stiff inner PD\n(UNCHANGED)", "#dfeaf5", fs=9)
arrow(axB, 8.6, 1.55, 9.4, 1.55)
box(axB, 8.7, 0.15, 1.1, 0.6, "arm", "#e8e8e8", fs=9)
# arm is below-right; route torque down
arrow(axB, 9.0, 1.05, 9.0, 0.78)
# force feedback from arm up into the filter
axB.add_patch(FancyArrowPatch((8.7, 0.45), (3.35, 0.45), arrowstyle="-|>",
              mutation_scale=13, color="#c0392b", lw=1.7,
              connectionstyle="arc3,rad=0.0"))
arrow(axB, 3.35, 0.45, 3.35, 1.05, color="#c0392b", lw=1.7)
axB.text(5.8, 0.28, r"$\mathcal{F}_{ext}$  (wrist force/torque sensor)",
         fontsize=9, color="#c0392b", ha="center")

fig.tight_layout()
fig.savefig(__file__.replace("gen_", "").replace(".py", ".png"), dpi=130)
print("wrote", __file__.replace("gen_", "").replace(".py", ".png"))
