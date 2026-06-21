"""Generate figures for notes/04b_forward_kinematics_body.md. Run with project .venv."""
import numpy as np
import matplotlib.pyplot as plt

OUT = __file__.rsplit("/", 1)[0]
L = 2.0

fig, ax = plt.subplots(figsize=(8.5, 5.2))

# 1R arm at home: link from base (0,0) to hand (L,0)
ax.plot([0, L], [0, 0], "-", color="tab:blue", lw=5, solid_capstyle="round", zorder=2)
ax.plot(0, 0, "o", color="white", mec="tab:blue", mew=2.5, ms=15, zorder=3)
ax.text(0, -0.32, "joint 1 axis\n(out of page)", ha="center", fontsize=9)

# space frame {s} at the base
ax.annotate("", xy=(0.8, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="black", lw=1.6))
ax.annotate("", xy=(0, 0.8), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="black", lw=1.6))
ax.text(0.83, -0.18, "$x_s$", fontsize=10); ax.text(-0.22, 0.82, "$y_s$", fontsize=10)

# body frame {b} at the hand (aligned with {s} at home)
ax.annotate("", xy=(L + 0.8, 0), xytext=(L, 0),
            arrowprops=dict(arrowstyle="->", color="tab:red", lw=1.8))
ax.annotate("", xy=(L, 0.8), xytext=(L, 0),
            arrowprops=dict(arrowstyle="->", color="tab:green", lw=1.8))
ax.text(L + 0.83, -0.18, "$x_b$", color="tab:red", fontsize=10)
ax.text(L - 0.22, 0.82, "$y_b$", color="tab:green", fontsize=10)
ax.plot(L, 0, "s", color="tab:red", ms=7, zorder=3)
ax.text(L, 0.95, "end-effector $\\{b\\}$", ha="center", fontsize=9)

# the same physical joint axis, described from each frame
ax.annotate("", xy=(0, 0), xytext=(L, 0),
            arrowprops=dict(arrowstyle="->", color="tab:purple", lw=1.4, ls="--"))
ax.text(L / 2, 0.18, "same axis, two descriptions", ha="center",
        color="tab:purple", fontsize=9)

ax.text(0.05, -1.25,
        "From $\\{s\\}$ (space form):\n"
        "  $q_s=(0,0,0)$,  $S=(0,0,1,\\;0,0,0)$",
        fontsize=10, family="monospace",
        bbox=dict(boxstyle="round", fc="#eef", ec="0.7"))
ax.text(L + 0.05, -1.25,
        "From $\\{b\\}$ (body form):\n"
        "  $q_b=(-L,0,0)$,  $B=(0,0,1,\\;0,L,0)$",
        fontsize=10, family="monospace", ha="left",
        bbox=dict(boxstyle="round", fc="#fee", ec="0.7"))
ax.annotate("", xy=(2.55, -1.0), xytext=(1.6, -0.75),
            arrowprops=dict(arrowstyle="->", color="0.5", lw=1))
ax.text(1.0, -0.62, "$B=[\\mathrm{Ad}_{M^{-1}}]\\,S$", fontsize=11, color="0.3")

ax.set_aspect("equal")
ax.set_xlim(-0.8, L + 2.2)
ax.set_ylim(-1.7, 1.4)
ax.axis("off")
ax.set_title("One physical joint axis, described in the space frame ($S$)\n"
             "vs the end-effector frame ($B$) — the only difference between the two PoE forms",
             fontsize=11, fontweight="bold")
fig.tight_layout()
fig.savefig(f"{OUT}/04b_space_vs_body_screw.png", dpi=130, bbox_inches="tight")
plt.close(fig)
print("wrote figure to", OUT)
