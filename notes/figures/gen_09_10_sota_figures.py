"""Figure for notes/09_10_learned_sota.md — the classical->learned spectrum.

Run: ./.venv/bin/python notes/figures/gen_09_10_sota_figures.py
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from pathlib import Path

OUT = Path(__file__).parent

fig, ax = plt.subplots(figsize=(13, 6.2))
ax.axis("off")
ax.set_xlim(0, 13)
ax.set_ylim(0, 10)

# horizontal "amount learned" arrow
ax.annotate("", (12.6, 9.4), (0.4, 9.4),
            arrowprops=dict(arrowstyle="-|>", lw=2.5, color="0.3"))
ax.text(0.4, 9.65, "MODEL-BASED / CLASSICAL", fontsize=10, weight="bold",
        color="0.3")
ax.text(12.6, 9.65, "END-TO-END LEARNED", fontsize=10, weight="bold",
        color="0.3", ha="right")
ax.text(6.5, 9.0, "← knows world geometry + explicit goal        "
        "perceives world + infers goal from pixels/language →",
        fontsize=8.5, color="0.45", ha="center", style="italic")

rungs = [
    ("1. Classical planner\n+ retiming",
     "RRT* / PRM / A*  →  TOPP-RA\n→ track (Ch 11)",
     "known model, given q_goal.\nCh10 path × Ch9 time-scaling.",
     "#cfe3ff"),
    ("2. Trajectory\noptimization (GPU)",
     "cuRobo, CHOMP, TrajOpt",
     "Ch10 §10.7 nonlinear-opt,\nmodernized + parallelized.\nStill needs a goal pose.",
     "#bcd6ff"),
    ("3. Learned\nplanner",
     "MotionPolicyNets (MπNets),\nneural SDF / collision fields",
     "net: (start, goal, point cloud)\n→ trajectory. Fast, reactive.",
     "#cfe6cf"),
    ("4. Imitation\nvisuomotor policy",
     "Diffusion Policy, ACT",
     "pixels+state → ACTION CHUNK.\nCollapses plan+traj+react.\nMulti-modal. (roadmap M2)",
     "#ffe6c2"),
    ("5. VLA\n(language-conditioned)",
     "RT-2, OpenVLA, Octo,\npi0, Gemini Robotics",
     "images + language → actions.\nInfers the goal itself.\n(roadmap M3)",
     "#ffd0c2"),
]

n = len(rungs)
w = 2.35
gap = (13 - n * w) / (n + 1)
y0, h = 3.0, 5.0
for i, (title, methods, desc, c) in enumerate(rungs):
    x = gap + i * (w + gap)
    box = FancyBboxPatch((x, y0), w, h, boxstyle="round,pad=0.04,rounding_size=0.12",
                         fc=c, ec="k", lw=1.2)
    ax.add_patch(box)
    ax.text(x + w / 2, y0 + h - 0.55, title, ha="center", va="top",
            fontsize=9.2, weight="bold")
    ax.text(x + w / 2, y0 + h - 1.9, methods, ha="center", va="top",
            fontsize=7.8, color="tab:blue")
    ax.text(x + w / 2, y0 + 1.55, desc, ha="center", va="top",
            fontsize=7.3, color="0.25")

# bottom: shared substrate
sub = FancyBboxPatch((gap, 0.5), 13 - 2 * gap, 1.6,
                     boxstyle="round,pad=0.04,rounding_size=0.12",
                     fc="#eeeeee", ec="k", lw=1.3)
ax.add_patch(sub)
ax.text(6.5, 1.75, "SHARED SUBSTRATE  (all 5 emit into this — this is MR)",
        ha="center", fontsize=9, weight="bold")
ax.text(6.5, 1.0,
        "action space = SE(3) pose / joint deltas  ·  IK + Jacobian (Ch 4–6)  ·  "
        "low-level tracking + impedance controller (Ch 11)  ·  SE(3) interpolation "
        "& TOPP retiming as plumbing",
        ha="center", fontsize=7.6, color="0.25")

# arrows from each rung down to substrate
for i in range(n):
    x = gap + i * (w + gap) + w / 2
    ax.annotate("", (x, 2.15), (x, 2.95),
                arrowprops=dict(arrowstyle="-|>", lw=1.1, color="0.5"))

ax.set_title("The learned spectrum that replaces Ch 9 (trajectory) + Ch 10 "
             "(planning)\n— but never replaces the MR substrate underneath",
             fontsize=11)
fig.tight_layout()
fig.savefig(OUT / "09_10_learned_spectrum.png", dpi=125, bbox_inches="tight")
print("wrote 09_10_learned_spectrum.png")
