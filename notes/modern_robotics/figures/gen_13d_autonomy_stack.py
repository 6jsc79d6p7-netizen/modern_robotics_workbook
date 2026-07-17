"""Generate the mobile-manipulator autonomy-stack figure for 13d.

A vertical block diagram: sensors -> SLAM/state-estimation -> semantic map
-> task/language -> planning -> control -> robot, with a 'classical' vs
'learned/SOTA' annotation on each layer, and the transform tree called out.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.font_manager as fm

plt.rcParams["font.size"] = 10

fig, ax = plt.subplots(figsize=(11, 9))
ax.set_xlim(0, 10)
ax.set_ylim(0, 12)
ax.axis("off")

# (y_center, height, title, classical, learned, color)
layers = [
    (11.0, 1.0, "SENSORS",
     "RGB-D / stereo, LiDAR, IMU, wheel encoders",
     "same hardware — feeds everything below", "#dbe9f4"),
    (9.4, 1.2, "SLAM / STATE ESTIMATION",
     "front-end tracking (features/ICP) +\nback-end pose-graph / bundle adjustment + loop closure",
     "MASt3R-SLAM · VGGT-SLAM · FoundationSLAM · 3DGS-SLAM", "#cfe6d4"),
    (7.7, 1.1, "MAP  (geometry)",
     "occupancy grid / point cloud / TSDF",
     "neural field · 3D Gaussian splat (photorealistic + queryable)", "#cfe6d4"),
    (6.1, 1.1, "SEMANTIC LAYER",
     "hand-labeled classes (chair, wall)",
     "open-vocab: CLIP feats in 3D · ConceptGraphs · HOV-SG · DualMap", "#f6e3c5"),
    (4.5, 1.1, "TASK / LANGUAGE",
     "hard-coded goal pose (x, y, theta)",
     "'go to the kitchen, find the mug' -> LLM/VLM plan · VLN", "#f6e3c5"),
    (2.9, 1.1, "PLANNING",
     "global A*/RRT* on costmap + local DWA/TEB (nav2)",
     "learned local policy · zero-shot VLM waypoints · frontier explore", "#f4d4d4"),
    (1.3, 1.1, "CONTROL  <- this is Modern Robotics",
     "diff-drive / mecanum kinematics -> wheel cmds (Ch 13)\nvelocity / torque / impedance (Ch 11)",
     "the substrate every layer above finally commands", "#e8d4f0"),
]

for yc, h, title, classical, learned, color in layers:
    box = FancyBboxPatch((0.4, yc - h / 2), 6.4, h,
                         boxstyle="round,pad=0.04,rounding_size=0.12",
                         linewidth=1.3, edgecolor="#33333a", facecolor=color)
    ax.add_patch(box)
    ax.text(0.62, yc + h / 2 - 0.24, title, fontsize=10.5, fontweight="bold",
            va="center", ha="left")
    ax.text(0.62, yc - 0.02, "classical:  " + classical, fontsize=8.2,
            va="center", ha="left", color="#333")
    ax.text(0.62, yc - h / 2 + 0.24, "SOTA:  " + learned, fontsize=8.2,
            va="center", ha="left", color="#7a1f1f", style="italic")

# downward arrows between layers
for (yc_top, h_top, *_), (yc_bot, h_bot, *_) in zip(layers[:-1], layers[1:]):
    a = FancyArrowPatch((3.6, yc_top - h_top / 2), (3.6, yc_bot + h_bot / 2),
                        arrowstyle="-|>", mutation_scale=14,
                        linewidth=1.4, color="#33333a")
    ax.add_patch(a)

# feedback arrow (robot state back up to SLAM)
fb = FancyArrowPatch((6.8, 1.3), (7.5, 1.3), arrowstyle="-", linewidth=1.2,
                     color="#555")
ax.add_patch(fb)
ax.annotate("", xy=(7.5, 9.4), xytext=(7.5, 1.3),
            arrowprops=dict(arrowstyle="-", linewidth=1.2, color="#555"))
ax.annotate("", xy=(6.8, 9.4), xytext=(7.5, 9.4),
            arrowprops=dict(arrowstyle="-|>", mutation_scale=13,
                            linewidth=1.2, color="#555"))
ax.text(7.65, 5.35, "motion changes\nwhat sensors see\n(closes the loop)",
        fontsize=8, color="#555", rotation=90, va="center", ha="center")

# transform-tree callout on the right
ax.text(8.55, 9.9, "TRANSFORM TREE\n(the MR Ch 3 language\nthreading it together)",
        fontsize=8.6, fontweight="bold", va="top", ha="left", color="#1f3a5f")
tree = ("map\n └─ odom\n     └─ base_link\n         ├─ camera_link\n"
        "         └─ arm_base\n             └─ … └─ ee")
ax.text(8.55, 8.9, tree, fontsize=8, family="monospace", va="top", ha="left",
        color="#1f3a5f")
ax.text(8.55, 6.6,
        "Every box speaks SE(3):\nposes, frames, transforms.\n"
        "SLAM estimates map->base,\nsemantics live in map,\n"
        "the goal is an SE(3) pose,\ncontrol closes it in base.",
        fontsize=7.8, va="top", ha="left", color="#1f3a5f")

ax.text(3.6, 11.75, "The Mobile-Manipulator Autonomy Stack",
        fontsize=13, fontweight="bold", ha="center")

plt.tight_layout()
out = __file__.replace("gen_", "").replace(".py", ".png")
plt.savefig(out, dpi=140, bbox_inches="tight")
print("wrote", out)
