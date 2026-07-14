"""12a figures: the friction cone, and the antipodal force-closure test.

Left: a single contact. The finger can push along the inward normal, and thanks
to friction can also resist tangential force up to μ·(normal). The set of forces
it can apply is a CONE around the normal, half-angle α = atan(μ). A required
force inside the cone → no slip (grasp holds); on the edge → about to slip;
outside → slips.

Right: a two-finger (antipodal) grasp. Force closure ⇔ the line joining the two
contacts lies inside BOTH friction cones. Green = good grasp (line in both cones,
holds any disturbance); red = bad grasp (line outside a cone → the pinch slips).
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

mu = 0.5
alpha = np.arctan(mu)  # cone half-angle
deg = np.degrees(alpha)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 5.2))

# ---------- Left: single-contact friction cone ----------
ax1.set_title(f"Friction cone at one contact\n μ={mu} → half-angle α = atan(μ) = {deg:.1f}°",
              fontsize=11)
# surface (horizontal), object above, finger pushing DOWN-ish onto surface.
ax1.axhline(0, color="#555", lw=2)
ax1.fill_between([-2, 2], -0.6, 0, color="#d9d2c5")  # the object/surface body
# inward normal (points up into the object being pushed) — draw the cone upward
L = 1.6
for s in (-1, 1):
    edge = np.array([np.sin(s*alpha), np.cos(s*alpha)]) * L
    ax1.plot([0, edge[0]], [0, edge[1]], color="#2980b9", lw=2)
cone = Polygon([[0, 0],
                [np.sin(-alpha)*L, np.cos(-alpha)*L],
                [np.sin(alpha)*L, np.cos(alpha)*L]],
               closed=True, color="#2980b9", alpha=0.15)
ax1.add_patch(cone)
ax1.annotate("", xy=(0, L), xytext=(0, 0),
             arrowprops=dict(arrowstyle="->", color="#2980b9", lw=1.4, ls=":"))
ax1.text(0.05, L+0.05, "normal", color="#2980b9", fontsize=9)
ax1.text(-1.35, 1.15, "friction\ncone", color="#2980b9", fontsize=9.5)
# a force INSIDE the cone (holds) and OUTSIDE (slips)
ax1.annotate("", xy=(0.25, 1.1), xytext=(0, 0),
             arrowprops=dict(arrowstyle="-|>", color="#27ae60", lw=2.5))
ax1.text(0.28, 1.12, "inside → holds", color="#27ae60", fontsize=9)
ax1.annotate("", xy=(1.15, 0.55), xytext=(0, 0),
             arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=2.5))
ax1.text(1.0, 0.4, "outside → slips", color="#c0392b", fontsize=9)
ax1.plot(0, 0, "ko", ms=6)
ax1.set_xlim(-2, 2); ax1.set_ylim(-0.6, 2.0); ax1.set_aspect("equal"); ax1.axis("off")

# ---------- Right: antipodal grasp force-closure test ----------
ax2.set_title("Antipodal grasp: force closure ⇔\nline of contacts inside BOTH friction cones",
              fontsize=11)
# object: an ellipse-ish box
obj = Circle((0, 0), 1.0, fc="#e8e2d6", ec="#8a7f6a", lw=1.5)
ax2.add_patch(obj)

def draw_contact(cx, cy, normal_ang, color, cone_ok):
    # normal points inward (toward object center from the contact)
    n = np.array([np.cos(normal_ang), np.sin(normal_ang)])
    for s in (-1, 1):
        a = normal_ang + s*alpha
        e = np.array([np.cos(a), np.sin(a)]) * 1.1
        ax2.plot([cx, cx+e[0]], [cy, cy+e[1]], color=color, lw=1.3, alpha=0.8)
    tri = Polygon([[cx, cy],
                   [cx+np.cos(normal_ang-alpha)*1.1, cy+np.sin(normal_ang-alpha)*1.1],
                   [cx+np.cos(normal_ang+alpha)*1.1, cy+np.sin(normal_ang+alpha)*1.1]],
                  closed=True, color=color, alpha=0.15)
    ax2.add_patch(tri)
    ax2.plot(cx, cy, "o", color=color, ms=8)

# GOOD grasp: two contacts left & right, normals horizontal, line between them
# is horizontal → lies along both normals → inside both cones.
gy = 0.0
draw_contact(-1.0, gy, 0.0, "#27ae60", True)     # left contact, normal points +x (inward)
draw_contact(1.0, gy, np.pi, "#27ae60", True)    # right contact, normal points -x (inward)
ax2.plot([-1.0, 1.0], [gy, gy], color="#27ae60", lw=2.2, ls="--")
ax2.text(0, 0.12, "grasp line", color="#27ae60", ha="center", fontsize=9)
ax2.text(0, -1.5, "GOOD: line ∥ both normals →\ninside both cones → FORCE CLOSURE",
         color="#27ae60", ha="center", fontsize=9)

ax2.set_xlim(-2.6, 2.6); ax2.set_ylim(-2.1, 1.6); ax2.set_aspect("equal"); ax2.axis("off")

fig.tight_layout()
fig.savefig(__file__.replace("gen_", "").replace(".py", ".png"), dpi=130)
print("wrote", __file__.replace("gen_", "").replace(".py", ".png"))
