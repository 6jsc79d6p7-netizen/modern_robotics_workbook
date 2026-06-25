"""6a figures: 2R planar IK — lefty/righty solutions and the annulus workspace."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge

L1, L2 = 2.0, 1.0          # link lengths (L1 > L2)
x, y = 1.8, 1.4            # target tip position

def fk(t1, t2):
    p1 = np.array([L1*np.cos(t1), L1*np.sin(t1)])
    p2 = p1 + np.array([L2*np.cos(t1+t2), L2*np.sin(t1+t2)])
    return p1, p2

# --- solve IK (law of cosines) ---
r2 = x*x + y*y
D = (r2 - L1**2 - L2**2) / (2*L1*L2)         # cos(theta2)
t2_a =  np.arccos(D)                          # righty (elbow one way)
t2_b = -np.arccos(D)                          # lefty
def t1_for(t2):
    return np.arctan2(y, x) - np.arctan2(L2*np.sin(t2), L1 + L2*np.cos(t2))
sols = [(t1_for(t2_a), t2_a, 'righty (elbow-down)', '#1f77b4'),
        (t1_for(t2_b), t2_b, 'lefty (elbow-up)',   '#d62728')]

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))

# ---------- left: workspace annulus ----------
ax1.add_patch(Wedge((0, 0), L1+L2, 0, 360, width=(L1+L2)-(L1-L2),
                    facecolor='#cfe8ff', edgecolor='none'))
for r, ls, lab in [(L1+L2, '-', 'outer  L1+L2'), (L1-L2, '-', 'inner  L1-L2')]:
    th = np.linspace(0, 2*np.pi, 200)
    ax1.plot(r*np.cos(th), r*np.sin(th), ls, color='#3b7dd8', lw=1.5)
ax1.plot([x], [y], 'k*', ms=15, zorder=5)
ax1.annotate('target (x,y)', (x, y), textcoords='offset points',
             xytext=(8, 6), fontsize=10)
ax1.text(0, 0, 'reachable\nannulus', ha='center', va='center', fontsize=10,
         color='#1a4f8a')
ax1.set_title('Workspace: where solutions exist\n(0 outside · 1 on edge · 2 inside)')
ax1.set_aspect('equal'); ax1.set_xlim(-3.4, 3.4); ax1.set_ylim(-3.4, 3.4)
ax1.axhline(0, color='gray', lw=.5); ax1.axvline(0, color='gray', lw=.5)

# ---------- right: the two IK solutions ----------
for t1, t2, lab, c in sols:
    p1, p2 = fk(t1, t2)
    ax2.plot([0, p1[0], p2[0]], [0, p1[1], p2[1]], '-o', color=c, lw=3,
             ms=7, label=f'{lab}:  θ=({np.degrees(t1):.0f}°,{np.degrees(t2):.0f}°)')
ax2.plot([x], [y], 'k*', ms=15, zorder=5)
ax2.set_title('Same target, two joint solutions\n("elbow up" vs "elbow down")')
ax2.set_aspect('equal'); ax2.set_xlim(-0.6, 2.6); ax2.set_ylim(-0.6, 2.6)
ax2.axhline(0, color='gray', lw=.5); ax2.axvline(0, color='gray', lw=.5)
ax2.legend(loc='lower left', fontsize=9)

plt.tight_layout()
out = __file__.rsplit('/', 1)[0] + '/06a_2R_ik_solutions.png'
plt.savefig(out, dpi=130, bbox_inches='tight')
print('wrote', out)
