"""Figure for note 05a §5 — the spatial RRRP chain (book Example 5.2, Fig 5.7).

Three revolute joints all about z_s (a planar 3R arm in the xy-plane), then a
prismatic 4th joint that slides along z_s (lifts the tip out of the plane).

The point of the figure is to make the column-by-column Jacobian build concrete:
  - each revolute column is (omega_i, -omega_i x q_i): so we SHOW where each
    axis point q_i sits at the current theta, with omega_i = z (out of plane).
  - the prismatic column is (0, v_hat): pure translation along z, no q needed.

Left  : top-down (xy) view -> read off q1,q2,q3 and the L1,L2 lever arms.
Right : 3D view -> see the prismatic joint lift the EE along z.
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

L1, L2, L3, d = 1.0, 1.0, 0.6, 0.9
t1, t2, t3 = np.deg2rad(30), np.deg2rad(45), np.deg2rad(20)

c1, s1 = np.cos(t1), np.sin(t1)
c12, s12 = np.cos(t1 + t2), np.sin(t1 + t2)
c123, s123 = np.cos(t1 + t2 + t3), np.sin(t1 + t2 + t3)

q1 = np.array([0.0, 0.0, 0.0])                 # joint 1 axis point
q2 = np.array([L1 * c1, L1 * s1, 0.0])         # joint 2 axis point
q3 = q2 + np.array([L2 * c12, L2 * s12, 0.0])  # joint 3 axis point
p4 = q3 + np.array([L3 * c123, L3 * s123, 0.0])  # prismatic base
ee = p4 + np.array([0.0, 0.0, d])              # end-effector (after sliding +z)

REV = 'tab:blue'
PRIS = 'tab:orange'
AX = 'tab:purple'

# ---------------- Left: top-down (xy) ----------------
fig = plt.figure(figsize=(13, 5.8))
axL = fig.add_subplot(1, 2, 1)

chain = np.array([q1, q2, q3, p4])
axL.plot(chain[:, 0], chain[:, 1], '-', color='0.4', lw=5, zorder=1)
# revolute joints: axis out of page, drawn as circled-dot
for q, name in [(q1, '$q_1$'), (q2, '$q_2$'), (q3, '$q_3$')]:
    axL.plot(q[0], q[1], 'o', ms=16, mfc='white', mec=REV, mew=2.5, zorder=3)
    axL.plot(q[0], q[1], '.', ms=5, color=REV, zorder=4)  # dot = z out of page
    axL.annotate(name, (q[0], q[1]), textcoords='offset points',
                 xytext=(8, 10), color=REV, fontsize=13, fontweight='bold')
# prismatic at the end: axis goes OUT of the page (along +z), so just a circled-dot too
axL.plot(p4[0], p4[1], 's', ms=14, mfc='white', mec=PRIS, mew=2.5, zorder=3)
axL.annotate('joint 4 (P)\nslides along $\\hat z$ (out of page)', (p4[0], p4[1]),
             textcoords='offset points', xytext=(10, -28), color=PRIS, fontsize=10)
# link length labels
mid1 = (q1 + q2) / 2
mid2 = (q2 + q3) / 2
axL.annotate('$L_1$', mid1[:2], textcoords='offset points', xytext=(-18, 4),
             fontsize=12)
axL.annotate('$L_2$', mid2[:2], textcoords='offset points', xytext=(-6, 14),
             fontsize=12)
axL.plot(0, 0, 'k^', ms=12, zorder=5)
axL.text(1.18, 0.18,
         r'all 3 revolute axes point'
         '\n'
         r'along $\hat z_s=(0,0,1)$, so every'
         '\n'
         r'revolute column is $(\,\hat z,\ -\hat z\times q_i\,)$',
         fontsize=9.5, color=REV, ha='left',
         bbox=dict(boxstyle='round', fc='aliceblue', ec=REV, alpha=0.9))
axL.set_title('Top-down (xy): read off the axis points $q_i$', fontsize=12)
axL.set_xlabel(r'$\hat x_s$'); axL.set_ylabel(r'$\hat y_s$')
axL.set_aspect('equal'); axL.grid(alpha=0.3)
axL.set_xlim(-0.6, 2.6); axL.set_ylim(-0.4, 2.4)

# ---------------- Right: 3D view ----------------
ax3 = fig.add_subplot(1, 2, 2, projection='3d')
ax3.plot(chain[:, 0], chain[:, 1], chain[:, 2], '-', color='0.4', lw=5)
# prismatic slide (along z)
ax3.plot([p4[0], ee[0]], [p4[1], ee[1]], [p4[2], ee[2]], '-',
         color=PRIS, lw=5)
ax3.quiver(p4[0], p4[1], p4[2], 0, 0, d, color=PRIS, lw=2.5,
           arrow_length_ratio=0.18)
# revolute axis arrows (omega = z) at each revolute joint
for q in (q1, q2, q3):
    ax3.quiver(q[0], q[1], q[2], 0, 0, 0.55, color=AX, lw=2,
               arrow_length_ratio=0.25)
# joint markers
for q, lbl in [(q1, 'R1'), (q2, 'R2'), (q3, 'R3')]:
    ax3.scatter(*q, color=REV, s=55, depthshade=False)
    ax3.text(q[0], q[1], q[2] + 0.62, lbl, color=AX, fontsize=10)
ax3.scatter(*p4, color=PRIS, s=55, depthshade=False)
ax3.scatter(*ee, color='k', s=70, marker='*', depthshade=False)
ax3.text(ee[0], ee[1], ee[2] + 0.08, ' EE', fontsize=10)
ax3.text(p4[0] + 0.05, p4[1], p4[2] + d / 2,
         r'  P: $v_s=\hat z$', color=PRIS, fontsize=10)
ax3.set_title('3D: revolutes about $\\hat z$, prismatic lifts along $\\hat z$',
              fontsize=12)
ax3.set_xlabel(r'$\hat x_s$'); ax3.set_ylabel(r'$\hat y_s$')
ax3.set_zlabel(r'$\hat z_s$')
ax3.set_box_aspect((1, 1, 0.8))
ax3.view_init(elev=22, azim=-60)

fig.suptitle('Spatial RRRP chain — building the space Jacobian column by column',
             fontsize=13, fontweight='bold')
fig.tight_layout()
fig.savefig('05a_rrrp_chain.png', dpi=130, bbox_inches='tight')
print('wrote 05a_rrrp_chain.png')
