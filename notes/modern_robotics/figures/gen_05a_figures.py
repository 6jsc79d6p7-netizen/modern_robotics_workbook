"""Figures for note 05a — the Jacobian as 'columns = tip velocities'.

Fig 1: a 2R planar arm with the two Jacobian columns drawn as the tip-velocity
       arrows you get from theta_dot_1 = 1 (joint 1 only) and theta_dot_2 = 1
       (joint 2 only). Reproduces the idea of book Fig 5.1.
Fig 2: the same arm at a singular configuration (theta_2 = 0, arm straight):
       the two columns become collinear -> the tip can't move along the arm.
"""
import numpy as np
import matplotlib.pyplot as plt

L1 = L2 = 1.0


def fk(t1, t2):
    """Joint positions: origin, elbow, tip."""
    p0 = np.array([0.0, 0.0])
    p1 = p0 + L1 * np.array([np.cos(t1), np.sin(t1)])
    p2 = p1 + L2 * np.array([np.cos(t1 + t2), np.sin(t1 + t2)])
    return p0, p1, p2


def jac(t1, t2):
    """2R planar Jacobian (Eq 5.1): columns are tip velocity for unit joint rate."""
    J1 = np.array([-L1 * np.sin(t1) - L2 * np.sin(t1 + t2),
                    L1 * np.cos(t1) + L2 * np.cos(t1 + t2)])
    J2 = np.array([-L2 * np.sin(t1 + t2),
                    L2 * np.cos(t1 + t2)])
    return J1, J2


def draw_arm(ax, t1, t2, title):
    p0, p1, p2 = fk(t1, t2)
    pts = np.array([p0, p1, p2])
    ax.plot(pts[:, 0], pts[:, 1], '-o', color='0.3', lw=4, ms=8, zorder=2)
    ax.plot(*p0, 'ks', ms=10, zorder=3)  # base
    J1, J2 = jac(t1, t2)
    # arrows from the tip
    ax.annotate('', xy=p2 + J1, xytext=p2,
                arrowprops=dict(arrowstyle='-|>', color='tab:red', lw=2.5))
    ax.annotate('', xy=p2 + J2, xytext=p2,
                arrowprops=dict(arrowstyle='-|>', color='tab:blue', lw=2.5))
    ax.text(*(p2 + 0.6 * J1 + np.array([0.10, 0.0])), r'$J_1\ (\dot\theta_1=1)$',
            color='tab:red', fontsize=11, fontweight='bold', ha='left', va='center')
    ax.text(*(p2 + 0.6 * J2 + np.array([-0.12, 0.0])), r'$J_2\ (\dot\theta_2=1)$',
            color='tab:blue', fontsize=11, fontweight='bold', ha='right', va='center')
    ax.set_title(title, fontsize=12)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlim(-1.4, 3.0)
    ax.set_ylim(-1.4, 3.2)


fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
draw_arm(axes[0], 0.0, np.pi / 4,
         r'Non-singular ($\theta_2=45°$):'
         '\n' r'$J_1,J_2$ span the plane $\Rightarrow$ tip moves any direction')
draw_arm(axes[1], np.pi / 6, 0.0,
         r'Singular ($\theta_2=0°$, arm straight):'
         '\n' r'$J_1\parallel J_2 \Rightarrow$ no tip velocity along the arm')
fig.suptitle('Jacobian columns ARE the tip velocities for unit joint rates',
             fontsize=13, fontweight='bold')
fig.tight_layout()
fig.savefig('05a_jacobian_columns.png', dpi=130, bbox_inches='tight')
print('wrote 05a_jacobian_columns.png')
