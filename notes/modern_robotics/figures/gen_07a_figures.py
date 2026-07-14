"""Figure for note 07a — parallel mechanisms: planar 3xRPR and the delta robot.

Left panel: planar 3xRPR. A fixed base triangle (anchors a_i), a moving
platform triangle (anchors b_i, rotated/translated by (p, phi)), three
actuated prismatic legs d_i = p + b_i - a_i. Illustrates loop closure and
the actuated-prismatic / passive-revolute split.

Right panel: schematic delta robot. Three base-mounted motors 120 deg apart,
upper arms, parallelogram forearms (drawn as double bars) to a small moving
platform that stays horizontal -> net 3-DOF translation.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

ROT = lambda a: np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def panel_3rpr(ax):
    # fixed base anchors a_i (large triangle), platform anchors b_i (small tri)
    Rbase = 3.0
    a = np.array([[np.cos(t), np.sin(t)] for t in np.deg2rad([90, 210, 330])]) * Rbase
    Rplat = 1.0
    b_local = np.array([[np.cos(t), np.sin(t)] for t in np.deg2rad([90, 210, 330])]) * Rplat
    p = np.array([0.4, 0.6])
    phi = np.deg2rad(20)
    b = (ROT(phi) @ b_local.T).T + p  # platform anchors in {s}

    # base triangle
    tri = np.vstack([a, a[0]])
    ax.plot(tri[:, 0], tri[:, 1], '-', color='0.4', lw=2, zorder=1)
    ax.fill(a[:, 0], a[:, 1], color='0.85', zorder=0)
    # platform triangle
    triP = np.vstack([b, b[0]])
    ax.fill(b[:, 0], b[:, 1], color='#cfe3ff', zorder=3)
    ax.plot(triP[:, 0], triP[:, 1], '-', color='#1f5fbf', lw=2, zorder=4)

    # legs (actuated prismatic) with double-line to suggest a piston
    for i in range(3):
        ax.plot([a[i, 0], b[i, 0]], [a[i, 1], b[i, 1]],
                color='#d2691e', lw=4, solid_capstyle='round', zorder=2)
        mid = 0.5 * (a[i] + b[i])
        d = b[i] - a[i]
        ax.annotate(f'$s_{i+1}$', mid + 0.18 * np.array([-d[1], d[0]]) / np.linalg.norm(d),
                    color='#a0522d', fontsize=11, ha='center', va='center')
        # base revolute (passive) and platform revolute (passive)
        ax.plot(*a[i], 'o', color='white', mec='0.3', ms=8, zorder=5)
        ax.plot(*b[i], 'o', color='white', mec='#1f5fbf', ms=7, zorder=6)
        ax.annotate(f'$a_{i+1}$', a[i] + 0.30 * a[i] / np.linalg.norm(a[i]),
                    fontsize=9, color='0.3', ha='center')

    # p vector from {s} origin to {b} origin
    ax.plot(0, 0, 'ks', ms=6, zorder=7)
    ax.add_patch(FancyArrowPatch((0, 0), tuple(p), arrowstyle='-|>',
                                 mutation_scale=14, color='green', lw=1.6, zorder=8))
    ax.annotate('$p$', p * 0.5 + np.array([-0.15, 0.18]), color='green', fontsize=12)
    ax.plot(*p, 'o', color='#1f5fbf', ms=5, zorder=9)
    ax.annotate('{s}', (-0.55, -0.35), fontsize=11)
    ax.annotate('{b}', p + np.array([0.12, 0.06]), color='#1f5fbf', fontsize=11)
    # phi arc
    ax.annotate(r'$\phi$ (platform tilt)', (p[0] + 0.2, p[1] + 0.9),
                color='#1f5fbf', fontsize=9)

    ax.set_title('Planar 3×RPR\n3 actuated prismatic legs (orange), 6 passive revolutes (○)',
                 fontsize=10)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_xlim(-4, 4); ax.set_ylim(-4, 4.2)


def panel_delta(ax):
    # base motors 120 deg apart on a circle of radius Rb at top
    Rb = 2.4
    ztop = 3.0
    motors = np.array([[Rb * np.cos(t), ztop] for t in np.deg2rad([150, 30, -90])])
    # collapse 3D->2D: show two side legs splayed + one center-ish for schematic
    # We draw a clean 2D elevation: base bar at top, platform small bar at bottom.
    # platform center
    plat = np.array([0.2, -2.4])
    plat_w = 0.7

    # base bar
    ax.plot([-Rb - 0.4, Rb + 0.4], [ztop, ztop], color='0.4', lw=6,
            solid_capstyle='round', zorder=1)
    ax.annotate('fixed base + 3 motors', (0, ztop + 0.35), ha='center', fontsize=9, color='0.3')

    # two visible legs (left and right) drawn as upper arm + parallelogram forearm
    for sgn, hub_x in [(-1, -Rb), (1, Rb)]:
        hub = np.array([hub_x, ztop])
        # motor pivot
        ax.plot(*hub, 'o', color='#b22222', ms=11, zorder=6)
        # upper arm: swings down-out to elbow
        elbow = hub + np.array([sgn * 0.7, -1.5])
        ax.plot([hub[0], elbow[0]], [hub[1], elbow[1]], color='#333', lw=5,
                solid_capstyle='round', zorder=4)
        ax.annotate(r'$\theta$', hub + np.array([sgn * 0.35, -0.45]),
                    color='#b22222', fontsize=11)
        # parallelogram forearm: two parallel bars from elbow region to platform
        offset = np.array([0.0, 0.32])
        pe_top = plat + np.array([sgn * plat_w, 0]) + offset
        pe_bot = plat + np.array([sgn * plat_w, 0]) - offset
        el_top = elbow + offset
        el_bot = elbow - offset
        ax.plot([el_top[0], pe_top[0]], [el_top[1], pe_top[1]], color='#1f5fbf', lw=3, zorder=3)
        ax.plot([el_bot[0], pe_bot[0]], [el_bot[1], pe_bot[1]], color='#1f5fbf', lw=3, zorder=3)
        # connecting rungs (the ball joints)
        for P in (elbow, plat + np.array([sgn * plat_w, 0])):
            ax.plot([P[0], P[0]], [P[1] + 0.32, P[1] - 0.32], color='#1f5fbf', lw=1.5, zorder=3)
            ax.plot(P[0], P[1] + 0.32, 'o', color='white', mec='#1f5fbf', ms=6, zorder=7)
            ax.plot(P[0], P[1] - 0.32, 'o', color='white', mec='#1f5fbf', ms=6, zorder=7)

    # moving platform (small horizontal bar) — emphasize it stays level
    ax.plot([plat[0] - plat_w, plat[0] + plat_w], [plat[1], plat[1]],
            color='#1f5fbf', lw=6, solid_capstyle='round', zorder=5)
    ax.annotate('moving platform\n(stays horizontal → pure x-y-z translation)',
                (plat[0], plat[1] - 0.6), ha='center', fontsize=9, color='#1f5fbf')

    # forearm label
    ax.annotate('parallelogram\nforearm', (Rb - 0.1, -0.3), fontsize=8.5,
                color='#1f5fbf', ha='center')

    ax.set_title('Delta robot (elevation)\nmotors on the base → light, fast moving parts',
                 fontsize=10)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_xlim(-Rb - 1.2, Rb + 1.4); ax.set_ylim(-3.6, 3.8)


fig, axes = plt.subplots(1, 2, figsize=(12, 5.6))
panel_3rpr(axes[0])
panel_delta(axes[1])
fig.suptitle('Parallel mechanisms (closed chains): IK easy / FK hard',
             fontsize=13, y=0.99)
fig.tight_layout(rect=(0, 0, 1, 0.96))
out = __file__.rsplit('/', 1)[0] + '/07a_parallel_mechanisms.png'
fig.savefig(out, dpi=130, bbox_inches='tight')
print('wrote', out)
