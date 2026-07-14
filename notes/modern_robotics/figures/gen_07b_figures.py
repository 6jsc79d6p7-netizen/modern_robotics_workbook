"""Figure for note 07b — the three types of closed-chain singularity,
illustrated on the planar five-bar linkage (the book's vehicle, Figs 7.7-7.9).

Five-bar: two ground pivots A, B (the actuated joints, shaded). Left arm
A-C-P, right arm B-D-P, meeting at P. Schematic poses chosen to make each
singularity legible; annotations name the failure mode and the lost/
uncontrollable motion.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch


def draw_bar(ax, pts, color='#333', lw=4):
    pts = np.asarray(pts)
    ax.plot(pts[:, 0], pts[:, 1], '-', color=color, lw=lw,
            solid_capstyle='round', zorder=2)


def joint(ax, p, actuated=False, label=None, dy=0.18):
    if actuated:
        ax.plot(*p, 'o', color='#777', mec='k', ms=13, zorder=5)
        ax.plot(*p, 'o', color='white', ms=4, zorder=6)
    else:
        ax.plot(*p, 'o', color='white', mec='k', ms=9, zorder=5)
    if label:
        ax.annotate(label, (p[0], p[1] + dy), ha='center', fontsize=9)


def ground(ax, p):
    ax.plot([p[0]-0.25, p[0]+0.25], [p[1]-0.12, p[1]-0.12], 'k-', lw=2)
    for k in np.linspace(-0.25, 0.18, 5):
        ax.plot([p[0]+k, p[0]+k-0.12], [p[1]-0.12, p[1]-0.30], 'k-', lw=1)


def panel_cspace(ax):
    A, B = np.array([-1.5, 0]), np.array([1.5, 0])
    # branch 1 (solid): P up
    C, D, P = np.array([-0.75, 1.05]), np.array([0.75, 1.05]), np.array([0, 1.7])
    # branch 2 (ghost): the elbows flipped -> the other assembly mode
    Cg, Dg = np.array([-1.15, 0.55]), np.array([1.15, 0.55])
    draw_bar(ax, [A, Cg, P], color='#bbb', lw=3)
    draw_bar(ax, [B, Dg, P], color='#bbb', lw=3)
    draw_bar(ax, [A, C, P]); draw_bar(ax, [B, D, P])
    for p in (C, D, P):
        joint(ax, p)
    for p in (Cg, Dg):
        ax.plot(*p, 'o', color='white', mec='#bbb', ms=9, zorder=4)
    joint(ax, A, actuated=True); joint(ax, B, actuated=True)
    ground(ax, A); ground(ax, B)
    # fork arrow
    ax.add_patch(FancyArrowPatch((0, 2.05), (-0.55, 2.45), arrowstyle='-|>',
                 mutation_scale=12, color='#b22222', lw=1.6))
    ax.add_patch(FancyArrowPatch((0, 2.05), (0.55, 2.45), arrowstyle='-|>',
                 mutation_scale=12, color='#b22222', lw=1.6))
    ax.annotate('two branches\nmeet here', (0, 2.62), ha='center',
                fontsize=8.5, color='#b22222')
    ax.set_title('(i) Configuration-space singularity\n'
                 r'self-intersection of C-space:  rank $H<p$'
                 '\nindependent of WHICH joints are actuated', fontsize=9)
    ax.set_xlim(-2.4, 2.4); ax.set_ylim(-0.6, 3.0)


def panel_actuator(ax):
    # everything collinear on the x-axis -> central joint P can buckle up/down
    A, B = np.array([-1.6, 0]), np.array([1.6, 0])
    C, D, P = np.array([-0.8, 0]), np.array([0.8, 0]), np.array([0, 0])
    draw_bar(ax, [A, C, P]); draw_bar(ax, [B, D, P])
    for p in (C, D):
        joint(ax, p, dy=-0.32)
    joint(ax, A, actuated=True, dy=-0.34); joint(ax, B, actuated=True, dy=-0.34)
    ax.plot(*P, 'o', color='#1f5fbf', mec='k', ms=10, zorder=6)
    ground(ax, A); ground(ax, B)
    ax.add_patch(FancyArrowPatch((0, 0.12), (0, 1.0), arrowstyle='-|>',
                 mutation_scale=13, color='#b22222', lw=1.8))
    ax.add_patch(FancyArrowPatch((0, -0.12), (0, -1.0), arrowstyle='-|>',
                 mutation_scale=13, color='#b22222', lw=1.8))
    ax.annotate('P buckles ↑ or ↓\neven with A, B locked', (0, 1.25),
                ha='center', fontsize=8.5, color='#b22222')
    ax.set_title('(ii) Actuator singularity\n'
                 r'rank $H_p<p$:  locking actuators fails to rigidify,'
                 '\nor they can’t be driven independently'
                 '\nDEPENDS on which joints are actuated', fontsize=9)
    ax.set_xlim(-2.4, 2.4); ax.set_ylim(-1.5, 2.0)


def panel_endeff(ax):
    A, B = np.array([-1.5, 0]), np.array([1.5, 0])
    # distal links C-P and D-P collinear (horizontal) -> P loses motion along that line
    C, D, P = np.array([-0.9, 1.15]), np.array([0.9, 1.15]), np.array([0, 1.15])
    draw_bar(ax, [A, C, P]); draw_bar(ax, [B, D, P])
    for p in (C, D):
        joint(ax, p)
    ax.plot(*P, 'o', color='#1f5fbf', mec='k', ms=10, zorder=6)
    joint(ax, A, actuated=True, dy=-0.34); joint(ax, B, actuated=True, dy=-0.34)
    ground(ax, A); ground(ax, B)
    # dashed line of the aligned distal links
    ax.plot([-1.6, 1.6], [1.15, 1.15], '--', color='#b22222', lw=1.3, zorder=1)
    # barred arrow: no motion along the line
    ax.add_patch(FancyArrowPatch((0.15, 1.5), (1.15, 1.5), arrowstyle='-|>',
                 mutation_scale=12, color='#b22222', lw=1.6))
    ax.plot([0.05, 0.05], [1.38, 1.62], '-', color='#b22222', lw=2.2)  # bar
    ax.annotate('no EE velocity\nalong the aligned links', (0.0, 1.8),
                ha='center', fontsize=8.5, color='#b22222')
    ax.annotate('P', (P[0]-0.05, P[1]+0.16), fontsize=9, color='#1f5fbf')
    ax.set_title('(iii) End-effector singularity\n'
                 'EE loses a DOF (distal links aligned, like a serial arm)'
                 '\nDEPENDS on the EE frame, NOT on actuation', fontsize=9)
    ax.set_xlim(-2.4, 2.4); ax.set_ylim(-0.6, 2.3)


fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
panel_cspace(axes[0]); panel_actuator(axes[1]); panel_endeff(axes[2])
for ax in axes:
    ax.set_aspect('equal'); ax.axis('off')
fig.suptitle('Three types of closed-chain singularity (planar five-bar linkage)',
             fontsize=13, y=1.02)
fig.tight_layout()
out = __file__.rsplit('/', 1)[0] + '/07b_singularity_types.png'
fig.savefig(out, dpi=130, bbox_inches='tight')
print('wrote', out)
