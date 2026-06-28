"""Figure for note 07a §7a.1 — delta robot inverse kinematics.

Left: ONE leg seen in its own vertical (u-z) plane. The single constraint
||F_i - E_i|| = l is a bicep CIRCLE (radius L about pivot B_i) meeting a
forearm CIRCLE (sphere of radius l about platform attachment E_i, cut by the
leg plane) -> two intersection elbows F+ (elbow-down, chosen) and F- (folded).
This is the geometric content of theta_i = atan2(B,A) +/- arccos(C/sqrt(A^2+B^2)).

Right: top view -- three legs 120 deg apart, each living in its own radial
vertical plane, solved independently. Uses the verified numbers
R=0.2, r=0.05, L=0.2, l=0.4, p=(0,0,-0.4) -> theta_i ~ 42 deg.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch

R, r, L, l = 0.2, 0.05, 0.2, 0.4
p = np.array([0.0, 0.0, -0.4])


def circle(ax, c, rad, **kw):
    t = np.linspace(0, 2 * np.pi, 200)
    ax.plot(c[0] + rad * np.cos(t), c[1] + rad * np.sin(t), **kw)


def panel_leg(ax):
    # leg-1 plane: coordinates (u, z). pivot B, attachment E.
    B = np.array([R, 0.0])
    E = np.array([p[0] + r, p[2]])           # (u,z) of platform attachment
    th = np.radians(42.0)
    Fp = B + L * np.array([np.cos(th), -np.sin(th)])   # elbow-down (chosen)
    thm = np.radians(-180.9)
    Fm = B + L * np.array([np.cos(thm), -np.sin(thm)])  # folded-back

    # base / ground hatch
    ax.plot([-0.12, 0.34], [0, 0], color='0.4', lw=3, zorder=1)
    for x0 in np.linspace(-0.10, 0.32, 11):
        ax.plot([x0, x0 - 0.04], [0, 0.05], color='0.6', lw=1)

    # the two constraint circles (dashed)
    circle(ax, B, L, ls='--', color='#333', lw=1.3, zorder=2)
    circle(ax, E, l, ls='--', color='#1f5fbf', lw=1.3, zorder=2)
    ax.annotate('bicep circle\n(radius $L$ about $B_i$)', (B[0] + 0.21, 0.02),
                fontsize=8.5, color='#333', ha='center')
    ax.annotate('forearm sphere ∩ plane\n(radius $l$ about $E_i$)',
                (0.40, -0.50), fontsize=8.5, color='#1f5fbf', ha='center')

    # chosen leg: bicep B->F+, forearm F+->E
    ax.plot([B[0], Fp[0]], [B[1], Fp[1]], color='#333', lw=5,
            solid_capstyle='round', zorder=5)
    ax.plot([Fp[0], E[0]], [Fp[1], E[1]], color='#d2691e', lw=5,
            solid_capstyle='round', zorder=5)
    # folded-back ghost leg
    ax.plot([B[0], Fm[0]], [B[1], Fm[1]], color='0.7', lw=3, zorder=3)
    ax.plot([Fm[0], E[0]], [Fm[1], E[1]], color='#f0c8a0', lw=3, zorder=3)

    # joints
    ax.plot(*B, 'o', color='#b22222', mec='k', ms=12, zorder=7)
    ax.plot(*Fp, 'o', color='white', mec='k', ms=9, zorder=7)
    ax.plot(*Fm, 'o', color='white', mec='0.7', ms=8, zorder=4)
    ax.plot(*E, 'o', color='#1f5fbf', mec='k', ms=10, zorder=7)

    ax.annotate('$B_i$ (motor pivot)', B + np.array([0.0, 0.07]), fontsize=9,
                color='#b22222', ha='center')
    ax.annotate('$F_i^{+}$ (elbow, chosen)', Fp + np.array([0.10, 0.02]), fontsize=9)
    ax.annotate('$F_i^{-}$ (folded mode)', Fm + np.array([0.0, 0.05]), fontsize=8.5,
                color='0.55', ha='center')
    ax.annotate('$E_i$ (platform attach.)', E + np.array([0.0, -0.05]), fontsize=9,
                color='#1f5fbf', ha='center')
    ax.annotate('$L$', (B + Fp) / 2 + np.array([-0.03, 0.03]), fontsize=11)
    ax.annotate('$l$', (Fp + E) / 2 + np.array([0.02, 0.03]), fontsize=11, color='#a0522d')

    # theta arc at B from horizontal down to bicep
    ax.add_patch(Arc(B, 0.16, 0.16, angle=0, theta1=-42, theta2=0,
                     color='#b22222', lw=1.6))
    ax.annotate(r'$\theta_i$', B + np.array([0.10, -0.045]), fontsize=11, color='#b22222')

    ax.set_title('One leg in its vertical plane:\n'
                 r'$\|F_i-E_i\|=l$  is  circle ∩ circle  →  2 elbow solutions',
                 fontsize=10)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_xlim(-0.18, 0.62); ax.set_ylim(-0.58, 0.20)


def panel_top(ax):
    phis = np.radians([0, 120, 240])
    # base circle and platform circle (top view, x-y)
    circle(ax, (0, 0), R, color='0.5', lw=1.5)
    circle(ax, (p[0], p[1]), r, color='#1f5fbf', lw=1.5)
    ax.plot(0, 0, 'k+', ms=8)
    ax.plot(p[0], p[1], 'o', color='#1f5fbf', ms=6, zorder=6)
    ax.annotate('platform\ncenter $p$', (p[0] + 0.02, p[1] - 0.09), fontsize=8.5,
                color='#1f5fbf')
    for i, phi in enumerate(phis):
        u = np.array([np.cos(phi), np.sin(phi)])
        B = R * u
        E = p[:2] + r * u
        # radial vertical-plane line
        ax.plot([0, 0.34 * u[0]], [0, 0.34 * u[1]], '--', color='0.7', lw=1)
        ax.plot(*B, 'o', color='#b22222', mec='k', ms=10, zorder=6)
        ax.plot(*E, 'o', color='#1f5fbf', mec='k', ms=7, zorder=6)
        ax.plot([B[0], E[0]], [B[1], E[1]], color='#d2691e', lw=2.5, zorder=4)
        ax.annotate(f'leg {i+1}', B * 1.28, fontsize=9, ha='center', color='#b22222')
    ax.annotate('each leg lives in its own radial\nvertical plane → solved independently',
                (0, -0.30), ha='center', fontsize=9)
    ax.set_title('Top view: 3 legs at 120°', fontsize=10)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_xlim(-0.32, 0.32); ax.set_ylim(-0.37, 0.32)


fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.3), gridspec_kw={'width_ratios': [1.5, 1]})
panel_leg(axes[0]); panel_top(axes[1])
fig.suptitle('Delta robot inverse kinematics:  position $p$ → motor angles $\\theta_i$  '
             '(closed form, per leg)', fontsize=12.5, y=1.0)
fig.tight_layout()
out = __file__.rsplit('/', 1)[0] + '/07a_delta_ik.png'
fig.savefig(out, dpi=130, bbox_inches='tight')
print('wrote', out)
