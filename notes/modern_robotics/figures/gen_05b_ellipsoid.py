"""Figures for note 05b — manipulability & force ellipsoids of a 2R planar arm.

The manipulability ellipse is the image of the unit circle of joint rates
||theta_dot||=1 mapped through J: an ellipse whose axes are the eigenvectors of
A = J J^T and whose semi-axis lengths are sqrt(eigenvalues). Round ellipse =
easy to move every direction (far from singular); thin sliver = near singular.

The force ellipse is its reciprocal (axes 1/sqrt(eigenvalue)): easy-to-push
directions are exactly the hard-to-move directions, and vice versa.

Left  : well-conditioned posture (theta2 = 90 deg) -> fat ellipse.
Right : near-singular posture   (theta2 = 15 deg) -> thin ellipse, blowing-up force.
"""
import numpy as np
import matplotlib.pyplot as plt

L1 = L2 = 1.0


def fk(t1, t2):
    p0 = np.array([0.0, 0.0])
    p1 = p0 + L1 * np.array([np.cos(t1), np.sin(t1)])
    p2 = p1 + L2 * np.array([np.cos(t1 + t2), np.sin(t1 + t2)])
    return p0, p1, p2


def jac(t1, t2):
    return np.array([
        [-L1 * np.sin(t1) - L2 * np.sin(t1 + t2), -L2 * np.sin(t1 + t2)],
        [L1 * np.cos(t1) + L2 * np.cos(t1 + t2),  L2 * np.cos(t1 + t2)],
    ])


def ellipse_pts(center, axes_dirs, axes_len, n=200):
    th = np.linspace(0, 2 * np.pi, n)
    circle = np.vstack([np.cos(th), np.sin(th)])
    M = axes_dirs @ np.diag(axes_len)        # columns = scaled principal axes
    pts = (M @ circle) + center[:, None]
    return pts


def draw(ax, t1, t2, title):
    p0, p1, p2 = fk(t1, t2)
    arm = np.array([p0, p1, p2])
    ax.plot(arm[:, 0], arm[:, 1], '-o', color='0.4', lw=4, ms=7, zorder=2)
    ax.plot(*p0, 'k^', ms=11, zorder=3)

    J = jac(t1, t2)
    A = J @ J.T
    w, V = np.linalg.eigh(A)          # eigenvalues w (ascending), eigenvectors V (columns)
    # manipulability ellipse: semi-axes sqrt(eigenvalue) along eigenvectors
    man = ellipse_pts(p2, V, np.sqrt(w))
    ax.fill(man[0], man[1], color='tab:blue', alpha=0.18, zorder=1)
    ax.plot(man[0], man[1], color='tab:blue', lw=2, label='velocity (manipulability)')
    # force ellipse: semi-axes 1/sqrt(eigenvalue) (reciprocal), same axes
    frc = ellipse_pts(p2, V, 1.0 / np.sqrt(w))
    ax.plot(frc[0], frc[1], color='tab:red', lw=2, ls='--', label='force')

    # principal-axis arrows of the manipulability ellipse
    for i in range(2):
        d = V[:, i] * np.sqrt(w[i])
        ax.annotate('', xy=p2 + d, xytext=p2,
                    arrowprops=dict(arrowstyle='-|>', color='tab:blue', lw=1.8))

    ratio = np.sqrt(w[-1] / w[0])     # mu1 = sqrt(lambda_max/lambda_min)
    ax.set_title(f'{title}\n' r'$\mu_1=\sqrt{\lambda_{max}/\lambda_{min}}=$'
                 f'{ratio:.2f}  (1 = isotropic, $\\infty$ = singular)',
                 fontsize=11)
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlim(-2.4, 2.6); ax.set_ylim(-2.4, 2.6)
    ax.legend(loc='lower left', fontsize=8.5)


fig, axes = plt.subplots(1, 2, figsize=(12.5, 6.2))
draw(axes[0], np.deg2rad(30), np.deg2rad(90), 'Well-conditioned  ($\\theta_2=90°$)')
draw(axes[1], np.deg2rad(30), np.deg2rad(15), 'Near-singular  ($\\theta_2=15°$)')
fig.suptitle('Manipulability (blue) vs force (red) ellipses — reciprocal shapes',
             fontsize=13, fontweight='bold')
fig.tight_layout()
fig.savefig('05b_manipulability_ellipse.png', dpi=130, bbox_inches='tight')
print('wrote 05b_manipulability_ellipse.png')
