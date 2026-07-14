"""13a helper: a driven mecanum wheel transmits force along the roller axle (45 deg diagonal)."""
import numpy as np
import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.8))

def draw(ax, gamma_deg, title):
    g = np.deg2rad(gamma_deg)
    slide_dir = np.array([-np.sin(g), np.cos(g)])   # free rolling of roller -> NO force
    force_dir = np.array([np.cos(g), np.sin(g)])    # roller axle -> force transmitted here
    ax.set_aspect('equal'); ax.set_xlim(-1.7,1.9); ax.set_ylim(-1.4,1.9); ax.axis('off')
    ax.set_title(title, fontsize=12.5, fontweight='bold')

    # wheel body (rectangle, driving = up = x_w)
    ax.add_patch(plt.Rectangle((-0.28,-0.9),0.56,1.8, fc='#e5e7eb', ec='0.4', lw=1.5))
    # rollers as short segments along the roller axle direction, hatched across the rim
    for yy in np.linspace(-0.72,0.72,5):
        p=np.array([0,yy]); d=0.26*force_dir
        ax.plot([p[0]-d[0],p[0]+d[0]],[p[1]-d[1],p[1]+d[1]], color='#c2410c', lw=4, solid_capstyle='round')

    # driving direction (motor spins wheel -> tries to roll forward = +x_w = up)
    ax.annotate('', xy=(0,1.55), xytext=(0,0.95), arrowprops=dict(arrowstyle='-|>',color='#1d4ed8',lw=2.4))
    ax.text(0.06,1.5,'motor drives\n(rolls "forward")',color='#1d4ed8',fontsize=10)

    # free-slide direction (dashed) - NO force
    t=np.linspace(-1.15,1.15,2)
    ax.plot(t*slide_dir[0], t*slide_dir[1], color='0.55', lw=2, ls='--')
    ax.text(1.2*slide_dir[0]-0.5, 1.2*slide_dir[1]+0.05, 'free-slide dir\n(no force)', color='0.45', fontsize=9.5)

    # FORCE on the robot (solid red thick) along roller axle
    F=1.3*force_dir
    ax.annotate('', xy=(F[0],F[1]), xytext=(0,0), arrowprops=dict(arrowstyle='-|>',color='#b91c1c',lw=3.6))
    ax.text(F[0]+0.02,F[1]+0.02, f'traction FORCE\non robot ({gamma_deg}° diag)', color='#b91c1c',
            fontsize=10.5, fontweight='bold')

    # right-angle tick between slide and force
    ax.plot([0.16*slide_dir[0]+0.16*force_dir[0]],[0.16*slide_dir[1]+0.16*force_dir[1]],'k',marker='')
    ax.text(-1.6,-1.25,'force is perpendicular to free-slide', fontsize=10, style='italic', color='0.3')

draw(axes[0], 45,  'Roller γ = +45°  →  force pushes up-right')
draw(axes[1], -45, 'Roller γ = −45°  →  force pushes up-left')
fig.suptitle('A driven mecanum wheel pushes the robot along the ROLLER AXLE — a 45° diagonal, not straight forward',
             fontsize=12.5, fontweight='bold', y=0.99)
fig.tight_layout()
import os
out=os.path.join(os.path.dirname(__file__),'13a_wheel_force.png')
fig.savefig(out,dpi=130,bbox_inches='tight'); print('wrote',out)
