"""13a helper figure: how the wheel-center velocity v splits into driven + free-slide."""
import numpy as np
import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 2, figsize=(12, 5.6))

def draw(ax, gamma_deg, v, title):
    g = np.deg2rad(gamma_deg)
    drive_dir = np.array([1.0, 0.0])                 # x_w
    slide_dir = np.array([-np.sin(g), np.cos(g)])    # roller slide direction
    # solve v = v_drive*drive_dir + v_slide*slide_dir
    M = np.column_stack([drive_dir, slide_dir])
    v_drive, v_slide = np.linalg.solve(M, v)

    ax.set_aspect('equal'); ax.set_xlim(-1.4, 2.2); ax.set_ylim(-0.7, 2.3)
    ax.axhline(0, color='0.85', lw=1); ax.axvline(0, color='0.85', lw=1)
    ax.set_title(title, fontsize=12.5, fontweight='bold')

    # wheel frame axes
    ax.annotate('', xy=(1.6,0), xytext=(0,0), arrowprops=dict(arrowstyle='-|>',color='0.35',lw=1.6))
    ax.annotate('', xy=(0,1.6), xytext=(0,0), arrowprops=dict(arrowstyle='-|>',color='0.35',lw=1.6))
    ax.text(1.62,-0.02,'x̂_w  (driving dir)',fontsize=10,color='0.35',va='top')
    ax.text(0.03,1.63,'ŷ_w (sideways)',fontsize=10,color='0.35')

    # roller / slide line through origin
    t=np.linspace(-1.2,1.4,2)
    ax.plot(t*slide_dir[0], t*slide_dir[1], color='#c2410c', lw=2, ls='--')
    ax.text(1.35*slide_dir[0]-0.15, 1.35*slide_dir[1], f'slide dir (γ={gamma_deg}°)',
            color='#c2410c', fontsize=10)

    # the actual hub velocity v (green)
    ax.annotate('', xy=(v[0],v[1]), xytext=(0,0),
                arrowprops=dict(arrowstyle='-|>',color='#059669',lw=3.2))
    ax.text(v[0]+0.05, v[1]+0.05, f'v = ({v[0]:.0f},{v[1]:.0f})\n(hub velocity)',
            color='#059669', fontsize=11, fontweight='bold')

    # decomposition parallelogram: driven part then slide part
    dpt = v_drive*drive_dir
    ax.annotate('', xy=(dpt[0],dpt[1]), xytext=(0,0),
                arrowprops=dict(arrowstyle='-|>',color='#1d4ed8',lw=2.6))
    ax.annotate('', xy=(v[0],v[1]), xytext=(dpt[0],dpt[1]),
                arrowprops=dict(arrowstyle='-|>',color='#c2410c',lw=2.6))
    ax.text(dpt[0]/2, -0.16, f'v_drive={v_drive:.2f}\n(motor)', color='#1d4ed8',
            fontsize=10.5, ha='center', fontweight='bold')
    ax.text((dpt[0]+v[0])/2+0.06, (dpt[1]+v[1])/2, f'v_slide={v_slide:.2f}\n(free)',
            color='#c2410c', fontsize=10.5, fontweight='bold')

axv = np.array([0.0, 1.0])
draw(axes[0], 0,  axv, 'Omniwheel (γ=0): strafe → motor OFF\nv_drive = v_x = 0')
draw(axes[1], 45, axv, 'Mecanum (γ=45°): strafe → motor SPINS\nv_drive = v_x + v_y·tan45° = 1')

fig.suptitle('Same sideways hub velocity v=(0,1), decomposed into driven (blue) + free-slide (orange)',
             fontsize=12.5, fontweight='bold', y=1.0)
fig.tight_layout()
import os
out=os.path.join(os.path.dirname(__file__),'13a_wheel_decomp.png')
fig.savefig(out,dpi=130,bbox_inches='tight'); print('wrote',out)
