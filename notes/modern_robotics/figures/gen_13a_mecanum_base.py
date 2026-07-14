"""13a figure: wheel types (top) + 4-mecanum base with its 3 motion primitives (bottom)."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow

fig = plt.figure(figsize=(11, 8.5))
gs = fig.add_gridspec(2, 3, height_ratios=[1, 1.35], hspace=0.35, wspace=0.25)

# ---------- Top row: three wheel types ----------
def draw_wheel(ax, title, roller_angle_deg, slides, conventional=False):
    ax.set_aspect('equal'); ax.set_xlim(-1.6, 1.6); ax.set_ylim(-1.7, 1.7)
    ax.axis('off'); ax.set_title(title, fontsize=12, fontweight='bold', pad=6)
    # wheel as a circle (side view rim)
    th = np.linspace(0, 2*np.pi, 100)
    ax.plot(np.cos(th), np.sin(th), 'k-', lw=2.2)
    ax.plot(0.5*np.cos(th), 0.5*np.sin(th), color='0.6', lw=1)
    # rollers around rim
    if not conventional:
        for a in np.linspace(0, 2*np.pi, 12, endpoint=False):
            cx, cy = np.cos(a), np.sin(a)
            ra = np.deg2rad(roller_angle_deg)
            dx, dy = 0.22*np.cos(ra), 0.22*np.sin(ra)
            ax.plot([cx-dx, cx+dx], [cy-dy, cy+dy], color='#c2410c', lw=3, solid_capstyle='round')
    # driving direction (forward = +x here)
    ax.annotate('', xy=(1.35, 0), xytext=(0.0, 0),
                arrowprops=dict(arrowstyle='-|>', color='#1d4ed8', lw=2.5))
    ax.text(1.15, 0.22, 'drive', color='#1d4ed8', fontsize=10, fontweight='bold')
    # slide direction
    if slides:
        sa = np.deg2rad(roller_angle_deg + 90)
        ax.annotate('', xy=(1.15*np.cos(sa), 1.15*np.sin(sa)),
                    xytext=(-1.15*np.cos(sa), -1.15*np.sin(sa)),
                    arrowprops=dict(arrowstyle='<|-|>', color='#059669', lw=2))
        ax.text(-1.5, -1.55, 'free slide', color='#059669', fontsize=10, fontweight='bold')
    else:
        ax.plot([0,0],[0,0])
        ax.text(-1.5, -1.55, 'NO sideways slip', color='#b91c1c', fontsize=10, fontweight='bold')

ax1 = fig.add_subplot(gs[0, 0]); draw_wheel(ax1, 'Conventional', 0, slides=False, conventional=True)
ax2 = fig.add_subplot(gs[0, 1]); draw_wheel(ax2, 'Omniwheel  (γ = 0)', 0, slides=True)
ax3 = fig.add_subplot(gs[0, 2]); draw_wheel(ax3, 'Mecanum  (γ = ±45°)', 45, slides=True)

# ---------- Bottom: mecanum base + 3 primitives ----------
# youBot-like geometry
L, W, rw = 0.95, 0.6, 0.16   # half-length, half-width, wheel drawing radius
wheel_pos = {1:(-L,  W), 2:( L,  W), 3:( L, -W), 4:(-L, -W)}  # xb along +x (up in plot? use xb=up)
# We'll draw with xb pointing UP for a top-view "forward". Map body (x,y) -> plot (y_left, x_up)
def to_plot(bx, by):  # body x -> up, body y -> left
    return (-by, bx)

def draw_base(ax, title, drive_signs, motion_arrow):
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_title(title, fontsize=12, fontweight='bold', pad=4)
    ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.7, 1.9)
    # chassis
    cw = [to_plot(bx, by) for bx, by in [(-L-0.35, W+0.35),(L+0.35,W+0.35),(L+0.35,-W-0.35),(-L-0.35,-W-0.35)]]
    ax.add_patch(plt.Polygon(cw, closed=True, fill=True, fc='#e5e7eb', ec='0.4', lw=1.5))
    # wheels
    for i,(bx,by) in wheel_pos.items():
        px, py = to_plot(bx, by)
        s = drive_signs[i-1]
        col = '#1d4ed8' if s > 0 else '#b91c1c'
        ax.add_patch(Rectangle((px-0.18, py-0.28), 0.36, 0.56, fc=col, ec='k', lw=1.2, alpha=0.85))
        # roller hatch (45)
        ax.text(px, py+0.42, f'{i}', ha='center', fontsize=10, fontweight='bold')
        # drive arrow (up = +xb forward), sign flips direction
        ay = 0.34*np.sign(s)
        ax.annotate('', xy=(px, py+ay), xytext=(px, py-ay),
                    arrowprops=dict(arrowstyle='-|>', color='white', lw=2))
    # body frame at center
    ax.annotate('', xy=to_plot(0.5,0), xytext=(0,0), arrowprops=dict(arrowstyle='-|>',color='k',lw=1.5))
    ax.annotate('', xy=to_plot(0,0.5), xytext=(0,0), arrowprops=dict(arrowstyle='-|>',color='k',lw=1.5))
    ax.text(*to_plot(0.62,0.05),'x̂_b',fontsize=9); ax.text(*to_plot(0.02,0.66),'ŷ_b',fontsize=9)
    # net motion arrow
    if motion_arrow == 'x':
        ax.annotate('', xy=(0,1.55), xytext=(0,1.0), arrowprops=dict(arrowstyle='-|>',color='#059669',lw=3.5))
    elif motion_arrow == 'y':
        ax.annotate('', xy=(-1.3,-1.35), xytext=(-0.6,-1.35), arrowprops=dict(arrowstyle='-|>',color='#059669',lw=3.5))
    elif motion_arrow == 'rot':
        th=np.linspace(0.2,2.6,40); ax.plot(0.75*np.cos(th),0.75*np.sin(th)-0.0,color='#059669',lw=3)
        ax.annotate('', xy=(0.75*np.cos(2.75),0.75*np.sin(2.75)), xytext=(0.75*np.cos(2.6),0.75*np.sin(2.6)),
                    arrowprops=dict(arrowstyle='-|>',color='#059669',lw=3))

ax4 = fig.add_subplot(gs[1,0]); draw_base(ax4, 'Forward  v_bx\n[+ + + +]', [1,1,1,1], 'x')
ax5 = fig.add_subplot(gs[1,1]); draw_base(ax5, 'Sideways  v_by\n[− + − +]', [-1,1,-1,1], 'y')
ax6 = fig.add_subplot(gs[1,2]); draw_base(ax6, 'Rotate  ω_bz\n[− + + −]', [-1,1,1,-1], 'rot')

fig.text(0.5, 0.955, 'Wheel types: only conventional wheels impose the "no sideways slip" constraint',
         ha='center', fontsize=12.5, fontweight='bold')
fig.text(0.5, 0.585, '4-mecanum base: combining wheel drive signs (blue = fwd, red = back) makes all 3 planar DOF',
         ha='center', fontsize=12.5, fontweight='bold')

import os
out = os.path.join(os.path.dirname(__file__), '13a_mecanum_base.png')
fig.savefig(out, dpi=130, bbox_inches='tight')
print('wrote', out)
