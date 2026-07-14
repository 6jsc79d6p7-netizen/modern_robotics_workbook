"""13c fig 2: mobile-manipulator frame tree {s}->{b}->{0}->{e} + combined Jacobian block."""
import numpy as np
import matplotlib.pyplot as plt

fig, (axL, axR) = plt.subplots(1, 2, figsize=(13.5, 5.6), gridspec_kw={'width_ratios':[1.25,1]})

# ---------- LEFT: frame tree ----------
axL.set_aspect('equal'); axL.set_xlim(-0.6, 5.2); axL.set_ylim(-0.6, 4.2); axL.axis('off')
axL.set_title('Mobile manipulator: T_se = T_sb · T_b0 · T_0e', fontsize=13, fontweight='bold')

def frame(ax, o, ang=0, s=0.55, label='', lc='k', dx=0.12, dy=0.12):
    c,si=np.cos(ang),np.sin(ang)
    ax.annotate('', xy=(o[0]+s*c, o[1]+s*si), xytext=o, arrowprops=dict(arrowstyle='-|>',color=lc,lw=2))
    ax.annotate('', xy=(o[0]-s*si, o[1]+s*c), xytext=o, arrowprops=dict(arrowstyle='-|>',color=lc,lw=2))
    ax.text(o[0]+dx, o[1]+dy, label, fontsize=12, fontweight='bold', color=lc)

# ground
axL.plot([-0.4,5.0],[0,0], color='0.7', lw=1.5)
# {s} world
frame(axL, (0.1,0.1), 0, label='{s}', lc='#1f2937', dx=-0.35, dy=0.05)
# base chassis
bx,by=2.4,0.55
axL.add_patch(plt.Rectangle((bx-0.7,by-0.35),1.4,0.5, fc='#e5e7eb', ec='0.4', lw=1.5))
for wx in [bx-0.5,bx+0.5]:
    axL.add_patch(plt.Circle((wx,by-0.35),0.16, fc='#374151'))
frame(axL, (bx,by+0.2), 0.12, s=0.5, label='{b}', lc='#2563eb', dx=-0.15, dy=0.12)
# arm base {0}
ox,oy=bx+0.2, by+0.2
frame(axL, (ox,oy), 0.12, s=0.42, label='{0}', lc='#7c3aed', dx=0.1, dy=-0.28)
# arm links
j1=np.array([ox,oy]); L1,L2=1.3,1.1; a1,a2=1.1,-0.55
j2=j1+L1*np.array([np.cos(a1),np.sin(a1)])
je=j2+L2*np.array([np.cos(a1+a2),np.sin(a1+a2)])
axL.plot(*zip(j1,j2), color='#111827', lw=5, solid_capstyle='round')
axL.plot(*zip(j2,je), color='#111827', lw=5, solid_capstyle='round')
for j in [j1,j2]:
    axL.add_patch(plt.Circle(j,0.09, fc='#f59e0b', ec='k', zorder=5))
frame(axL, je, a1+a2, s=0.5, label='{e}', lc='#059669', dx=0.12, dy=0.05)

# transform labels along the chain
axL.annotate('', xy=(bx-0.7,by+0.05), xytext=(0.6,0.3), arrowprops=dict(arrowstyle='-|>',color='#2563eb',lw=1.6,ls='--'))
axL.text(1.0,0.65,'T_sb(q)\nbase pose', fontsize=9.5, color='#2563eb', fontweight='bold')
axL.text(bx+0.02,by+0.75,'T_b0\n(fixed)', fontsize=9, color='#7c3aed', fontweight='bold')
axL.text((j2[0]+je[0])/2+0.1,(j2[1]+je[1])/2,'T_0e(θ)\narm FK', fontsize=9.5, color='#111827', fontweight='bold')

# ---------- RIGHT: combined Jacobian block ----------
axR.axis('off'); axR.set_xlim(0,1); axR.set_ylim(0,1)
axR.set_title('One combined Jacobian for the whole machine', fontsize=13, fontweight='bold')

axR.text(0.5, 0.9, r'$\mathcal{V}_e = J_e(\theta)\,[\,u\ ;\ \dot\theta\,]$',
         ha='center', fontsize=15, fontweight='bold')

# the block matrix
axR.add_patch(plt.Rectangle((0.12,0.5),0.32,0.28, fc='#dbeafe', ec='#1e40af', lw=2))
axR.add_patch(plt.Rectangle((0.44,0.5),0.32,0.28, fc='#dcfce7', ec='#166534', lw=2))
axR.text(0.28,0.64,'J_base(θ)\n6×m', ha='center', va='center', fontsize=11, color='#1e40af', fontweight='bold')
axR.text(0.60,0.64,'J_arm(θ)\n6×n', ha='center', va='center', fontsize=11, color='#166534', fontweight='bold')
axR.text(0.28,0.83,'wheels  u', ha='center', fontsize=10, color='#1e40af', fontweight='bold')
axR.text(0.60,0.83,'joints  θ̇', ha='center', fontsize=10, color='#166534', fontweight='bold')

axR.text(0.28,0.40,'= [Ad] · F₆\n(base twist →\n{e} frame, Ch 3b)', ha='center', va='top', fontsize=9.5, color='#1e40af')
axR.text(0.60,0.40,'= body Jacobian\nJ_b(θ)  (Ch 5)\nunchanged!', ha='center', va='top', fontsize=9.5, color='#166534')

# invert
axR.text(0.5, 0.14, r'control / IK:  $[\,u;\dot\theta\,] = J_e^{\dagger}\,\mathcal{V}$',
         ha='center', fontsize=13, fontweight='bold', color='#7c2d12')
axR.text(0.5, 0.04, 'redundant (m+n>6): weighted pinv trades base↔arm effort (Ch 6b)',
         ha='center', fontsize=9.5, style='italic', color='0.3')

fig.tight_layout()
import os
out=os.path.join(os.path.dirname(__file__),'13c_mobile_manip.png')
fig.savefig(out, dpi=130, bbox_inches='tight'); print('wrote', out)
