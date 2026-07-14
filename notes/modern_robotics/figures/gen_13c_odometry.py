"""13c fig 1: odometry pipeline + why you integrate an arc, not add velocities."""
import numpy as np
import matplotlib.pyplot as plt

fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5.4))

def draw_car(ax, x, y, phi, L=0.16, W=0.1, color='#374151', alpha=1.0, lw=1.4):
    c, s = np.cos(phi), np.sin(phi)
    R = np.array([[c, -s], [s, c]])
    corners = np.array([[-L,-W],[L,-W],[L,W],[-L,W]])
    pts = (R @ corners.T).T + [x, y]
    ax.add_patch(plt.Polygon(pts, closed=True, fill=True, fc=color, ec='k', lw=lw, alpha=alpha))
    ax.annotate('', xy=(x+(L+0.09)*c, y+(L+0.09)*s), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=color, lw=1.6, alpha=alpha))

# ---------- LEFT: pipeline ----------
axL.axis('off'); axL.set_xlim(0,1); axL.set_ylim(0,1)
axL.set_title('Odometry pipeline — every tick', fontsize=13, fontweight='bold')
boxes = [
    (0.5, 0.86, 'Δθ  (encoder increments)', '#e0e7ff', '#3730a3'),
    (0.5, 0.63, 'V_b = F·Δθ\n(F = H†, pseudoinverse — Ch 6b)', '#dcfce7', '#166534'),
    (0.5, 0.38, 'integrate screw:  T_bb′ = e^[V_b6]\n→ Δq_b  (arc if ω≠0 — Ch 3b)', '#fef9c3', '#854d0e'),
    (0.5, 0.13, 'rotate by φ_k → Δq;   q_{k+1}=q_k+Δq', '#fee2e2', '#991b1b'),
]
for (x,y,txt,fc,ec) in boxes:
    axL.add_patch(plt.Rectangle((x-0.42,y-0.075),0.84,0.15, fc=fc, ec=ec, lw=2))
    axL.text(x, y, txt, ha='center', va='center', fontsize=10.5, color=ec, fontweight='bold')
for y in [0.785, 0.555, 0.305]:
    axL.annotate('', xy=(0.5,y-0.02), xytext=(0.5,y+0.02),
                 arrowprops=dict(arrowstyle='-|>', color='0.4', lw=2))

# ---------- RIGHT: arc vs straight ----------
axR.set_aspect('equal'); axR.set_xlim(-0.3, 2.5); axR.set_ylim(-0.4, 1.9); axR.axis('off')
axR.set_title('Why integrate an ARC, not add velocities', fontsize=13, fontweight='bold')

# start pose
x0,y0,phi0 = 0.0, 0.2, 0.35
draw_car(axR, x0, y0, phi0, color='#9ca3af')
axR.text(x0-0.1, y0-0.28, 'start', fontsize=10, color='#374151', fontweight='bold')

# TRUE arc (omega != 0): robot turns while moving
w = 1.0; vbx = 1.4
# integrate
xs=[x0]; ys=[y0]; ph=phi0; x,y=x0,y0
for _ in range(60):
    dt=1.0/60; ph+=w*dt; x+=vbx*np.cos(ph)*dt; y+=vbx*np.sin(ph)*dt
    xs.append(x); ys.append(y)
axR.plot(xs, ys, color='#166534', lw=2.6, label='true path (arc)')
draw_car(axR, xs[-1], ys[-1], ph, color='#16a34a')
axR.text(xs[-1]-0.1, ys[-1]+0.16, 'true end\n(exponentiate twist)', fontsize=9.5, color='#166534', fontweight='bold')

# WRONG: just add linear velocity along initial heading (straight)
xe_w = x0 + vbx*np.cos(phi0); ye_w = y0 + vbx*np.sin(phi0)
axR.plot([x0, xe_w],[y0, ye_w], color='#b91c1c', lw=2.2, ls='--', label='wrong (add velocity)')
draw_car(axR, xe_w, ye_w, phi0, color='#ef4444', alpha=0.55)
axR.text(xe_w-0.15, ye_w-0.3, 'wrong end\n(ignored turning)', fontsize=9.5, color='#b91c1c', fontweight='bold')

# error gap
axR.annotate('', xy=(xs[-1],ys[-1]), xytext=(xe_w,ye_w),
             arrowprops=dict(arrowstyle='<->', color='0.4', lw=1.4, ls=':'))
axR.text((xs[-1]+xe_w)/2+0.05, (ys[-1]+ye_w)/2-0.05, 'drift', fontsize=9.5, color='0.35', style='italic')
axR.legend(loc='lower right', fontsize=9.5, frameon=True)

fig.tight_layout()
import os
out=os.path.join(os.path.dirname(__file__),'13c_odometry.png')
fig.savefig(out, dpi=130, bbox_inches='tight'); print('wrote', out)
