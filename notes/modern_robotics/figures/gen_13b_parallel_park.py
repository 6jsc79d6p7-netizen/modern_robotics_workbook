"""13b figure: (L) the no-sideways-slip constraint, (R) the Lie-bracket parallel-park wiggle."""
import numpy as np
import matplotlib.pyplot as plt

fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5.8))

def draw_car(ax, x, y, phi, L=0.34, W=0.2, color='#374151', alpha=1.0, lw=1.6):
    c, s = np.cos(phi), np.sin(phi)
    R = np.array([[c, -s], [s, c]])
    corners = np.array([[-L,-W],[L,-W],[L,W],[-L,W]])
    pts = (R @ corners.T).T + [x, y]
    ax.add_patch(plt.Polygon(pts, closed=True, fill=True, fc=color, ec='k', lw=lw, alpha=alpha))
    # heading arrow
    ax.annotate('', xy=(x+ (L+0.12)*c, y+(L+0.12)*s), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=color, lw=2, alpha=alpha))

# ---------------- LEFT: constraint ----------------
axL.set_aspect('equal'); axL.set_xlim(-1.6, 1.9); axL.set_ylim(-1.5, 1.7); axL.axis('off')
axL.set_title('The "no sideways slip" constraint', fontsize=13, fontweight='bold')
draw_car(axL, 0, 0, 0, color='#9ca3af')
# allowed: forward/back
axL.annotate('', xy=(1.55, 0), xytext=(0.55, 0), arrowprops=dict(arrowstyle='-|>', color='#059669', lw=3))
axL.annotate('', xy=(-1.15, 0), xytext=(-0.55, 0), arrowprops=dict(arrowstyle='-|>', color='#059669', lw=3))
axL.text(1.0, 0.16, 'drive (allowed)', color='#059669', fontsize=11, fontweight='bold')
# allowed: turn
th = np.linspace(-0.5, 0.9, 30)
axL.plot(0.75*np.cos(th), 0.75*np.sin(th)+0.75, color='#059669', lw=2.5)
axL.annotate('', xy=(0.75*np.cos(0.95), 0.75*np.sin(0.95)+0.75),
             xytext=(0.75*np.cos(0.9), 0.75*np.sin(0.9)+0.75),
             arrowprops=dict(arrowstyle='-|>', color='#059669', lw=2.5))
axL.text(-0.2, 1.5, 'turn (allowed)', color='#059669', fontsize=11, fontweight='bold', ha='center')
# forbidden: sideways
axL.annotate('', xy=(0, 1.25), xytext=(0, 0.35), arrowprops=dict(arrowstyle='-|>', color='#b91c1c', lw=3))
axL.annotate('', xy=(0, -1.1), xytext=(0, -0.35), arrowprops=dict(arrowstyle='-|>', color='#b91c1c', lw=3))
# red X over sideways
axL.plot([-0.22, 0.22], [0.75-0.22, 0.75+0.22], 'r', lw=3)
axL.plot([-0.22, 0.22], [0.75+0.22, 0.75-0.22], 'r', lw=3)
axL.text(0.12, -1.0, 'sideways: FORBIDDEN\n(ẋ sinφ − ẏ cosφ = 0)', color='#b91c1c', fontsize=10.5, fontweight='bold')

# ---------------- RIGHT: wiggle (faithful sim of canonical model) ----------------
axR.set_aspect('equal'); axR.set_xlim(-0.5, 1.5); axR.set_ylim(-1.15, 0.95); axR.axis('off')
axR.set_title('Parallel-park wiggle → net SIDEWAYS shift', fontsize=13, fontweight='bold')

# integrate phi'=w, x'=v cosphi, y'=v sinphi through 4 phases: +fwd,+turn,-fwd,-turn
def integrate(a=0.7, th=0.7, n=60):
    xs, ys, ph = [0.0], [0.0], 0.0
    x, y = 0.0, 0.0
    phases = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    durs = [a, th, a, th]
    keys = [(x, y, ph)]
    for (v, w), T in zip(phases, durs):
        for _ in range(n):
            dt = T / n
            ph += w * dt; x += v*np.cos(ph)*dt; y += v*np.sin(ph)*dt
            xs.append(x); ys.append(y)
        keys.append((x, y, ph))
    return np.array(xs), np.array(ys), keys

xs, ys, keys = integrate()
axR.plot(xs, ys, color='#6366f1', lw=2, alpha=0.7, zorder=1)

# dashed reference line at start y
axR.axhline(0, color='0.7', ls=':', lw=1.2)
axR.axhline(keys[-1][1], color='#166534', ls=':', lw=1.2)

labels = ['start', '1. fwd', '2. turn', '3. back', '4. un-turn (end)']
cols = ['#c7d2fe', '#a5b4fc', '#818cf8', '#6366f1', '#16a34a']
for (x, y, phi), lab, col in zip(keys, labels, cols):
    end = (lab.startswith('4'))
    draw_car(axR, x, y, phi, L=0.19, W=0.12, color=col, alpha=0.95, lw=2.2 if end else 1.3)

axR.text(keys[0][0]-0.02, 0.12, 'start', ha='center', fontsize=10, color='#3730a3', fontweight='bold')
axR.text(keys[-1][0]+0.05, keys[-1][1]-0.28, 'end:\nsame heading,\nmoved sideways!',
         ha='center', fontsize=9.5, color='#166534', fontweight='bold')

# net sideways arrow
axR.annotate('', xy=(-0.32, keys[-1][1]), xytext=(-0.32, 0),
             arrowprops=dict(arrowstyle='-|>', color='#166534', lw=3))
axR.text(-0.47, keys[-1][1]/2, 'net\nΔy', color='#166534', fontsize=11, fontweight='bold', va='center')

# step legend
for i, (lab, col) in enumerate(zip(labels[1:], cols[1:])):
    axR.text(0.92, 0.85 - i*0.22, lab, color=col, fontsize=10, fontweight='bold')
axR.text(0.92, 0.85 - 4*0.22, '(= Lie bracket [g₁,g₂])', color='#166534', fontsize=9.5, style='italic')

fig.suptitle('Nonholonomic base: can\'t slide sideways instantly (left), but wiggles sideways over a maneuver (right)',
             fontsize=12.5, fontweight='bold', y=1.0)
fig.tight_layout()
import os
out = os.path.join(os.path.dirname(__file__), '13b_parallel_park.png')
fig.savefig(out, dpi=130, bbox_inches='tight'); print('wrote', out)
