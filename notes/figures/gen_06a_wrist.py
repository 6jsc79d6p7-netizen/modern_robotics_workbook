"""6a: the spherical (orthogonal) wrist.

Panel A — three revolute axes (joints 4,5,6) intersecting at ONE point, the
          wrist center. Drawn as a gimbal: 3 rings in orthogonal planes. Any
          combination of the three rotations re-orients the tool but leaves the
          wrist center fixed -> a 3-DOF "ball joint".
Panel B — why position/orientation decouple: joints 1-3 (the arm) place the
          wrist center; joints 4-6 (the wrist) set orientation. From a desired
          gripper frame, back off along the approach axis by d to get the wrist
          center, then solve the arm for position alone.
"""
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(12.5, 5.6))

# =================================================================== Panel A
axA = fig.add_subplot(1, 2, 1, projection='3d')

def ring(plane, R=1.0, n=120):
    t = np.linspace(0, 2*np.pi, n)
    z = np.zeros_like(t)
    if plane == 'xy':  return R*np.cos(t), R*np.sin(t), z
    if plane == 'yz':  return z, R*np.cos(t), R*np.sin(t)
    if plane == 'xz':  return R*np.cos(t), z, R*np.sin(t)

# three gimbal rings, each rotating about one axis
rings = [('xy', '#1f77b4', 'joint 4  (ω̂₄ = ẑ)', (0, 0, 1)),
         ('xz', '#2ca02c', 'joint 5  (ω̂₅ = ŷ)', (0, 1, 0)),
         ('yz', '#d62728', 'joint 6  (ω̂₆ = x̂)', (1, 0, 0))]
for plane, c, lab, ax in rings:
    xs, ys, zs = ring(plane, R=1.0)
    axA.plot(xs, ys, zs, color=c, lw=2.5)
    # the rotation axis (normal to that ring's plane)
    a = np.array(ax, float)
    axA.plot(*np.array([-1.5*a, 1.5*a]).T, color=c, lw=1.2, ls=(0, (5, 4)))
labpos = {'joint 4  (ω̂₄ = ẑ)': (0, 0, 1.7),
          'joint 5  (ω̂₅ = ŷ)': (0, 1.55, 0.15),
          'joint 6  (ω̂₆ = x̂)': (1.15, 0, -0.42)}
for plane, c, lab, ax in rings:
    axA.text(*labpos[lab], lab, color=c, fontsize=9, ha='center')

# wrist center
axA.scatter([0], [0], [0], color='k', s=55, zorder=10)
axA.text(-0.15, 0, -0.42, 'wrist center', fontsize=9)

# the tool/gripper sticking out along +x from the wrist center
tool = np.array([[0, 0, 0], [1.9, 0, 0]])
axA.plot(tool[:, 0], tool[:, 1], tool[:, 2], color='0.3', lw=4)
for dy in (-0.18, 0.18):                       # little gripper fingers
    axA.plot([1.9, 2.25], [0, dy], [0, 0], color='0.3', lw=4)
axA.text(2.15, 0, 0.30, 'tool', fontsize=9, color='0.3')

axA.set_title('A.  Spherical wrist: 3 axes meet at one point\n'
              '(a 3-DOF ball joint — full orientation, fixed center)',
              fontsize=10)
axA.set_box_aspect((1, 1, 1)); axA.set_axis_off()
axA.view_init(elev=22, azim=-58)
lim = 1.7
axA.set_xlim(-lim, lim+0.6); axA.set_ylim(-lim, lim); axA.set_zlim(-lim, lim)

# =================================================================== Panel B
axB = fig.add_subplot(1, 2, 2, projection='3d')

base = np.array([0, 0, 0])
shoulder = np.array([0, 0, 1.0])
elbow = np.array([1.2, 0, 1.6])
wc = np.array([2.3, 0, 1.1])         # wrist center
# gripper sits offset from wrist center along the tool/approach axis
approach = np.array([0.9, 0, -0.45]); approach = approach/np.linalg.norm(approach)
d = 0.8
grip = wc + d*approach

# the arm: joints 1-3 (base -> shoulder -> elbow -> wrist center)
arm = np.array([base, shoulder, elbow, wc])
axB.plot(arm[:, 0], arm[:, 1], arm[:, 2], '-o', color='#1f77b4', lw=4, ms=7)
# the tool: wrist center -> gripper
axB.plot([wc[0], grip[0]], [wc[1], grip[1]], [wc[2], grip[2]],
         '-', color='#d62728', lw=4)
axB.scatter(*wc, color='k', s=55, zorder=10)
axB.scatter(*grip, color='#d62728', s=45, zorder=10)

# back-off annotation: from desired gripper, step -d*approach to reach wc
mid = (wc + grip)/2
axB.text(mid[0]-0.15, mid[1], mid[2]-0.30, 'back off d\nalong approach axis',
         fontsize=8.5, color='#d62728', ha='right')
axB.text(wc[0]-0.15, wc[1], wc[2]+0.22, 'wrist center', fontsize=9)
axB.text(grip[0]-0.05, grip[1], grip[2]-0.18, 'desired\ngripper pose', fontsize=8.5,
         color='#d62728', ha='center')
axB.text(0.55, 0, 1.5, 'joints 1–3\n(arm → position)', fontsize=8.5,
         color='#1f77b4')

axB.set_title('B.  Decoupling: arm sets the center, wrist sets orientation\n'
              'wrist-center position depends ONLY on joints 1–3',
              fontsize=10)
axB.set_box_aspect((1, 1, 1)); axB.set_axis_off()
axB.view_init(elev=18, azim=-72)
axB.set_xlim(0, 2.8); axB.set_ylim(-1.4, 1.4); axB.set_zlim(0, 2.0)

plt.tight_layout()
out = __file__.rsplit('/', 1)[0] + '/06a_spherical_wrist.png'
plt.savefig(out, dpi=135, bbox_inches='tight')
print('wrote', out)
