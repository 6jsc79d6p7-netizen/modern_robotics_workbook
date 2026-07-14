"""11b figure: impedance = the force/motion slope you get to CHOOSE.

Left: restoring force vs displacement for three stiffnesses. Impedance control
lets you pick this slope K. A stiff (position-like) controller is a near-vertical
line — tiny motion makes huge force → dangerous on contact. A compliant
controller is a gentle slope — push it and it gives.

Right: a step in the target position, showing how a low-stiffness / well-damped
impedance responds softly vs a stiff one that slams to the target.
"""
import numpy as np
import matplotlib.pyplot as plt

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.4))

# ---- Left: force vs displacement (the "spring you command") ----
x = np.linspace(-0.05, 0.05, 200)  # displacement from target, meters
for K, c, lab in [(4000, "#c0392b", "stiff  K=4000 (position-like)"),
                  (800, "#e67e22", "medium K=800"),
                  (150, "#27ae60", "compliant K=150")]:
    ax1.plot(x * 100, K * x, color=c, lw=2.5, label=lab)
# mark the force at 2 cm penetration
xp = 0.02
for K, c in [(4000, "#c0392b"), (800, "#e67e22"), (150, "#27ae60")]:
    ax1.plot([xp * 100], [K * xp], "o", color=c, ms=6)
ax1.axvline(0, color="gray", lw=0.8, ls=":")
ax1.axhline(0, color="gray", lw=0.8, ls=":")
ax1.annotate("2 cm into a\nsurface", xy=(2, 4000 * 0.02), xytext=(2.4, 55),
             fontsize=8.5, color="#c0392b",
             arrowprops=dict(arrowstyle="->", color="#c0392b", lw=1))
ax1.set_xlabel("displacement from target  (cm)")
ax1.set_ylabel("restoring / contact force  (N)")
ax1.set_title("Impedance = the force/motion SLOPE you choose\n(slope = stiffness K)",
              fontsize=10.5)
ax1.legend(fontsize=8, loc="upper left")
ax1.set_ylim(-90, 90)
ax1.grid(alpha=0.25)

# ---- Right: step response of a rendered mass-spring-damper ----
t = np.linspace(0, 1.2, 600)
def step_resp(K, B, M=1.0):
    wn = np.sqrt(K / M)
    zeta = B / (2 * np.sqrt(K * M))
    if zeta < 1:
        wd = wn * np.sqrt(1 - zeta**2)
        return 1 - np.exp(-zeta * wn * t) * (np.cos(wd * t) + (zeta * wn / wd) * np.sin(wd * t))
    else:
        return 1 - np.exp(-wn * t) * (1 + wn * t)

# stiff: high K, critically damped -> fast slam
ax2.plot(t, step_resp(400, 2*np.sqrt(400)), color="#c0392b", lw=2.3,
         label="stiff impedance (slams to target)")
# compliant: low K, critically damped -> soft approach
ax2.plot(t, step_resp(40, 2*np.sqrt(40)), color="#27ae60", lw=2.3,
         label="compliant impedance (eases in, forgiving)")
ax2.axhline(1, color="gray", lw=0.8, ls=":")
ax2.set_xlabel("time  (s)")
ax2.set_ylabel("end-effector position")
ax2.set_title("Same target, different rendered stiffness\n(the arm 'feels' soft or rigid)",
              fontsize=10.5)
ax2.legend(fontsize=8.5, loc="lower right")
ax2.grid(alpha=0.25)

fig.tight_layout()
fig.savefig(__file__.replace("gen_", "").replace(".py", ".png"), dpi=130)
print("wrote", __file__.replace("gen_", "").replace(".py", ".png"))
