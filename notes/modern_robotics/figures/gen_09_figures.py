"""Figures for notes/09_trajectory_generation.md.

Run with the project venv:
    ./.venv/bin/python notes/figures/gen_09_figures.py
"""
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

OUT = Path(__file__).parent


# ---------------------------------------------------------------------------
# Figure 1: time-scaling profiles s(t), s'(t), s''(t) for
#   cubic, quintic, trapezoidal, S-curve.  All normalized to T=1, s:0->1.
# ---------------------------------------------------------------------------
def cubic(T, t):
    s = 3 * (t / T) ** 2 - 2 * (t / T) ** 3
    sd = 6 * t / T ** 2 - 6 * t ** 2 / T ** 3
    sdd = 6 / T ** 2 - 12 * t / T ** 3
    return s, sd, sdd


def quintic(T, t):
    x = t / T
    s = 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5
    sd = (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4) / T
    sdd = (60 * x - 180 * x ** 2 + 120 * x ** 3) / T ** 2
    return s, sd, sdd


def trapezoid(T, t, v=1.5, a=4.5):
    # v, a chosen so v^2/a <= 1 and motion reaches s=1 in T=1
    ta = v / a
    s = np.zeros_like(t)
    sd = np.zeros_like(t)
    sdd = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < ta:
            sdd[i] = a
            sd[i] = a * ti
            s[i] = 0.5 * a * ti ** 2
        elif ti < T - ta:
            sdd[i] = 0
            sd[i] = v
            s[i] = v * ti - v ** 2 / (2 * a)
        else:
            sdd[i] = -a
            sd[i] = a * (T - ti)
            s[i] = (2 * a * v * T - 2 * v ** 2 - a ** 2 * (ti - T) ** 2) / (2 * a)
    return s, sd, sdd


def scurve(T, t, J=60.0):
    # integrate a symmetric 7-stage jerk profile numerically, then normalize.
    n = len(t)
    dt = t[1] - t[0]
    # pick stage durations: tj (jerk), taa (const accel), tvv (coast)
    tj = T / 8
    taa = T / 8
    tvv = T - 4 * tj - 2 * taa
    bounds = np.cumsum([tj, taa, tj, tvv, tj, taa, tj])
    jerk = np.zeros(n)
    for i, ti in enumerate(t):
        if ti < bounds[0]:
            jerk[i] = J
        elif ti < bounds[1]:
            jerk[i] = 0
        elif ti < bounds[2]:
            jerk[i] = -J
        elif ti < bounds[3]:
            jerk[i] = 0
        elif ti < bounds[4]:
            jerk[i] = -J
        elif ti < bounds[5]:
            jerk[i] = 0
        else:
            jerk[i] = J
    sdd = np.cumsum(jerk) * dt
    sd = np.cumsum(sdd) * dt
    s = np.cumsum(sd) * dt
    # normalize so s ends at 1
    scale = 1.0 / s[-1]
    return s * scale, sd * scale, sdd * scale


T = 1.0
t = np.linspace(0, T, 600)
profiles = {
    "cubic": (cubic(T, t), "tab:blue"),
    "quintic": (quintic(T, t), "tab:green"),
    "trapezoid": (trapezoid(T, t), "tab:orange"),
    "S-curve": (scurve(T, t), "tab:red"),
}

fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))
titles = [r"position  $s(t)$", r"velocity  $\dot s(t)$", r"acceleration  $\ddot s(t)$"]
for k, ax in enumerate(axes):
    for name, ((s, sd, sdd), c) in profiles.items():
        y = (s, sd, sdd)[k]
        ax.plot(t, y, color=c, label=name, lw=2)
    ax.set_title(titles[k])
    ax.set_xlabel("t / T")
    ax.grid(alpha=0.3)
axes[0].legend(fontsize=8, loc="upper left")
axes[2].axhline(0, color="k", lw=0.6)
fig.suptitle("Time scalings: smoother profile = lower jerk = less vibration "
             "(cost: higher peak velocity)", fontsize=11)
fig.tight_layout()
fig.savefig(OUT / "09_time_scalings.png", dpi=130, bbox_inches="tight")
print("wrote 09_time_scalings.png")


# ---------------------------------------------------------------------------
# Figure 2: the (s, sdot) phase plane — motion cones, velocity-limit curve,
#           bang-bang time-optimal scaling.
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7, 4.6))

s = np.linspace(0, 1, 400)
# a made-up velocity limit curve
vlim = 2.2 + 0.9 * np.sin(2.5 * np.pi * s) * np.exp(-1.5 * s) + 0.6 * s
ax.plot(s, vlim, "k-", lw=2, label="velocity limit curve $\\dot s_{lim}(s)$")
ax.fill_between(s, vlim, vlim.max() + 0.6, color="0.85", zorder=0)
ax.text(0.5, vlim.max() + 0.25, "inadmissible (no torque keeps robot on path)",
        ha="center", fontsize=8.5, color="0.3")

# bang-bang-ish optimal curve: accelerate up then decelerate down
sa = np.linspace(0, 0.62, 200)
up = 3.0 * np.sqrt(sa + 1e-3)
sd2 = np.linspace(0.62, 1, 200)
down = 3.0 * np.sqrt(1 - sd2 + 1e-3)
# clip below vlim
ax.plot(sa, np.minimum(up, np.interp(sa, s, vlim) - 0.05), "tab:red", lw=2.5,
        label="time-optimal scaling")
ax.plot(sd2, np.minimum(down, np.interp(sd2, s, vlim) - 0.05), "tab:red", lw=2.5)
ax.plot(0.62, min(3.0 * np.sqrt(0.62), np.interp(0.62, s, vlim) - 0.05),
        "ko", ms=6)
ax.annotate("switch:\nmax accel → max decel", (0.62, 2.0),
            (0.66, 0.7), fontsize=8.5,
            arrowprops=dict(arrowstyle="->", color="k"))

# draw a few motion cones
for sc, sdc in [(0.2, 0.8), (0.45, 1.5), (0.8, 0.9)]:
    L, U = -3.0, 3.0
    ax.annotate("", (sc + 0.06, sdc + 0.06 * U), (sc, sdc),
                arrowprops=dict(arrowstyle="->", color="tab:blue"))
    ax.annotate("", (sc + 0.06, sdc + 0.06 * L), (sc, sdc),
                arrowprops=dict(arrowstyle="->", color="tab:blue"))

ax.plot(0, 0, "go", ms=7)
ax.plot(1, 0, "gs", ms=7)
ax.text(0.01, 0.12, "start (0,0)", fontsize=8.5)
ax.text(0.86, 0.12, "end (1,0)", fontsize=8.5)
ax.set_xlabel("path position  $s$")
ax.set_ylabel("path speed  $\\dot s$")
ax.set_title("Time-optimal time scaling lives in the $(s,\\dot s)$ phase plane")
ax.set_xlim(-0.02, 1.02)
ax.set_ylim(0, vlim.max() + 0.6)
ax.legend(fontsize=8.5, loc="lower center")
fig.tight_layout()
fig.savefig(OUT / "09_phase_plane.png", dpi=130, bbox_inches="tight")
print("wrote 09_phase_plane.png")


# ---------------------------------------------------------------------------
# Figure 3: classical vs learned trajectory generation (conceptual block diagram)
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(11, 3.4))
ax.axis("off")


def box(x, y, w, h, text, fc):
    ax.add_patch(plt.Rectangle((x, y), w, h, fc=fc, ec="k", lw=1.3))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=8.5)


def arrow(x1, y1, x2, y2):
    ax.annotate("", (x2, y2), (x1, y1),
                arrowprops=dict(arrowstyle="->", lw=1.4))


# classical row
ax.text(0.0, 2.55, "CLASSICAL (Ch 9)", fontsize=10, weight="bold")
box(0.0, 1.7, 1.6, 0.6, "task gives\nstart/end (+vias)", "#dbe9ff")
box(2.0, 1.7, 1.6, 0.6, "pick a PATH\n$\\theta(s)$", "#dbe9ff")
box(4.0, 1.7, 1.7, 0.6, "pick a TIME\nSCALING $s(t)$", "#dbe9ff")
box(6.1, 1.7, 1.7, 0.6, "$\\theta(t)$ ref\n→ controller", "#dbe9ff")
arrow(1.6, 2.0, 2.0, 2.0)
arrow(3.6, 2.0, 4.0, 2.0)
arrow(5.7, 2.0, 6.1, 2.0)
ax.text(8.1, 2.0, "hand-designed,\ngeometry-only,\nopen-loop ref",
        fontsize=8, va="center", color="0.3")

# learned row
ax.text(0.0, 1.15, "LEARNED (SOTA)", fontsize=10, weight="bold")
box(0.0, 0.3, 1.6, 0.6, "observation\nimage(s)+state", "#ffe2c4")
box(2.0, 0.3, 2.2, 0.6, "policy net\n(Diffusion / ACT)", "#ffe2c4")
box(4.6, 0.3, 3.0, 0.6, "predicts a CHUNK of\nfuture actions $a_{t:t+H}$", "#ffe2c4")
box(8.0, 0.3, 1.7, 0.6, "execute, then\nre-plan", "#ffe2c4")
arrow(1.6, 0.6, 2.0, 0.6)
arrow(4.2, 0.6, 4.6, 0.6)
arrow(7.6, 0.6, 8.0, 0.6)
ax.set_xlim(-0.1, 11)
ax.set_ylim(0, 2.9)
fig.tight_layout()
fig.savefig(OUT / "09_classical_vs_learned.png", dpi=130, bbox_inches="tight")
print("wrote 09_classical_vs_learned.png")
