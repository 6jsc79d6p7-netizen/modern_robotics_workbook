"""Generate 11a_pid_step_response.png — P vs PD vs PID step response on a 1-DOF
inertia with a constant gravity-like disturbance. Shows: P rings (underdamped),
PD settles cleanly (critical), PID also erases the steady-state offset."""
import numpy as np
import matplotlib.pyplot as plt

HERE = __file__.rsplit("/", 1)[0]

# 1-DOF plant with a little natural damping:
#   M x'' + b x' = tau - d     (unit inertia, constant disturbance d = "gravity")
# The small b lets P-only *ring and decay* (visible underdamping) instead of
# oscillating forever off-chart.
M, b, d = 1.0, 2.5, 5.0
kp, kd = 36.0, 10.0           # kd picked so plant+D damping ~ critical
ki = 90.0
dt, T = 0.001, 1.6
t = np.arange(0, T, dt)
target = 1.0


def sim(use_d, use_i):
    x = v = ei = 0.0
    xs = []
    for _ in t:
        e = target - x
        ei += e * dt
        tau = kp * e + (kd * (-v) if use_d else 0.0) + (ki * ei if use_i else 0.0)
        a = (tau - b * v - d) / M
        v += a * dt
        x += v * dt
        xs.append(x)
    return np.array(xs)


p = sim(False, False)          # P only  -> rings + offset
pd = sim(True, False)          # PD      -> clean settle, small offset from d
pid = sim(True, True)          # PID     -> clean settle, no offset

fig, ax = plt.subplots(figsize=(8, 4.5))
ax.axhline(target, color="0.6", ls="--", lw=1, label="target")
ax.plot(t, p, color="#d1495b", lw=2, label="P only  (underdamped: overshoot + ring)")
ax.plot(t, pd, color="#2e86ab", lw=2, label="PD  (near-critical: clean settle)")
ax.plot(t, pid, color="#3a9d23", lw=2, label="PID  (PD + integral kills the offset)")
# annotate the leftover steady-state error of PD (from the constant disturbance)
ax.annotate("steady-state\nerror (P/PD)", xy=(1.5, pd[-1]), xytext=(1.05, 0.55),
            fontsize=9, color="#2e86ab",
            arrowprops=dict(arrowstyle="->", color="#2e86ab", lw=1.2))
ax.set_xlabel("time (s)")
ax.set_ylabel("position  (target = 1)")
ax.set_title("P vs PD vs PID: the controller renders a spring-damper")
ax.legend(loc="lower right", fontsize=9, framealpha=0.95)
ax.set_ylim(-0.05, 1.6)
print("peak P=%.2f  final PD=%.3f  final PID=%.3f" % (p.max(), pd[-1], pid[-1]))
ax.grid(alpha=0.25)
fig.tight_layout()
out = f"{HERE}/11a_pid_step_response.png"
fig.savefig(out, dpi=130)
print("wrote", out)
