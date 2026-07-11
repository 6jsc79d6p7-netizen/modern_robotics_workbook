"""CartesianImpedanceController — the compliant alternative to EEController.

Same job, same interface (`reset` / `step(data, cmd)`), *opposite* control mode.

  EEController (controller.py):   target pose → IK → **position servos**  (STIFF)
  This one:                       target pose → **Cartesian impedance torque** (SOFT)

Instead of solving IK to a joint target and letting a stiff position servo slam
the arm there, we render a *virtual spring-damper* between the TCP and the target
pose and push the arm with the corresponding **joint torques**:

    F  (wrench) = K (x_d − x)  −  B ẋ          ← spring pulls to target, damper bleeds speed
    τ  (torque) = Jᵀ F                          ← the velocity/force duality (Ch 11b)

Why this is the contact-rich upgrade (11b): a stiff position controller told to
go *through* an object pushes with enormous force (huge error → huge servo
torque). This controller only ever pulls with `K · error`, so choosing a low `K`
makes the arm **give** on contact instead of crushing — and forgives the few-mm
pose errors an imperfect policy / camera calibration will produce. It also skips
the IK solve entirely: it needs forward kinematics + the Jacobian, never the
inverse-position solve.

MuJoCo specifics:
  * The Franka arm ships as position servos (see scene.py). To command torque we
    **neutralize those servos** at init (zero their gains) and inject joint
    torque through `data.qfrc_applied[arm_dof_ids]` every tick.
  * **Gravity + Coriolis feedforward**: we add `data.qfrc_bias` (= C(q,q̇)q̇ + g(q))
    so the arm floats and the spring only fights the *residual* — the note's
    "gravity feedforward + PD". NB: the scene sets `body_gravcomp=1`, but in this
    MuJoCo build that flag is a **no-op** (`qfrc_gravcomp` stays 0; the arm falls
    under zero torque). The stiff EEController masked it by brute force; a soft
    impedance controller must compensate explicitly, which is what we do here.
  * The gripper (actuator8) stays a position servo, unchanged.

Redundancy: the 7-DOF arm has a null space the 6-D task leaves free, so we add a
soft posture spring toward the home configuration to stop the elbow drifting.
"""
import numpy as np
import mujoco

from mr.so3 import matrix_log3, matrix_exp3
from .scene import tcp_pose


def _diag6(rot, pos):
    """Build a 6-vector of gains in [ω(3); v(3)] order from rotational and
    translational parts (each a scalar or a 3-vector). Diagonal K/B, so a
    per-axis value like pos=[600, 600, 150] means 'stiff in x,y, soft in z'."""
    rot = np.broadcast_to(rot, 3)
    pos = np.broadcast_to(pos, 3)
    return np.concatenate([rot, pos]).astype(float)


class CartesianImpedanceController:
    def __init__(self, model, info,
                 kp_pos=600.0, kp_rot=30.0,        # stiffness (N/m, N·m/rad)
                 damping_ratio=1.0,                # ζ: 1 = critical (no overshoot)
                 kp_null=5.0, kd_null=2.0,         # null-space posture spring
                 max_lin_speed=1.5, max_ang_speed=3.0):
        self.model = model
        self.info = info
        self.dt = model.opt.timestep
        self.max_lin_step = max_lin_speed * self.dt
        self.max_ang_step = max_ang_speed * self.dt

        # Stiffness K and damping B (diagonal, [ω(3); v(3)] to match the Jacobian).
        self.K = _diag6(kp_rot, kp_pos)
        # Critical-ish damping per axis: B = 2ζ√K  (unit-inertia rule from 11a;
        # the true reflected inertia isn't 1, so treat ζ as a tuning knob).
        self.B = 2.0 * damping_ratio * np.sqrt(self.K)

        self.kp_null = kp_null
        self.kd_null = kd_null

        # Jacobian scratch (MuJoCo fills full-nv columns; we slice the arm dofs).
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))

        self.cmd_pos = None      # rate-limited equilibrium pose (the spring anchor)
        self.cmd_R = None
        self.q_home = info.home_qpos[info.arm_qpos_ids].copy()
        self._torque_mode = False

    # ------------------------------------------------------------------ setup
    def _enable_torque_mode(self):
        """Neutralize the arm position servos so `data.ctrl[arm]` exerts no force;
        we drive the arm purely through `data.qfrc_applied` afterwards.

        A position servo's force is  kp·(ctrl − q) − kd·q̇  via
        gainprm[0]=kp, biasprm[1]=−kp, biasprm[2]=−kd. Zero all three → 0 force,
        whatever ctrl/qpos are. Reversible (we don't touch the XML)."""
        for a in self.info.arm_act_ids:
            self.model.actuator_gainprm[a, 0] = 0.0
            self.model.actuator_biasprm[a, 1] = 0.0
            self.model.actuator_biasprm[a, 2] = 0.0
        self._torque_mode = True

    def reset(self, data):
        """Seed the spring anchor at the current TCP pose and enter torque mode."""
        if not self._torque_mode:
            self._enable_torque_mode()
        self.cmd_pos, self.cmd_R = tcp_pose(self.model, data, self.info)

    # ------------------------------------------------------------ setpoint move
    def _rate_limit(self, target_pos, target_R):
        """Move the spring anchor toward the requested target at a bounded speed,
        so a teleop jump / policy step doesn't yank the spring (a big Δx = a big
        force spike). Identical logic to EEController — smooths the *equilibrium*,
        not the arm."""
        dp = target_pos - self.cmd_pos
        n = np.linalg.norm(dp)
        if n > self.max_lin_step:
            dp *= self.max_lin_step / n
        self.cmd_pos = self.cmd_pos + dp

        omega_hat, theta = matrix_log3(target_R @ self.cmd_R.T)
        if theta > self.max_ang_step:
            theta = self.max_ang_step
        self.cmd_R = matrix_exp3(omega_hat, theta) @ self.cmd_R if theta > 0 else self.cmd_R

    # ------------------------------------------------------------------- tick
    def step(self, data, cmd):
        """One control tick: render the Cartesian spring-damper as joint torque."""
        if self.cmd_pos is None:
            self.reset(data)
        self._rate_limit(cmd.pos, cmd.R)

        info = self.info
        dadr = info.arm_dof_ids

        # --- current TCP pose + 6-D pose error (in [ω; v] order) ---
        p, R = tcp_pose(self.model, data, info)
        pos_err = self.cmd_pos - p                                  # (3,)
        omega_hat, theta = matrix_log3(self.cmd_R @ R.T)            # world-frame
        rot_err = omega_hat * theta                                # (3,)
        err6 = np.concatenate([rot_err, pos_err])                  # [ω; v] (6,)

        # --- Jacobian at the TCP site, arm columns, [jacr; jacp] to match err ---
        mujoco.mj_jacSite(self.model, data, self._jacp, self._jacr, info.tcp_site_id)
        J = np.vstack([self._jacr[:, dadr], self._jacp[:, dadr]])  # (6,7)

        # --- current EE spatial velocity  ẋ = J q̇  ---
        qd = data.qvel[dadr]                                       # (7,)
        xdot = J @ qd                                              # [ω; v] (6,)

        # --- desired EE wrench: spring pulls to target, damper opposes velocity ---
        wrench = self.K * err6 - self.B * xdot                     # [M; F] (6,)

        # --- map wrench → joint torque (the transpose is the whole point) ---
        tau = J.T @ wrench                                         # (7,)

        # --- null-space posture: pull the redundant arm toward home without
        #     disturbing the TCP.  N = I − Jᵀ (Jᵀ)⁺  projects into no-EE-wrench torques.
        q = data.qpos[info.arm_qpos_ids]
        tau_null = self.kp_null * (self.q_home - q) - self.kd_null * qd
        N = np.eye(7) - J.T @ np.linalg.pinv(J.T)
        tau = tau + N @ tau_null

        # --- gravity + Coriolis feedforward (the note's g(θ) term).  qfrc_bias =
        #     C(q,q̇)q̇ + g(q); cancelling it leaves a floating arm the spring then
        #     controls.  (We compute it ourselves: MuJoCo's body_gravcomp is a
        #     no-op in this build — see notes; the stiff EEController masked that.)
        tau = tau + data.qfrc_bias[dadr]

        # --- apply as generalized force ---
        data.qfrc_applied[dadr] = tau

        # --- gripper unchanged: still its own position servo ---
        data.ctrl[info.gripper_act_id] = np.clip(cmd.gripper, 0, 255)
