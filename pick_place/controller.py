"""EEController — the pure EE-pose target-follower.

Its job never changes: *given a target TCP pose this tick → IK → position
actuators*. It does not know or care whether the target came from a dummy path,
the phone, or a scripted expert.

The one bit of built-in intelligence is a **Cartesian rate limiter**: the
commanded target is moved toward the requested target by at most `max_lin/ang
speed · dt` per tick. That is a safety net (a dropped teleop packet or a clutch
re-engage would otherwise ask for a huge instantaneous jump) — it is *not*
trajectory generation. Multi-waypoint trajectory generation lives in the
scripted-expert TargetSource, not here.
"""
import numpy as np
import mujoco

from mr.so3 import matrix_log3, matrix_exp3
from .ik import DLSIK
from .scene import tcp_pose


class EEController:
    def __init__(self, model, info, max_lin_speed=1.5, max_ang_speed=3.0, **ik_kw):
        self.model = model
        self.info = info
        self.dt = model.opt.timestep
        self.max_lin_step = max_lin_speed * self.dt
        self.max_ang_step = max_ang_speed * self.dt
        self.ik = DLSIK(model, info, **ik_kw)
        self.cmd_pos = None      # rate-limited commanded target pose
        self.cmd_R = None
        self.q_ref = None        # internal IK reference config (decoupled from lag)

    def reset(self, data):
        """Seed the commanded target + IK reference at the current state."""
        self.cmd_pos, self.cmd_R = tcp_pose(self.model, data, self.info)
        self.q_ref = data.qpos.copy()

    def _rate_limit(self, target_pos, target_R):
        # translation: clamp the step vector's length
        dp = target_pos - self.cmd_pos
        n = np.linalg.norm(dp)
        if n > self.max_lin_step:
            dp *= self.max_lin_step / n
        self.cmd_pos = self.cmd_pos + dp

        # rotation: clamp the angle of the relative rotation R_rel = R* cmdᵀ
        omega_hat, theta = matrix_log3(target_R @ self.cmd_R.T)
        if theta > self.max_ang_step:
            theta = self.max_ang_step
        self.cmd_R = matrix_exp3(omega_hat, theta) @ self.cmd_R if theta > 0 else self.cmd_R

    def step(self, data, cmd):
        """Apply one control tick from a TargetCommand (sets data.ctrl)."""
        if self.cmd_pos is None:
            self.reset(data)
        self._rate_limit(cmd.pos, cmd.R)
        # Seed IK from our own integrated reference, NOT the lagging physical
        # qpos: keeps the redundant solution consistent so joint targets are
        # smooth and the position servo actually catches up.
        q_arm = self.ik.solve(self.q_ref, self.cmd_pos, self.cmd_R)
        self.q_ref[self.info.arm_qpos_ids] = q_arm
        data.ctrl[self.info.arm_act_ids] = q_arm
        data.ctrl[self.info.gripper_act_id] = np.clip(cmd.gripper, 0, 255)
