"""Numerical inverse kinematics for the Franka arm — the Ch 5 + Ch 6 payoff.

This is *differential* IK by damped least squares (DLS / Levenberg-Marquardt):
given a target TCP pose, repeatedly take the 6-D pose error (a twist), map it to
a joint-space step through the manipulator Jacobian, and update the joints.

    err (6)  =  [ rotation error ; position error ]      (world frame)
    J   (6x7) =  [ jacr ; jacp ]  at the TCP site         (mj_jacSite)
    dq  (7)  =  Jᵀ (J Jᵀ + λ²I)⁻¹ err                     (DLS: λ damps singularities)

Why DLS and not the plain pseudo-inverse J⁺: near a singularity J⁺ blows dq up
and the arm snaps. The +λ²I keeps the 6x6 solve well-conditioned — you trade a
little tracking accuracy for stability, which is exactly right for a follower.

Called every control tick warm-started from the current qpos, it converges in
1–3 iterations, so it doubles as a smooth resolved-rate controller.
"""
import numpy as np
import mujoco

from mr.so3 import matrix_log3


class DLSIK:
    def __init__(self, model, info, damping=0.05, max_iters=20,
                 pos_tol=1e-3, rot_tol=1e-2, max_dq=0.25):
        self.model = model
        self.info = info
        self.damping = damping
        self.max_iters = max_iters
        self.pos_tol = pos_tol
        self.rot_tol = rot_tol
        self.max_dq = max_dq          # per-iter joint step cap (rad)
        self._scratch = mujoco.MjData(model)
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))

    def solve(self, seed_qpos, target_p, target_R):
        """Return the 7 arm joint angles (joint1..7 order) reaching the target.

        seed_qpos: full (nq,) configuration to warm-start from (usually data.qpos).
        target_p, target_R: desired TCP position (3,) and orientation (3x3).
        """
        m, info, d = self.model, self.info, self._scratch
        d.qpos[:] = seed_qpos
        qadr, dadr, sid = info.arm_qpos_ids, info.arm_dof_ids, info.tcp_site_id

        for _ in range(self.max_iters):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)                       # needed for site Jacobian

            p = d.site_xpos[sid]
            R = d.site_xmat[sid].reshape(3, 3)
            pos_err = target_p - p
            # orientation error as a world-frame rotation vector: R_err = R* Rᵀ
            omega_hat, theta = matrix_log3(target_R @ R.T)
            rot_err = omega_hat * theta

            if np.linalg.norm(pos_err) < self.pos_tol and \
               np.linalg.norm(rot_err) < self.rot_tol:
                break

            err = np.concatenate([rot_err, pos_err])     # twist order (omega, v)
            mujoco.mj_jacSite(m, d, self._jacp, self._jacr, sid)
            J = np.vstack([self._jacr[:, dadr], self._jacp[:, dadr]])  # (6,7)

            # DLS step: dq = Jᵀ (J Jᵀ + λ²I)⁻¹ err
            JJt = J @ J.T + (self.damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)

            n = np.linalg.norm(dq)
            if n > self.max_dq:
                dq *= self.max_dq / n

            q = d.qpos[qadr] + dq
            q = np.clip(q, info.arm_jnt_range[:, 0], info.arm_jnt_range[:, 1])
            d.qpos[qadr] = q

        return d.qpos[qadr].copy()
