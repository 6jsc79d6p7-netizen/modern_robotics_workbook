"""Scripted privileged-state expert (Step 4).

Reads ground-truth object/bin poses and plans a pick→place as a list of
waypoint segments `(pos, R, gripper, min_ticks, max_ticks)`. The runner drives
the *same* EEController through them (its rate limiter smooths waypoint→waypoint)
and records via the *same* Recorder — the expert is just another target source.

Key bit: the gripper is yawed so the fingers close **across** an elongated
object's axis (the capsule can only be grasped perpendicular to its long axis;
box/cylinder are yaw-forgiving). Modest per-episode randomization injects
multimodality so the demos aren't a single canonical trajectory.

The path lifts high before translating over the bin, so it naturally **arcs over
the bin wall** (the planning challenge) instead of clipping it.
"""
import numpy as np
import mujoco

from .scene import tcp_pose, TABLE_TOP


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _quat_wxyz_to_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


class ScriptedExpert:
    def __init__(self, env):
        self.env = env
        d = mujoco.MjData(env.model)
        d.qpos[:] = env.info.home_qpos
        mujoco.mj_forward(env.model, d)
        self.home_p, self.R0 = tcp_pose(env.model, d, env.info)   # home pose

    def _grasp_R(self, i, rng):
        """Gripper orientation: down, yawed so fingers close across the object."""
        env, d = self.env, self.env.data
        shape = env.env.obj_desc[i][1]
        yaw = 0.0
        if shape == "capsule":
            qadr = env.env.obj_qadr[i]
            axis = _quat_wxyz_to_mat(d.qpos[qadr + 3:qadr + 7]) @ np.array([0, 0, 1.0])
            axis[2] = 0.0
            n = np.linalg.norm(axis)
            if n > 1e-6:
                axis /= n
                yaw = np.arctan2(axis[1], axis[0]) - np.pi / 2   # close ⊥ to axis
        yaw += rng.normal(0, 0.04)                                # multimodality
        return _rz(yaw) @ self.R0

    def plan(self, rng):
        env = self.env
        i, b = env.target_obj, env.target_bin
        op = env.obj_pos(i).copy()
        bx, by = env.bin_xy[b]
        Rg = self._grasp_R(i, rng)

        gz = op[2]                                    # grasp: TCP at object center
        h_app = op[2] + 0.12 + rng.uniform(-0.01, 0.02)
        h_lift = TABLE_TOP + 0.25 + rng.uniform(-0.02, 0.03)
        drop = [bx + rng.uniform(-0.02, 0.02), by + rng.uniform(-0.02, 0.02),
                TABLE_TOP + 0.08]                          # low release → less roll-out
        OPEN, CLOSE = 255, 0
        hp = self.home_p
        #      pos,                     R,   grip,  min, max
        return [
            ([hp[0], hp[1], h_lift],    self.R0, OPEN, 30, 500),  # rise straight up
            ([op[0], op[1], h_lift],    Rg, OPEN,   30, 800),   # translate over object (high)
            ([op[0], op[1], h_app],     Rg, OPEN,   30, 600),   # descend to pre-grasp
            ([op[0], op[1], gz],        Rg, OPEN,   40, 600),   # descend to object (vertical)
            ([op[0], op[1], gz],        Rg, CLOSE, 220, 340),   # close (dwell)
            ([op[0], op[1], h_lift],    Rg, CLOSE,  40, 800),   # lift
            ([bx, by, h_lift],          Rg, CLOSE,  40, 900),   # over the bin (arc over wall)
            (list(drop),                Rg, CLOSE,  40, 800),   # descend into bin
            (list(drop),                Rg, OPEN,  200, 340),   # release (dwell)
            ([bx, by, h_lift],          Rg, OPEN,   30, 600),   # retreat
        ]
