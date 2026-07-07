"""TeleopSource — phone WebXR pose → EE target, the clutched relative mapping.

This is the Ch-3 payoff (proj_phone_teleop.md §4). The phone streams its pose in
a fixed, gravity-aligned AR world frame {ar} (WebXR `local`, y-up). We map motion
*since the clutch was engaged* (so you can ratchet, and VIO drift resets on every
re-engage):

  on engage (t0):  p0,R0 = phone pose in {ar};  E0 = current held EE target
  while engaged:   Δp = p(t) - p0            (translation in {ar})
                   ΔR = R(t) · R0ᵀ           (rotation in {ar})
                   p_tgt = p(E0) + s · R_align · Δp
                   R_tgt = R_align · ΔR · R_alignᵀ · R(E0)

Working in {ar} (world) rather than the phone's device frame means tilting/holding
the phone differently during a motion doesn't corrupt the axes — only gravity and
the calibrated heading matter.

R_align maps {ar} axes (y-up) → robot base axes (z-up), built so that:
  - your calibrated **forward** (`_f0`, a horizontal dir in {ar}) → into the screen
  - {ar} up (+y)                                                  → robot up (+z)
  - the remaining horizontal axis                                → screen-right
"Into the screen" follows the live sim-view azimuth. Tap "Set Fwd" on the phone
(pointing it the way you want forward) to set `_f0` empirically — no assumptions
about the WebXR device-axis or MuJoCo azimuth conventions.
"""
import numpy as np
import mujoco

from mr.so3 import matrix_log3
from .target_source import TargetSource, TargetCommand
from .scene import tcp_pose

AR_UP = np.array([0.0, 1.0, 0.0])          # WebXR {ar} is y-up
F0_DEFAULT = np.array([0.0, 0.0, -1.0])    # session-start forward ({ar} -z)


def quat_xyzw_to_mat(q):
    """WebXR quaternion (x, y, z, w) → 3x3 rotation matrix."""
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def build_R_align(f0, azimuth_deg):
    """Rotation mapping {ar}(y-up) → robot base(z-up) for the given forward `f0`
    (horizontal, in {ar}) and sim-view azimuth.

        f0 (ar forward)      → fh (into the screen)
        {ar} up  [0,1,0]     → robot up [0,0,1]
        ar-right = f0 × up   → screen-right = fh × up_robot
    R_align = M_robot · M_arᵀ  (both orthonormal frames).
    """
    a = np.radians(azimuth_deg)
    fh = np.array([np.cos(a), np.sin(a), 0.0])          # camera forward (into screen)
    up_r = np.array([0.0, 0.0, 1.0])
    r = np.cross(fh, up_r); r /= np.linalg.norm(r)      # screen-right
    M_robot = np.column_stack([r, up_r, fh])

    f0 = np.asarray(f0, float)
    f0 = f0 / np.linalg.norm(f0)                         # ensure unit horizontal fwd
    right_ar = np.cross(f0, AR_UP); right_ar /= np.linalg.norm(right_ar)
    M_ar = np.column_stack([right_ar, AR_UP, f0])
    return M_robot @ M_ar.T


class TeleopSource(TargetSource):
    def __init__(self, server, scale=1.5, view_azimuth=90.0,
                 pos_jump=0.06, ang_jump=0.25):
        self.server = server
        self.scale = scale
        self.view_azimuth = view_azimuth        # set each tick by run_teleop
        self._f0 = F0_DEFAULT.copy()            # calibrated forward in {ar}
        self._Ralign = build_R_align(self._f0, view_azimuth)
        # per-frame VIO discontinuity thresholds (loop-closure / relocalization)
        self.pos_jump = pos_jump
        self.ang_jump = ang_jump
        # held EE target (persists across clutch releases → the ratchet)
        self.tp = None
        self.tR = None
        self.gripper = 255.0
        self._prev_clutch = False
        self._p0 = None        # phone pos/rot in {ar} at engage
        self._R0 = None
        self._E0p = None       # EE pose at engage
        self._E0R = None
        self._p_prev = None    # phone pose last engaged frame (jump detection)
        self._R_prev = None
        self._last_R = None    # most recent phone orientation (for Set Fwd)
        self.home_p = None
        self.home_R = None

    def reset(self, model, data, info):
        self.tp, self.tR = tcp_pose(model, data, info)
        self._prev_clutch = False
        self._p0 = self._R0 = self._p_prev = self._R_prev = None
        d = mujoco.MjData(model)
        d.qpos[:] = info.home_qpos
        mujoco.mj_forward(model, d)
        self.home_p, self.home_R = tcp_pose(model, d, info)

    def _go_home(self):
        """Snap the held target to home and drop the engage latch (recover)."""
        self.tp, self.tR = self.home_p.copy(), self.home_R.copy()
        self._p0 = self._R0 = self._p_prev = self._R_prev = None
        self._prev_clutch = False

    def _set_forward(self):
        """Calibrate: current phone forward (device -z, horizontal) = into-screen."""
        if self._last_R is None:
            return
        fwd = self._last_R @ np.array([0.0, 0.0, -1.0])   # device forward in {ar}
        fwd[1] = 0.0                                        # project to horizontal
        n = np.linalg.norm(fwd)
        if n > 1e-6:
            self._f0 = fwd / n
            self._Ralign = build_R_align(self._f0, self.view_azimuth)

    def __call__(self, t, model, data, info) -> TargetCommand:
        if self.server.state.take_home_request():
            self._go_home()

        msg = self.server.latest()
        if msg is None:
            return TargetCommand(pos=self.tp, R=self.tR, gripper=self.gripper)

        self.gripper = float(msg.get("gripper", self.gripper))
        clutch = bool(msg.get("clutch", False))
        tracked = bool(msg.get("tracked", True))
        p = np.asarray(msg["p"], float)
        R = quat_xyzw_to_mat(msg["q"])
        self._last_R = R
        if self.server.state.take_setfwd_request():        # uses current orientation
            self._set_forward()

        if clutch and not self._prev_clutch:
            # rising edge: latch engage reference (phone + EE + view mapping)
            self._p0, self._R0 = p.copy(), R.copy()
            self._E0p, self._E0R = self.tp.copy(), self.tR.copy()
            self._p_prev, self._R_prev = p.copy(), R.copy()
            self._Ralign = build_R_align(self._f0, self.view_azimuth)
        self._prev_clutch = clutch

        if clutch and self._p0 is not None:
            # Reject VIO discontinuities: teleport since last frame (or tracking
            # lost) → re-latch without moving the robot (jump invisible to arm).
            jump = not tracked
            if self._p_prev is not None and not jump:
                _, ang = matrix_log3(R @ self._R_prev.T)
                jump = (np.linalg.norm(p - self._p_prev) > self.pos_jump
                        or ang > self.ang_jump)
            if jump:
                self._p0, self._R0 = p.copy(), R.copy()
                self._E0p, self._E0R = self.tp.copy(), self.tR.copy()
            else:
                dp = p - self._p0                 # translation since engage, {ar}
                dR = R @ self._R0.T               # rotation since engage, {ar}
                self.tp = self._E0p + self.scale * (self._Ralign @ dp)
                self.tR = self._Ralign @ dR @ self._Ralign.T @ self._E0R
            self._p_prev, self._R_prev = p.copy(), R.copy()
        # else: disengaged → freeze target (ratchet / reposition)

        return TargetCommand(pos=self.tp, R=self.tR, gripper=self.gripper)
