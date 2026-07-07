"""PickPlaceEnv — the Step 3 task environment.

Builds on the Franka scene (scene.make_spec) and adds:
  - graspable free-body objects (shape x color),
  - walled open-top bins as mocap bodies (immovable, still collidable),
  - a scene camera + a wrist camera (Step 4 observations),
and provides reset() with randomized, non-overlapping, reachable placement, a
generated language instruction ("put the red box into the purple bin"), and
success detection (target object settled inside the target bin) for auto-reset.

Object *identity* (which 3 shapes/colors) is baked into the model; each reset
randomizes their *positions* and the target object/bin. (Identity diversity via
model rebuild can come later.)
"""
from dataclasses import dataclass

import numpy as np
import mujoco

from . import scene
from .scene import make_spec, apply_gravcomp, scene_info, TABLE_TOP

# ---- palette -------------------------------------------------------------
COLORS = {
    "red": (0.85, 0.15, 0.15), "green": (0.15, 0.7, 0.2),
    "blue": (0.15, 0.35, 0.85), "yellow": (0.9, 0.8, 0.1),
}
BIN_COLORS = {"orange": (0.95, 0.5, 0.1), "purple": (0.6, 0.2, 0.8)}

# fixed object set for this model instance (diverse shapes + colors).
# capsule replaces the sphere: round-looking but graspable (a ball can't be held
# by parallel jaws — low grip slips, high grip ejects it).
OBJECTS = [("red", "box"), ("green", "capsule"), ("blue", "cylinder")]
OBJ_HALF = 0.025                       # box half-extent / cyl radius
CYL_HALFH = 0.03
CAP_R = 0.02                           # capsule radius
CAP_HALFL = 0.03                       # capsule cylinder-part half-length
# friction = [slide, torsion, roll]; condim=6 enables torsional + ROLLING
# friction (default condim=3 has neither, so a sphere just rolls out of a pinch).
OBJ_FRICTION = [3.0, 0.1, 0.05]
OBJ_CONDIM = 6
FINGER_FRICTION = [3.0, 0.1, 0.05]
FINGER_BODIES = ("left_finger", "right_finger")
# The Franka gripper is a soft position servo (~2 N grip by default), too weak to
# hold a sphere under lateral motion. Stiffen it so it squeezes harder; scaling
# gain + bias together preserves the ctrl→width mapping but multiplies force.
GRIP_STIFFEN = 4.0

# bin geometry (open-top box)
BIN_INNER = 0.075                      # inner half-width
BIN_WALL_T = 0.01
BIN_WALL_H = 0.06                      # tall enough that a straight-line place clips it
BINS = list(BIN_COLORS.keys())

# reachable sampling region on the table top (x forward, y left)
REGION_X = (0.40, 0.64)
REGION_Y = (-0.28, 0.28)
BIN_REGION_X = (0.45, 0.64)            # keep bins a bit further out
# min center separations (m)
SEP_BIN_BIN, SEP_BIN_OBJ, SEP_OBJ_OBJ = 0.24, 0.14, 0.10


def _rest_z(shape):
    if shape == "cylinder":
        return TABLE_TOP + CYL_HALFH
    if shape == "capsule":
        return TABLE_TOP + CAP_R              # lies on its side
    return TABLE_TOP + OBJ_HALF


def _yaw_quat(yaw):
    return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])   # (w,x,y,z) about z


_LAY_Y = np.array([0.70710678, 0, 0.70710678, 0])              # rot 90° about y


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _rest_quat(shape, yaw):
    """Capsules lie on their side (a vertical capsule topples); others upright."""
    if shape == "capsule":
        return _quat_mul(_yaw_quat(yaw), _LAY_Y)
    return _yaw_quat(yaw)


@dataclass
class EnvInfo:
    obj_bodies: list          # body ids of the 3 objects
    obj_qadr: list            # free-joint qpos address per object
    obj_geoms: list           # geom id per object (for grasp haptics)
    obj_desc: list            # (color, shape) per object
    bin_mocap: list           # mocap id per bin
    bin_desc: list            # color name per bin


class PickPlaceEnv:
    def __init__(self, seed=None):
        self.rng = np.random.default_rng(seed)
        self.model, self.info, self.env = self._build()
        self.data = mujoco.MjData(self.model)
        self.target_obj = 0
        self.target_bin = 0
        self.instruction = ""
        self.bin_xy = np.zeros((len(BINS), 2))

    # ---- model construction ----
    def _build(self):
        spec = make_spec()
        wb = spec.worldbody

        for i, (color, shape) in enumerate(OBJECTS):
            b = wb.add_body(name=f"obj{i}", pos=[0.5, 0.0, _rest_z(shape) + 0.2 * i])
            b.add_freejoint()
            rgba = list(COLORS[color]) + [1.0]
            kw = dict(name=f"obj{i}", rgba=rgba, friction=OBJ_FRICTION,
                      condim=OBJ_CONDIM)
            if shape == "box":
                b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[OBJ_HALF] * 3, **kw)
            elif shape == "capsule":
                b.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                           size=[CAP_R, CAP_HALFL, 0], **kw)
            else:  # cylinder
                b.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                           size=[OBJ_HALF - 0.003, CYL_HALFH, 0], **kw)

        for color in BINS:
            self._add_bin(wb, f"bin_{color}", BIN_COLORS[color])

        # cameras (Step 4 observations)
        C, T = np.array([0.5, -0.75, 1.0]), np.array([0.5, 0.0, TABLE_TOP])
        wb.add_camera(name="scene", pos=C, xyaxes=_look_at(C, T), fovy=50)
        hand = spec.body("hand")
        hand.add_camera(name="wrist", pos=[0, 0, 0.03],
                        xyaxes=[1, 0, 0, 0, -1, 0], fovy=70)

        model = spec.compile()
        apply_gravcomp(model)
        # grippier fingertips so the grasp holds (contact friction combines the
        # two geoms, but bumping both sides is the robust belt-and-suspenders)
        for gid in range(model.ngeom):
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                      model.geom_bodyid[gid])
            if bname in FINGER_BODIES:
                model.geom_friction[gid] = FINGER_FRICTION
                model.geom_condim[gid] = OBJ_CONDIM
        base = scene_info(model)
        # stiffen the gripper so it grips hard enough to hold under motion
        g = base.gripper_act_id
        model.actuator_gainprm[g, 0] *= GRIP_STIFFEN
        model.actuator_biasprm[g, 1] *= GRIP_STIFFEN
        model.actuator_biasprm[g, 2] *= GRIP_STIFFEN

        def bid(n): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        def gid(n): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
        obj_bodies = [bid(f"obj{i}") for i in range(len(OBJECTS))]
        env = EnvInfo(
            obj_bodies=obj_bodies,
            obj_qadr=[model.jnt_qposadr[model.body_jntadr[b]] for b in obj_bodies],
            obj_geoms=[gid(f"obj{i}") for i in range(len(OBJECTS))],
            obj_desc=list(OBJECTS),
            bin_mocap=[model.body_mocapid[bid(f"bin_{c}")] for c in BINS],
            bin_desc=list(BINS),
        )
        return model, base, env

    def _add_bin(self, wb, name, rgb):
        body = wb.add_body(name=name, mocap=True, pos=[0.5, 0.0, TABLE_TOP])
        rgba = list(rgb) + [1.0]
        e, t, h = BIN_INNER, BIN_WALL_T, BIN_WALL_H
        # floor
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[e + t, e + t, 0.005],
                      pos=[0, 0, 0.005], rgba=rgba)
        # 4 walls
        for sx, sy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            if sx:
                size, pos = [t / 2, e + t, h / 2], [sx * (e + t / 2), 0, h / 2]
            else:
                size, pos = [e + t, t / 2, h / 2], [0, sy * (e + t / 2), h / 2]
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=pos, rgba=rgba)

    # ---- reset / randomization ----
    def reset(self):
        d = self.data
        mujoco.mj_resetData(self.model, d)
        d.qpos[:] = self.model.key_qpos[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")]
        d.ctrl[:] = self.info.home_ctrl

        n_bins, n_obj = len(BINS), len(OBJECTS)
        centers = self._sample_layout(n_bins, n_obj)
        bins, objs = centers[:n_bins], centers[n_bins:]

        for k, m in enumerate(self.env.bin_mocap):
            d.mocap_pos[m] = [bins[k][0], bins[k][1], TABLE_TOP]
            d.mocap_quat[m] = [1, 0, 0, 0]
            self.bin_xy[k] = bins[k]

        for i, (qadr, (_, shape)) in enumerate(zip(self.env.obj_qadr, OBJECTS)):
            d.qpos[qadr:qadr + 3] = [objs[i][0], objs[i][1], _rest_z(shape)]
            d.qpos[qadr + 3:qadr + 7] = _rest_quat(shape, self.rng.uniform(-np.pi, np.pi))

        # choose task + build instruction
        self.target_obj = int(self.rng.integers(n_obj))
        self.target_bin = int(self.rng.integers(n_bins))
        color, shape = OBJECTS[self.target_obj]
        self.instruction = (f"put the {color} {shape} into the "
                            f"{BINS[self.target_bin]} bin")

        mujoco.mj_forward(self.model, d)
        for _ in range(60):                     # settle objects onto the table
            mujoco.mj_step(self.model, d)
        return self.instruction

    def _sample_layout(self, n_bins, n_obj):
        """Rejection-sample non-overlapping centers: bins first, then objects."""
        pts, kinds = [], []
        def ok(p, kind):
            for q, k in zip(pts, kinds):
                sep = (SEP_BIN_BIN if kind == k == "bin" else
                       SEP_OBJ_OBJ if kind == k == "obj" else SEP_BIN_OBJ)
                if np.linalg.norm(p - q) < sep:
                    return False
            return True
        for kind, n, xr in [("bin", n_bins, BIN_REGION_X), ("obj", n_obj, REGION_X)]:
            for _ in range(n):
                for _try in range(200):
                    p = np.array([self.rng.uniform(*xr), self.rng.uniform(*REGION_Y)])
                    if ok(p, kind):
                        pts.append(p); kinds.append(kind); break
                else:
                    pts.append(p); kinds.append(kind)     # give up, accept last
        return pts

    # ---- success ----
    def obj_pos(self, i):
        return self.data.xpos[self.env.obj_bodies[i]].copy()

    def success(self):
        """True when the TARGET object is settled inside the TARGET bin."""
        p = self.obj_pos(self.target_obj)
        bx, by = self.bin_xy[self.target_bin]
        inside_xy = abs(p[0] - bx) < BIN_INNER and abs(p[1] - by) < BIN_INNER
        inside_z = TABLE_TOP < p[2] < TABLE_TOP + BIN_WALL_H
        return bool(inside_xy and inside_z)

    # ---- rendering ----
    def render(self, cam="scene", w=640, h=480):
        r = mujoco.Renderer(self.model, height=h, width=w)
        r.update_scene(self.data, camera=cam)
        return r.render()


def _look_at(eye, target, up=(0, 0, 1)):
    """xyaxes (camera x,y in world) so the camera at `eye` looks at `target`."""
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    z = eye - target
    z /= np.linalg.norm(z)                     # +z_cam points back toward the eye
    x = np.cross(up, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return [*x, *y]
