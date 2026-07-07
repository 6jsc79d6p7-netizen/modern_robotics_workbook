"""Build the Franka Panda pick-place scene programmatically with MjSpec.

Why MjSpec instead of an XML `<include>`: it lets us (a) add a **TCP site** to
the Franka `hand` body — the frame our IK targets — without editing the pristine
`mujoco_menagerie` asset, and (b) later drop in randomized objects/bins for
Step 3 with a few lines. It also sidesteps the `meshdir`-relative-to-main-file
pain of XML includes.

The Franka's arm actuators (`actuator1..7`) are MuJoCo *position servos*
(`biastype=affine`, gain=kp, bias=(0,-kp,-kd)) — so `data.ctrl[0:7]` is the
desired joint angle. `actuator8` drives the gripper tendon, `ctrl` in [0, 255]
(0 = closed, 255 = open).
"""
import os
from dataclasses import dataclass

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
PANDA_XML = os.path.join(
    HERE, "..", "mujoco_menagerie", "franka_emika_panda", "panda.xml"
)

ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
ARM_ACTUATORS = [f"actuator{i}" for i in range(1, 8)]
GRIPPER_ACTUATOR = "actuator8"
# Franka nominal TCP: ~0.1034 m down the hand-frame z axis (between the pads).
TCP_OFFSET = 0.1034
GRAVCOMP_BODIES = [f"link{i}" for i in range(1, 8)] + ["hand", "left_finger", "right_finger"]


@dataclass
class SceneInfo:
    """Cached indices + home pose so the control loop never re-looks-up names."""
    tcp_site_id: int
    arm_qpos_ids: np.ndarray      # (7,) qpos addresses of joint1..7
    arm_dof_ids: np.ndarray       # (7,) dof addresses of joint1..7 (Jacobian cols)
    arm_act_ids: np.ndarray       # (7,) actuator ids for actuator1..7
    gripper_act_id: int
    arm_jnt_range: np.ndarray     # (7, 2) lower/upper limits for clamping IK
    target_mocap_id: int          # mocap index of the green target marker
    home_qpos: np.ndarray         # (nq,)
    home_ctrl: np.ndarray         # (nu,)


def _id(model, objtype, name):
    return mujoco.mj_name2id(model, objtype, name)


# table geometry (shared with the env so objects/bins sit on the top surface)
TABLE_POS = np.array([0.5, 0.0, 0.2])
TABLE_HALF = np.array([0.30, 0.40, 0.20])
TABLE_TOP = TABLE_POS[2] + TABLE_HALF[2]           # z of the table surface


def make_spec(add_table=True):
    """Build (but don't compile) the Franka scene spec. The env extends this
    with objects/bins/cameras before compiling."""
    spec = mujoco.MjSpec.from_file(PANDA_XML)
    wb = spec.worldbody

    # --- lighting + ground so launch_passive isn't a black void ---
    wb.add_light(pos=[0, 0, 3.0], dir=[0, 0, -1],
                 type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                 diffuse=[0.6, 0.6, 0.6], specular=[0.1, 0.1, 0.1])
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                size=[0, 0, 0.05], rgba=[0.3, 0.3, 0.35, 1])

    # --- a simple table in front of the base ---
    if add_table:
        wb.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
                    pos=TABLE_POS, size=TABLE_HALF, rgba=[0.55, 0.45, 0.35, 1])

    # --- TCP site on the hand: the frame our controller drives ---
    hand = spec.body("hand")
    # invisible (alpha 0): used only programmatically for pose/Jacobian, and a
    # visible marker here would clutter the wrist camera.
    hand.add_site(name="tcp", pos=[0, 0, TCP_OFFSET], size=[0.008],
                  rgba=[1, 0, 0, 0])

    # --- mocap marker visualizing where the target EE pose is ---
    # group 4 = hidden by default (so it's absent from the wrist cam render);
    # the live viewer re-enables group 4 to keep it visible third-person.
    target = wb.add_body(name="target", mocap=True, pos=[0.4, 0.0, 0.6])
    target.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.02], group=4,
                    rgba=[0.2, 1.0, 0.2, 0.4], contype=0, conaffinity=0)
    return spec


def apply_gravcomp(model):
    """MuJoCo applies the anti-gravity force per body, so the position PD only
    fights disturbances (no explicit inverse dynamics)."""
    for name in GRAVCOMP_BODIES:
        bid = _id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            model.body_gravcomp[bid] = 1.0


def scene_info(model):
    """Cache the Franka indices + home pose from a compiled model."""
    arm_jnt_ids = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in ARM_JOINTS]
    return SceneInfo(
        tcp_site_id=_id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp"),
        arm_qpos_ids=np.array([model.jnt_qposadr[j] for j in arm_jnt_ids]),
        arm_dof_ids=np.array([model.jnt_dofadr[j] for j in arm_jnt_ids]),
        arm_act_ids=np.array([_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
                              for a in ARM_ACTUATORS]),
        gripper_act_id=_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, GRIPPER_ACTUATOR),
        arm_jnt_range=model.jnt_range[arm_jnt_ids].copy(),
        target_mocap_id=model.body_mocapid[
            _id(model, mujoco.mjtObj.mjOBJ_BODY, "target")],
        home_qpos=model.key_qpos[_id(model, mujoco.mjtObj.mjOBJ_KEY, "home")].copy(),
        home_ctrl=model.key_ctrl[_id(model, mujoco.mjtObj.mjOBJ_KEY, "home")].copy(),
    )


def build_scene(add_table=True):
    """Compile the bare Franka scene and return (model, SceneInfo)."""
    spec = make_spec(add_table)
    model = spec.compile()
    apply_gravcomp(model)
    return model, scene_info(model)


def tcp_pose(model, data, info):
    """Current TCP pose as (p, R) in the world frame."""
    p = data.site_xpos[info.tcp_site_id].copy()
    R = data.site_xmat[info.tcp_site_id].reshape(3, 3).copy()
    return p, R
