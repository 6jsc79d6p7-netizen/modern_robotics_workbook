"""LeRobot recorder — writes teleop demos as a LeRobot dataset (Step 5).

Per timestep (downsampled to `fps` from the 500 Hz loop) it logs:
  observation.images.scene / .wrist  — RAW camera renders (the wrist obs is the
                                        unrotated sensor, NOT the operator-aligned
                                        live view — that rotation is a human aid).
  observation.state  — [ee_pos(3), ee_rot6d(6), gripper_width(1), joint_pos(7)]
  action             — base-frame EE-delta to the NEXT pose:
                       [Δpos(3), Δrot_rotvec(3), gripper_cmd(1)]
  task               — the language instruction (LeRobot task table)

The action needs the next pose, so we hold one frame pending and emit it once the
following step arrives (the last frame of an episode has no successor and is
dropped). Structured task fields go to a sidecar `episode_meta.jsonl` — LeRobot's
only native string channel is `task`. See notes/proj_lerobot_format.md.
"""
import json
import os

import numpy as np
import mujoco

from lerobot.datasets.lerobot_dataset import LeRobotDataset

from mr.so3 import matrix_log3
from .scene import tcp_pose
from .env import CAMERAS, render_obs

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROOT = os.path.join(HERE, "data", "pick_place")
# 2-camera + target-highlight-mask dataset (kept separate so the original
# 3-camera data stays intact). Point run_script/train/infer at this.
MASK_ROOT = os.path.join(HERE, "data", "pick_place_masked")

# gripper_width = MEASURED finger aperture; gripper_cmd = the COMMANDED gripper
# (data.ctrl, 0=closed..255=open). The pair gives the policy a grasp/stall signal:
# commanded-closed but width won't collapse ⇒ something is held. Both exist on real
# hardware too (e.g. SO-100: present-position vs goal-position), so it transfers.
STATE_NAMES = (["ee_x", "ee_y", "ee_z"] + [f"rot6d_{i}" for i in range(6)]
               + ["gripper_width", "gripper_cmd"] + [f"joint{i}" for i in range(1, 8)])
ACTION_NAMES = ["dx", "dy", "dz", "drx", "dry", "drz", "gripper"]


def _features(img_hw):
    h, w = img_hw
    vid = {"dtype": "video", "shape": (h, w, 3),
           "names": ["height", "width", "channels"]}
    feats = {key: dict(vid) for key, _ in CAMERAS}
    feats["observation.state"] = {"dtype": "float32", "shape": (len(STATE_NAMES),),
                                  "names": STATE_NAMES}
    feats["action"] = {"dtype": "float32", "shape": (len(ACTION_NAMES),),
                       "names": ACTION_NAMES}
    return feats


class Recorder:
    def __init__(self, env, repo_id="local/pick_place", root=DEFAULT_ROOT,
                 fps=15, img_hw=(256, 256)):
        self.env = env
        self.model = env.model
        self.info = env.info
        self.img_hw = img_hw
        self.root = root
        self.renderer = mujoco.Renderer(self.model, height=img_hw[0], width=img_hw[1])
        # second renderer in segmentation mode → per-pixel geom ids for the mask
        self.seg_renderer = mujoco.Renderer(self.model, height=img_hw[0], width=img_hw[1])
        self.seg_renderer.enable_segmentation_rendering()
        fj = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
              for n in ("finger_joint1", "finger_joint2")]
        self.finger_qadr = [self.model.jnt_qposadr[j] for j in fj if j >= 0]

        # create a new dataset, or resume (append) an existing one at this root —
        # so scripts and teleop can pool into ONE dataset. Resuming needs the
        # prior session to have been finalize()d (see finalize()).
        if os.path.exists(os.path.join(root, "meta", "info.json")):
            self.ds = LeRobotDataset.resume(repo_id, root=root)
            print(f"[rec] resuming dataset at {root} ({self.ds.num_episodes} episodes)")
        else:
            self.ds = LeRobotDataset.create(
                repo_id, fps=fps, features=_features(img_hw), root=root,
                robot_type="franka", use_videos=True)
        self._meta_path = os.path.join(root, "episode_meta.jsonl")
        self._pending = None
        self._n = 0
        self._instruction = ""

    def finalize(self):
        """Flush writers so the dataset is valid + loadable offline. Idempotent.
        MUST be called at the end of a recording session."""
        self.ds.finalize()

    # ---- episode lifecycle ----
    def start_episode(self, instruction):
        self._pending = None
        self._n = 0
        self._instruction = instruction

    def record_step(self, data, gripper_cmd):
        obs, pose = self._observe(data)
        if self._pending is not None:
            pobs, ppose, pgrip = self._pending
            pobs["action"] = self._delta(ppose, pose, pgrip)
            pobs["task"] = self._instruction
            self.ds.add_frame(pobs)
            self._n += 1
        self._pending = (obs, pose, float(gripper_cmd))

    def save_episode(self, target_color, target_shape, target_bin, success=True,
                     grasp_verified=None, held_cleanly=None, source="script"):
        """Finalize the current episode (drops the last, action-less frame).
        `source` ('script' | 'teleop') is logged to the sidecar for co-training.
        `success` is the REAL env.success() result (not a hardcoded True);
        `grasp_verified` records whether the target was picked up; `held_cleanly`
        whether it was held the whole way (no empty grasp / mid-carry drop) — so a
        recurrence of the empty-gripper-placement bug is auditable from the meta."""
        if self._n < 2:
            self.discard_episode()
            return False
        self._pending = None
        idx = self.ds.num_episodes
        self.ds.save_episode(parallel_encoding=False)   # synchronous: files flushed
        meta = {"episode_index": idx, "source": source,
                "target_color": target_color, "target_shape": target_shape,
                "target_bin": target_bin, "success": bool(success)}
        if grasp_verified is not None:
            meta["grasp_verified"] = bool(grasp_verified)
        if held_cleanly is not None:
            meta["held_cleanly"] = bool(held_cleanly)
        with open(self._meta_path, "a") as f:
            f.write(json.dumps(meta) + "\n")
        print(f"[rec] saved episode {idx}  ({self._n} frames)")
        return True

    def discard_episode(self):
        self._pending = None
        self._n = 0
        if getattr(self.ds, "has_pending_frames", False):
            self.ds.clear_episode_buffer()

    # ---- feature extraction ----
    def _observe(self, data):
        obs = {}
        keep = self.env.target_keep_geoms()                  # target obj + target bin
        for key, cam in CAMERAS:
            obs[key] = render_obs(self.renderer, self.seg_renderer, data, cam, keep).copy()
        p, R = tcp_pose(self.model, data, self.info)
        rot6d = R[:, :2].T.reshape(6)                        # 6D rotation rep
        gw = float(sum(data.qpos[a] for a in self.finger_qadr))
        gcmd = float(data.ctrl[self.info.gripper_act_id])     # commanded gripper (0..255)
        jp = data.qpos[self.info.arm_qpos_ids]
        obs["observation.state"] = np.concatenate(
            [p, rot6d, [gw], [gcmd], jp]).astype(np.float32)
        return obs, (p, R)

    def _delta(self, pose0, pose1, gripper):
        (p0, R0), (p1, R1) = pose0, pose1
        dpos = p1 - p0                                       # base frame
        axis, theta = matrix_log3(R1 @ R0.T)
        return np.concatenate([dpos, axis * theta, [gripper]]).astype(np.float32)
