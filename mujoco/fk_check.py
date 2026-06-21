"""Validate our Product-of-Exponentials forward kinematics against MuJoCo.

We define the 3R arm's home pose M and its space-frame screw axes S_i by hand
(the FK "calculation"), evaluate T(theta) = e^{[S1]t1} e^{[S2]t2} e^{[S3]t3} M
with mr/se3.py, and compare to the end-effector pose MuJoCo computes via
mj_forward. If our screw table is right, they match to machine precision.
"""
import os
import sys

import numpy as np
import mujoco

sys.path.insert(0, os.path.abspath(".."))  # project root, for `import mr`
from mr import matrix_exp6, rp_to_trans, trans_inv, adjoint

np.set_printoptions(precision=4, suppress=True)

HERE = os.path.dirname(os.path.abspath(__file__))
L1, L2, L3 = 0.30, 0.25, 0.20

# --- PoE ingredients, read off the home position by hand --------------------
M = rp_to_trans(np.eye(3), [L1 + L2 + L3, 0, 0])     # end-effector at home

# screw axes S_i = (omega_i, v_i) in the space frame, v = -omega x q
S = np.array([
    [0, 0, 1,   0, 0, 0],            # joint 1: about z at origin
    [0, 1, 0,   0, 0, L1],           # joint 2: about y at (L1,0,0)
    [0, 1, 0,   0, 0, L1 + L2],      # joint 3: about y at (L1+L2,0,0)
], dtype=float)


# body-form screw axes B_i = [Ad_{M^-1}] S_i (joint axes seen from the hand)
B = np.array([adjoint(trans_inv(M)) @ Si for Si in S])


def fk_poe(thetas):
    """Forward kinematics via space-form PoE:  e^{[S1]t1}...e^{[Sn]tn} M."""
    T = np.eye(4)
    for Si, th in zip(S, thetas):
        T = T @ matrix_exp6(Si, th)
    return T @ M


def fk_poe_body(thetas):
    """Forward kinematics via body-form PoE:  M e^{[B1]t1}...e^{[Bn]tn}."""
    T = M.copy()
    for Bi, th in zip(B, thetas):
        T = T @ matrix_exp6(Bi, th)
    return T


def fk_mujoco(model, data, thetas):
    """Forward kinematics via MuJoCo: set joints, run mj_forward, read the site."""
    data.qpos[:] = thetas
    mujoco.mj_forward(model, data)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee")
    T = np.eye(4)
    T[:3, :3] = data.site_xmat[sid].reshape(3, 3)
    T[:3, 3] = data.site_xpos[sid]
    return T


def main():
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "arm3r.xml"))
    data = mujoco.MjData(model)

    rng = np.random.default_rng(0)
    print("Comparing PoE (space & body form) vs MuJoCo for random joint angles:\n")
    worst = 0.0
    for k in range(5):
        thetas = rng.uniform(-np.pi, np.pi, size=3)
        Tm = fk_mujoco(model, data, thetas)
        e_space = np.abs(fk_poe(thetas) - Tm).max()
        e_body = np.abs(fk_poe_body(thetas) - Tm).max()
        worst = max(worst, e_space, e_body)
        print(f"theta = {np.round(thetas, 3)}   "
              f"space err = {e_space:.1e}   body err = {e_body:.1e}")
    print(f"\nworst-case error over all samples: {worst:.2e}")
    print("Both PoE forms match MuJoCo:", worst < 1e-6)

    # show one pose in full
    thetas = np.array([np.pi / 2, -np.pi / 4, np.pi / 3])
    print(f"\nExample pose theta = {np.round(thetas,3)}:")
    print("T_poe =\n", fk_poe(thetas))
    print("end-effector position:", fk_poe(thetas)[:3, 3])


if __name__ == "__main__":
    main()
