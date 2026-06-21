"""Core SE(3) operations: transforms, twists, adjoint, screw exp/log.

The SE(3) upgrade of mr/so3.py. A transform T = [[R, p], [0, 1]] bundles an
orientation R in SO(3) with a position p in R^3. Twists V = (omega, v) are the
6-vector velocity analog; screws are normalized twists; exp/log bridge se(3)
and SE(3) exactly as in so(3)/SO(3).
"""
import numpy as np

from .so3 import vec_to_so3, so3_to_vec, matrix_exp3, matrix_log3


# --- building / unpacking transforms ---------------------------------------

def rp_to_trans(R, p):
    """(R, p) -> 4x4 homogeneous transform T = [[R, p], [0, 1]]."""
    R = np.asarray(R, dtype=float)
    p = np.asarray(p, dtype=float).reshape(3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def trans_to_rp(T):
    """T -> (R, p). Inverse of rp_to_trans."""
    T = np.asarray(T, dtype=float)
    return T[:3, :3], T[:3, 3]


def trans_inv(T):
    """Inverse of T in SE(3) via the block form [[R^T, -R^T p], [0, 1]].

    No general 4x4 inverse needed: only R transposes; p picks up -R^T p
    because undoing the motion reverses the shift then the rotation.
    """
    R, p = trans_to_rp(T)
    Rt = R.T
    return rp_to_trans(Rt, -Rt @ p)


# --- twists / se(3) ---------------------------------------------------------

def vec_to_se3(V):
    """6-vector twist V = (omega, v) -> 4x4 matrix [V] = [[ [omega], v ], [0, 0]]."""
    V = np.asarray(V, dtype=float).reshape(6)
    omega, v = V[:3], V[3:]
    se3 = np.zeros((4, 4))
    se3[:3, :3] = vec_to_so3(omega)
    se3[:3, 3] = v
    return se3


def se3_to_vec(se3mat):
    """4x4 se(3) matrix -> 6-vector twist. Inverse of vec_to_se3."""
    se3mat = np.asarray(se3mat, dtype=float)
    return np.concatenate([so3_to_vec(se3mat[:3, :3]), se3mat[:3, 3]])


def adjoint(T):
    """T -> 6x6 adjoint [Ad_T] = [[R, 0], [ [p]R, R ]].

    Converts a twist's frame: V_a = [Ad_(T_ab)] V_b. The bottom-left [p]R block
    is the omega x p lever-arm correction for tracking a different point.
    """
    R, p = trans_to_rp(T)
    pR = vec_to_so3(p) @ R
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, :3] = pR
    Ad[3:, 3:] = R
    return Ad


# --- screws / exponential coordinates --------------------------------------

def _G(omega_hat, theta):
    """The position-block integrator G(theta) used in exp([S]theta)."""
    om = vec_to_so3(omega_hat)
    return (np.eye(3) * theta
            + (1 - np.cos(theta)) * om
            + (theta - np.sin(theta)) * (om @ om))


def matrix_exp6(S, theta):
    """Screw axis S = (omega, v) + distance theta -> T in SE(3).

    Assumes S normalized: either ||omega|| = 1 (rotation+pitch) or omega = 0
    and ||v|| = 1 (pure translation).
    """
    S = np.asarray(S, dtype=float).reshape(6)
    omega, v = S[:3], S[3:]
    if np.linalg.norm(omega) < 1e-12:          # pure translation
        return rp_to_trans(np.eye(3), v * theta)
    R = matrix_exp3(omega, theta)
    p = _G(omega, theta) @ v
    return rp_to_trans(R, p)


def matrix_log6(T):
    """T in SE(3) -> (S, theta) with S = (omega, v) a normalized screw axis.

    Inverse of matrix_exp6 (returns theta in [0, pi] for the rotation case).
    """
    R, p = trans_to_rp(T)
    if np.allclose(R, np.eye(3)):              # pure translation
        dist = np.linalg.norm(p)
        if dist < 1e-12:
            return np.zeros(6), 0.0
        return np.concatenate([np.zeros(3), p / dist]), dist
    omega_hat, theta = matrix_log3(R)
    om = vec_to_so3(omega_hat)
    # G^{-1}(theta) -- undoes G to recover the screw's linear part v
    Ginv = (np.eye(3) / theta
            - om / 2
            + (1 / theta - 1 / (2 * np.tan(theta / 2))) * (om @ om))
    v = Ginv @ p
    return np.concatenate([omega_hat, v]), theta


# --- screw axis geometry ----------------------------------------------------

def screw_to_axis(q, s_hat, h):
    """Geometric screw {point q, unit axis s_hat, pitch h} -> normalized S=(omega,v)."""
    q = np.asarray(q, dtype=float).reshape(3)
    s_hat = np.asarray(s_hat, dtype=float).reshape(3)
    omega = s_hat
    v = -np.cross(s_hat, q) + h * s_hat
    return np.concatenate([omega, v])
