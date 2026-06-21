"""Core SO(3) operations: skew-symmetric vectors, exp/log (Rodrigues' formula)."""
import numpy as np


def vec_to_so3(omega):
    """3-vector -> 3x3 skew-symmetric matrix [omega], where [omega] @ p == omega x p."""
    omega = np.asarray(omega, dtype=float)
    return np.array([
        [0,         -omega[2],  omega[1]],
        [omega[2],   0,        -omega[0]],
        [-omega[1],  omega[0],  0],
    ])


def so3_to_vec(so3mat):
    """3x3 skew-symmetric matrix -> 3-vector. Inverse of vec_to_so3."""
    return np.array([so3mat[2, 1], so3mat[0, 2], so3mat[1, 0]])


def matrix_exp3(omega_hat, theta):
    """Rodrigues' formula: axis omega_hat (unit vector) + angle theta -> R in SO(3)."""
    omg_mat = vec_to_so3(omega_hat)
    return np.eye(3) + np.sin(theta) * omg_mat + (1 - np.cos(theta)) * (omg_mat @ omg_mat)


def matrix_log3(R):
    """R in SO(3) -> (omega_hat, theta). Inverse of matrix_exp3."""
    cos_theta = np.clip((np.trace(R) - 1) / 2, -1, 1)
    theta = np.arccos(cos_theta)

    if np.isclose(theta, 0):
        return np.zeros(3), 0.0

    if np.isclose(theta, np.pi):
        M = (R + np.eye(3)) / 2
        i = np.argmax(np.diag(M))  # avoid dividing by a near-zero entry
        col = M[:, i]
        omega_hat = col / np.sqrt(col[i])
        return omega_hat, theta

    omg_mat = (R - R.T) / (2 * np.sin(theta))
    return so3_to_vec(omg_mat), theta
