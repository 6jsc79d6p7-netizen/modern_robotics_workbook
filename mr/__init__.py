"""Shared helpers built up while working through Modern Robotics.

Functions get promoted here from notebooks once they're reusable
(rotations, transforms, twists, screws, Jacobians, ...).
"""
from .so3 import vec_to_so3, so3_to_vec, matrix_exp3, matrix_log3
from .se3 import (
    rp_to_trans, trans_to_rp, trans_inv,
    vec_to_se3, se3_to_vec, adjoint,
    matrix_exp6, matrix_log6, screw_to_axis,
)
from .plotting import draw_frame, setup_3d_axes

__all__ = [
    "vec_to_so3", "so3_to_vec", "matrix_exp3", "matrix_log3",
    "rp_to_trans", "trans_to_rp", "trans_inv",
    "vec_to_se3", "se3_to_vec", "adjoint",
    "matrix_exp6", "matrix_log6", "screw_to_axis",
    "draw_frame", "setup_3d_axes",
]
