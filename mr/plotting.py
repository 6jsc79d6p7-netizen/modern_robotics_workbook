"""Shared matplotlib 3D helpers for visualizing frames and vectors."""
import numpy as np


def draw_frame(ax, R, origin=(0, 0, 0), colors=("tab:red", "tab:green", "tab:blue"),
                labels=("x", "y", "z"), length=1.0, lw=2.5, style="-", alpha=1.0):
    """Plot R's columns as arrows from origin -- each column is a body axis."""
    o = np.array(origin, dtype=float)
    for i in range(3):
        axis = R[:, i] * length
        ax.quiver(*o, *axis, color=colors[i], linewidth=lw, linestyle=style,
                  alpha=alpha, arrow_length_ratio=0.15)
        ax.text(*(o + axis * 1.15), labels[i], color=colors[i], fontsize=12, fontweight="bold")


def setup_3d_axes(ax, lim=1.5, title=""):
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim, lim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    ax.set_box_aspect([1, 1, 1])
