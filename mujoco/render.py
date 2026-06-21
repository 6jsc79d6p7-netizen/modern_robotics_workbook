"""Render the 3R arm at a given joint configuration to a PNG.

Usage:  ../.venv/bin/python render.py            # renders the example pose
A free camera auto-frames the arm by aiming at the mean of the joint positions.
"""
import os
import sys

import numpy as np
import mujoco
import imageio

HERE = os.path.dirname(os.path.abspath(__file__))


def render_pose(thetas, out_name, azimuth=120, elevation=-18):
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "arm3r.xml"))
    data = mujoco.MjData(model)
    data.qpos[:] = thetas
    mujoco.mj_forward(model, data)

    # auto-frame: aim the camera at the centroid of the body origins
    centroid = data.xpos[1:].mean(axis=0)        # skip world body 0
    cam = mujoco.MjvCamera()
    cam.lookat[:] = centroid
    cam.distance = 1.2
    cam.azimuth = azimuth
    cam.elevation = elevation

    # draw an axis triad (x=red, y=green, z=blue) at every site:
    # the world origin {s} and the end-effector {b}
    scene_option = mujoco.MjvOption()
    scene_option.frame = mujoco.mjtFrame.mjFRAME_SITE

    renderer = mujoco.Renderer(model, height=720, width=960)
    renderer.update_scene(data, cam, scene_option=scene_option)
    img = renderer.render()
    path = os.path.join(HERE, out_name)
    imageio.imwrite(path, img)
    print("wrote", path)


if __name__ == "__main__":
    render_pose([0, 0, 0], "arm3r_home.png")
    render_pose([np.pi / 2, -np.pi / 4, np.pi / 3], "arm3r_pose.png")
