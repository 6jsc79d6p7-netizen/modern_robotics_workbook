"""TargetSource — the abstraction that sits *above* the controller.

The controller is a pure follower; it never knows where its target comes from.
A TargetSource produces, each tick, an absolute TCP target pose + gripper cmd.
Swapping the source is how the whole rig changes personality:

    DummyOrbitSource  — a scripted path, no human, used to bring up the controller
    (later) TeleopSource  — phone WebXR pose, clutched relative mapping
    (later) ScriptSource  — privileged-state waypoints → trajectory generation

All three will emit the *same* stream shape so the policy sees one action
distribution. For Step 1 the target is absolute (easiest to visualize); teleop
and the expert integrate their EE-deltas internally.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class TargetCommand:
    pos: np.ndarray      # (3,) desired TCP position, world frame
    R: np.ndarray        # (3,3) desired TCP orientation, world frame
    gripper: float       # actuator8 ctrl, [0, 255]  (0 closed, 255 open)


class TargetSource(ABC):
    def reset(self, model, data, info):
        """Called on (re)start; default no-op."""

    @abstractmethod
    def __call__(self, t, model, data, info) -> TargetCommand:
        ...


class DummyOrbitSource(TargetSource):
    """Orbit the TCP on a horizontal circle with a gentle vertical bob and roll,
    cycling the gripper. Exercises translation *and* rotation IK with no teleop —
    the bring-up test for the controller + viewer."""

    def __init__(self, radius=0.08, period=10.0, bob=0.04, roll=0.15):
        self.radius = radius
        self.period = period
        self.bob = bob
        self.roll = roll          # rad of orientation wobble about world x
        self.center = None
        self.R0 = None

    def reset(self, model, data, info):
        from .scene import tcp_pose
        p, R = tcp_pose(model, data, info)
        self.center = p.copy()
        self.R0 = R.copy()

    def __call__(self, t, model, data, info) -> TargetCommand:
        w = 2 * np.pi / self.period
        pos = self.center + np.array([
            self.radius * np.cos(w * t),
            self.radius * np.sin(w * t),
            self.bob * np.sin(2 * w * t),
        ])
        # small roll about world x so rotation tracking is visible
        a = self.roll * np.sin(w * t)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(a), -np.sin(a)],
                       [0, np.sin(a),  np.cos(a)]])
        R = Rx @ self.R0
        # gripper cycles open/closed
        gripper = 127.5 * (1 + np.sin(0.5 * w * t))
        return TargetCommand(pos=pos, R=R, gripper=gripper)
