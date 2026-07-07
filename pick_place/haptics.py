"""Contact → haptic events, edge-triggered, two channels (the agreed design):

  WARNING (vibrate): arm/hand/fingers touch the environment (table, floor, bin
                     walls) — you rammed the robot into something.
  GRASP  (sound):    the gripper fingers touch an object — contact confirmation.

Both fire once on the rising edge (not every contact tick), so a sustained
contact signals once. `poll()` returns the list of channels that fired this step.
"""
import mujoco

ARM_BODIES = {f"link{i}" for i in range(1, 8)} | {"hand", "left_finger", "right_finger"}
FINGER_BODIES = {"left_finger", "right_finger"}
ENV_GEOM_NAMES = {"floor", "table"}


def classify_geoms(model):
    """Return (arm, env, finger, obj) geom-id sets.
    env = table/floor + all bin walls; obj = the graspable objects."""
    arm, env, finger, obj = set(), set(), set(), set()
    for gid in range(model.ngeom):
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gid])
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if bname in ARM_BODIES:
            arm.add(gid)
        if bname in FINGER_BODIES:
            finger.add(gid)
        if gname in ENV_GEOM_NAMES or (bname and bname.startswith("bin_")):
            env.add(gid)
        if gname and gname.startswith("obj"):
            obj.add(gid)
    return arm, env, finger, obj


class CollisionMonitor:
    def __init__(self, model):
        self.arm, self.env, self.finger, self.obj = classify_geoms(model)
        self._warn = False
        self._grasp = False

    def _touching(self, data, A, B):
        for i in range(data.ncon):
            c = data.contact[i]
            a, b = c.geom1, c.geom2
            if (a in A and b in B) or (b in A and a in B):
                return True
        return False

    def poll(self, data):
        """Return channels firing this tick: 'vibrate' (warn) and/or 'sound' (grasp)."""
        events = []
        warn = self._touching(data, self.arm, self.env)
        if warn and not self._warn:
            events.append("vibrate")
        self._warn = warn

        grasp = self._touching(data, self.finger, self.obj)
        if grasp and not self._grasp:
            events.append("sound")
        self._grasp = grasp
        return events
