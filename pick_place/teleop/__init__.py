"""Phone WebXR teleop for the pick_place controller (Step 2).

Self-contained copy of the tested simple_slam server<->browser plumbing,
stripped of the SLAM pipeline. The phone streams its ARKit/ARCore 6-DoF pose
(hardware VIO) over WSS; the MuJoCo process consumes it as a TargetSource.
"""
