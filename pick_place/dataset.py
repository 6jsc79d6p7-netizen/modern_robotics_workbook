"""Load the recorded LeRobot dataset locally (offline).

LeRobot 0.6.0 only reaches the HF Hub when the LOCAL metadata is *incomplete*.
A `finalize()`d dataset (see `recorder.Recorder.finalize`) loads fully offline
from its `root` directory — no Hub, no auth. Use this for inspection and as the
entry point for training.
"""
from .recorder import DEFAULT_ROOT


def load_dataset(root=DEFAULT_ROOT, repo_id="local/pick_place",
                 video_backend="pyav", **kw):
    """Load the local dataset. Pass e.g.
    `delta_timestamps={"action": [i/fps for i in range(H)]}` for action chunks.

    Defaults to the **pyav** decode backend: torchcodec (LeRobot's default) ships
    no loadable native lib on this macOS/arm setup, so frame reads crash with it.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    return LeRobotDataset(repo_id, root=str(root), video_backend=video_backend, **kw)
