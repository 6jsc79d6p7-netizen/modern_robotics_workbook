"""Load the recorded LeRobot dataset locally (offline).

LeRobot 0.6.0 only reaches the HF Hub when the LOCAL metadata is *incomplete*.
A `finalize()`d dataset (see `recorder.Recorder.finalize`) loads fully offline
from its `root` directory — no Hub, no auth. Use this for inspection and as the
entry point for training.
"""
from .recorder import DEFAULT_ROOT, MASK_ROOT

MASKED_ROOT= MASK_ROOT


def load_dataset(root=DEFAULT_ROOT, repo_id="local/pick_place",
                 video_backend=None, **kw):
    """Load the local dataset. Pass e.g.
    `delta_timestamps={"action": [i/fps for i in range(H)]}` for action chunks.

    `video_backend=None` uses LeRobot's default (**torchcodec**), which needs a
    system FFmpeg — `brew install ffmpeg`. Pass `video_backend="pyav"` for the
    self-contained fallback (bundles its own FFmpeg; no system dependency).
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    return LeRobotDataset(repo_id, root=str(root), video_backend=video_backend, **kw)
