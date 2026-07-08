"""Push the local LeRobot dataset to the HF Hub, so cloud/Colab can pull it.

Cloud training reaches the Hub (unlike our offline local setup), so the clean
handoff is: push once from your Mac → `lerobot-train --dataset.repo_id=<repo>`
downloads it on the GPU box. Private by default.

Prereq (once):   .venv/bin/huggingface-cli login      # or export HF_TOKEN=...
Usage:           .venv/bin/python -m pick_place.cloud.push_dataset \
                     --repo-id Saurabh-sinha-209/pick_place --root 
"""
import argparse

from ..dataset import DEFAULT_ROOT,MASKED_ROOT
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-id", required=True, help="<hf-username>/pick_place")
    ap.add_argument("--root", default=MASKED_ROOT)
    ap.add_argument("--public", action="store_true", help="make the Hub repo public")
    args = ap.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("local/pick_place_masked", root=args.root)
    # relabel with the Hub repo id (push_to_hub uses self.repo_id)
    ds.repo_id = args.repo_id
    if getattr(ds, "meta", None) is not None:
        ds.meta.repo_id = args.repo_id

    print(f"pushing {ds.num_episodes} episodes / {ds.num_frames} frames → "
          f"https://huggingface.co/datasets/{args.repo_id}  (private={not args.public})")
    ds.push_to_hub(private=not args.public)
    print("done. On the GPU box:  lerobot-train --dataset.repo_id="
          f"{args.repo_id} ...")


if __name__ == "__main__":
    main()
