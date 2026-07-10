"""Audit a recorded LeRobot pick-place dataset for empty-gripper "placements".

Background: env.success() used to collapse to a pure xy-proximity test, so an
object knocked/rolled into the bin footprint counted as a success even though the
gripper closed on *nothing*. That poisoned ~15% of the original scripted dataset
with frames that demonstrate the exact deploy failure (open the gripper over the
bin with no object). The env fix (LIFT_MARGIN + track()) stops new *scripted* data
from being contaminated; this script re-checks any dataset from the recorded
signals alone.

Two gripper metrics per episode (state[9] = measured width, action[6] = command):
  - wmin      : MIN width over the whole episode.
  - w@release : MIN width in the 8 frames before the FINAL close->open (the
                placement/release). This is what actually matters — was the
                object HELD when the gripper opened over the bin.

Why two: the scripted expert closes the gripper exactly once, so wmin catches its
empty grasps. But a HUMAN teleoperator fumbles — a transient empty-close (missed,
reopened, re-grasped) drives wmin to ~0 even on a perfectly clean placement. So we
classify on **w@release**:
  - w@release < --hold-thresh  -> EMPTY AT PLACEMENT  (real contamination; exclude)
  - wmin < --grip-thresh but placed clean -> fumble-then-clean (KEEP; often a
    useful missed-grasp -> recovery demo, which the scripted data never has)

    .venv/bin/python -m pick_place.audit_dataset                     # default masked root
    .venv/bin/python -m pick_place.audit_dataset --root pick_place/data/pick_place_masked_v2
    .venv/bin/python -m pick_place.audit_dataset --write-bad exclude.txt
"""
import argparse
import glob
import json
import os

import numpy as np
import pandas as pd

from .recorder import MASK_ROOT, STATE_NAMES, ACTION_NAMES

GW_IDX = STATE_NAMES.index("gripper_width")
GRIP_ACT_IDX = ACTION_NAMES.index("gripper")


def _load_frames(root):
    files = sorted(glob.glob(os.path.join(root, "data", "**", "*.parquet"),
                             recursive=True))
    if not files:
        raise SystemExit(f"no parquet under {root}/data — is this a LeRobot root?")
    return pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)


def _load_meta(root):
    path = os.path.join(root, "episode_meta.jsonl")
    meta = {}
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                r = json.loads(line)
                meta[r["episode_index"]] = r
    return meta


def _episode_metrics(g):
    """Return (wmin, w_at_release) for one episode."""
    st = np.stack(g["observation.state"].values)
    act = np.stack(g["action"].values)
    w = st[:, GW_IDX]
    closed = (act[:, GRIP_ACT_IDX] < 128).astype(int)
    opens = np.where(np.diff(closed) == -1)[0]      # final close->open = the release
    rel = opens[-1] if len(opens) else len(w) - 1
    w_release = float(w[max(0, rel - 8):rel + 1].min())
    return float(w.min()), w_release


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=MASK_ROOT)
    # 0.010 sits in the empirical gap: genuinely-empty placements cluster at
    # w@release ~0.000-0.002, real grasps start ~0.015 (a firm grip on the small
    # capsule/cylinder compresses the fingers to ~0.02-0.03 — still a real hold).
    ap.add_argument("--hold-thresh", type=float, default=0.010,
                    help="width at the final release below this = gripper was EMPTY "
                         "when it opened over the bin (real contamination)")
    ap.add_argument("--grip-thresh", type=float, default=0.015,
                    help="episode min width below this = the gripper went empty at "
                         "some point (a fumble if the placement itself was clean)")
    ap.add_argument("--write-bad", default=None,
                    help="write empty-at-placement episode indices (one per line) here")
    args = ap.parse_args()

    df = _load_frames(args.root)
    meta = _load_meta(args.root)
    n_ep = df["episode_index"].nunique()
    print(f"[audit] {args.root}: {len(df)} frames, {n_ep} episodes\n")

    empty_place, fumble = [], []
    for ep, g in df.groupby("episode_index"):
        wmin, wrel = _episode_metrics(g)
        src = meta.get(ep, {}).get("source", "?")
        if wrel < args.hold_thresh:
            empty_place.append((int(ep), src, wmin, wrel))
        elif wmin < args.grip_thresh:
            fumble.append((int(ep), src, wmin, wrel))

    bad = [ep for ep, *_ in empty_place]
    print(f"EMPTY-AT-PLACEMENT (exclude): {len(bad)}/{n_ep}  "
          f"({100*len(bad)/max(n_ep,1):.1f}%)")
    for ep, src, wmin, wrel in empty_place:
        print(f"    ep {ep:>4} [{src}]  wmin={wmin:.3f}  w@release={wrel:.3f}")
    print(f"\nfumble-then-clean placement (KEEP — held at release, useful recovery "
          f"signal): {len(fumble)}/{n_ep}")
    if fumble:
        print("    indices:", [ep for ep, *_ in fumble])

    if args.write_bad and bad:
        with open(args.write_bad, "w") as f:
            f.write("\n".join(str(e) for e in bad) + "\n")
        print(f"\nwrote {len(bad)} exclude indices -> {args.write_bad}")

    print("\nVERDICT:", "CLEAN ✓" if not bad
          else f"{len(bad)} empty-at-placement episode(s) — exclude before training")


if __name__ == "__main__":
    main()
