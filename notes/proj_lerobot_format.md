# Project note — the LeRobot dataset format (Step 4 recording target)

> **What this is.** A reference for how we store pick-and-place demos so Steps 6–7
> (train a conditioned flow-matching / diffusion policy → π0) can consume them with
> zero conversion. We adopt **LeRobot format from day 1** (see
> [`proj_pick_place_plan.md`](proj_pick_place_plan.md)). Emphasis on the **language
> instruction channel**, since that's what makes the policy *conditioned*.

---

## 1. Why LeRobot

It's the de-facto standard for imitation-learning datasets: HuggingFace-hosted,
and directly ingested by the training code we're targeting (LeRobot's diffusion
policy / ACT, and π0 via openpi). Rolling our own format means writing a converter
later; LeRobot means "point the trainer at the dataset." Its record is exactly our
tuple — `(camera obs, proprioception, action, language)` — the same
`(camera, EE-pose, gripper)` shape as UMI (see
[`proj_data_source_spectrum.md`](proj_data_source_spectrum.md) §5).

## 2. On-disk layout (v2.x)

```
<repo_id>/
├── meta/
│   ├── info.json        # fps, robot_type, feature specs, counts, video/data paths
│   ├── stats.json       # per-feature mean/std/min/max → normalization at train time
│   ├── tasks.jsonl      # task_index → instruction string   ← the language table
│   └── episodes.jsonl   # per episode: {episode_index, tasks:[...], length}
├── data/
│   └── chunk-000/episode_000000.parquet   # one ROW per timestep (non-video features)
└── videos/
    └── chunk-000/observation.images.<cam>/episode_000000.mp4   # frames as video
```

- **Images live in mp4** (encoded video), referenced by timestamp — not stuffed
  into the parquet. Cheap storage, fast decode.
- **The parquet row** per timestep holds the numeric features + bookkeeping:
  `timestamp, frame_index, episode_index, index` (global), and **`task_index`**.

> **What 0.6.0 actually writes** (verified): `meta/tasks.parquet` (not `.jsonl`),
> `meta/episodes/chunk-*/file-*.parquet` (episode metadata, **buffered, flushed
> every 10 episodes + on finalize**), a consolidated `data/chunk-000/file-000.parquet`,
> and one mp4 per camera under `videos/observation.images.<cam>/chunk-000/`.
>
> **Loading offline (solved):** LeRobot hits the Hub *only when the local
> metadata is incomplete*. So (1) call **`dataset.finalize()`** at the end of a
> recording session (flushes the trailing episode-metadata buffer + closes
> writers — `run_script`/`run_teleop` do this automatically), then (2)
> `LeRobotDataset(repo_id, root=)` loads fully offline. Use
> `pick_place.dataset.load_dataset`.
> - **Decode backend = `pyav`**: torchcodec (LeRobot's default) has no loadable
>   native lib on this macOS/arm setup; pyav works.
> - **Append / merge:** `LeRobotDataset.resume(repo_id, root=)` adds episodes to an
>   existing (finalized) dataset — how scripts + teleop pool into one dataset. The
>   `Recorder` auto-creates-or-resumes by checking `meta/info.json`.
> - `save_episode(parallel_encoding=False)` for synchronous flushing.

## 3. Our record schema (the frame)

| key | dtype / shape | meaning |
|---|---|---|
| `observation.images.scene` | video (H,W,3) u8 | fixed scene camera |
| `observation.images.wrist` | video (H,W,3) u8 | wrist camera |
| `observation.state` | float32 (N,) | proprioception: joint pos (7) + gripper width + EE pose |
| `action` | float32 (M,) | **EE-delta** (Δpos 3 + Δrot 3) + gripper cmd (1) |
| `task` | string | the instruction (stored via the task table, see §4) |

Action space = **EE-delta** (locked in the plan); note the delta *frame* (base vs
EE) is the one open action-representation decision — see
[`proj_pick_place_plan.md`](proj_pick_place_plan.md).

## 4. How the INSTRUCTION is stored (the part that matters)

**Raw text, never a pre-computed embedding.**

- Each frame is recorded with `task="put the red box into the purple bin"`.
- LeRobot **deduplicates** strings into `meta/tasks.jsonl` (`task_index → string`)
  and stores a per-frame **`task_index`** in the parquet. So the text isn't
  repeated on every row — it's one integer + a lookup table.
- Our instruction is **constant per episode**, so every frame in an episode shares
  the same `task_index`. That redundancy is deliberate: training draws samples
  per-timestep as `(obs_t, task, action_chunk)`, so having the task on every frame
  keeps the dataloader trivial.

**Why raw text (not an embedding):**
- Keeps the dataset encoder-agnostic — tokenize/embed with CLIP, SigLIP, T5, or a
  VLM's tokenizer *at train time*, swappable later.
- π0 ingests raw language through its VLM (PaliGemma); it wants tokens, not someone
  else's frozen vector.
- Text is tiny — no storage reason to pre-embed.

**At train time:** `task string → tokenizer → text encoder → conditioning`. For a
flow-matching/diffusion policy the language embedding conditions the denoiser
alongside the visual features (FiLM / cross-attention / concat into the global
cond vector). In a VLA the language tokens enter the transformer with the image
tokens. The dataset's job ends at handing over `(images, proprio, action, text)`.

## 5. The structured task-spec (store it alongside)

The NL string is what the policy trains on, but keep the **fields it was generated
from** too — for success labels, filtering, analysis, and paraphrase augmentation:

```
source ('script'|'teleop'), target_color, target_shape, target_bin, success
```
(`source` lets you weight/filter the teleop quality slice vs scripted volume at
train time — keyed by `episode_index`.)

**Nuance:** LeRobot's only first-class *string* channel is `task`; other per-frame
features are numeric/video. So store the structured spec as **integer-encoded
features** (e.g. `target_bin_idx`) *or* a **sidecar** `episode_meta.jsonl` keyed by
`episode_index`. A sidecar is simplest and keeps the LeRobot features clean.

## 6. Two data-quality contracts

1. **Same scene + different instruction → different action.** This is what teaches
   the policy to *read* language instead of learning a prior ("always grab red").
   Our env re-rolls the target each reset; just ensure the dataset covers every
   object/bin as a target enough times.
2. **Template overfitting.** Every instruction is one fixed phrasing → the policy
   can latch onto exact tokens. Plan: keep the **canonical string + structured
   spec**, and **paraphrase at train time** from the fields (dial diversity up
   later without re-collecting). Alternative: sample varied phrasings at record
   time.

## 7. Action chunking (flow-matching needs it)

Flow-matching / diffusion / ACT predict an **action chunk** (horizon H), not one
step. LeRobot serves chunks via `delta_timestamps`, e.g.
`{"action": [0, 1/fps, …, (H-1)/fps]}` returns `action` as an `(H, M)` window per
sample. Record at a policy-decision rate (~10–30 Hz, **downsampled** from the
500 Hz control loop — see the frequency note), not at 500 Hz.

## 8. API sketch (signatures drift across versions — treat as shape, not gospel)

```python
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset.create(repo_id="me/pick_place", fps=15, robot_type="franka",
    features={
      "observation.images.scene": {"dtype":"video","shape":(H,W,3),"names":["h","w","c"]},
      "observation.images.wrist": {"dtype":"video","shape":(H,W,3),"names":["h","w","c"]},
      "observation.state":        {"dtype":"float32","shape":(N,)},
      "action":                   {"dtype":"float32","shape":(M,)},
    })

# per timestep of a demo:
ds.add_frame({"observation.images.scene": scene, "observation.images.wrist": wrist,
              "observation.state": state, "action": action},
             task="put the red box into the purple bin")
ds.save_episode()          # encodes the mp4s + writes the parquet + updates meta

# train-time load with action chunking:
ds = LeRobotDataset("me/pick_place",
                    delta_timestamps={"action": [i/15 for i in range(H)]})
item = ds[t]               # tensors + item["task"] (string) + item["task_index"]
```

## 9. Cheat-sheet

- **Adopt LeRobot v2** — images→mp4, per-timestep numeric features→parquet.
- **Instruction = raw string via the `task` table** (deduped to `task_index`);
  **never store embeddings** — tokenize at train time.
- **Also store the structured task-spec** (sidecar or int features) for success,
  filtering, paraphrase, analysis.
- **Record at ~10–30 Hz** (downsample from 500 Hz), action = **EE-delta chunks**
  via `delta_timestamps`.
- **Coverage + paraphrase** are the two data-quality contracts for real language
  conditioning.
