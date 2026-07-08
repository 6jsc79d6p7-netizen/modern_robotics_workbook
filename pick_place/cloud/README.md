# Cloud training (HF Hub + Colab)

Glue to train on a real GPU when MPS is too slow/small. The handoff is via the HF
Hub: push the dataset once, then any GPU box (Colab, RunPod, …) pulls it by
`repo_id`. (This also sidesteps our local offline-load workaround — cloud can
reach the Hub.)

## 1. Push the dataset (once, from your Mac)

```bash
.venv/bin/huggingface-cli login            # or: export HF_TOKEN=...
.venv/bin/python -m pick_place.cloud.push_dataset --repo-id <your-hf-username>/pick_place
```
Uploads the 3-camera LeRobot dataset (private by default) → `huggingface.co/datasets/<you>/pick_place`.

## 2. Train on Colab

Open **`train_dit_colab.ipynb`** in Colab (upload it, or File → Open):
1. Runtime → Change runtime type → **GPU** (T4 fine for the DiT; A100 for smolvla/pi0).
2. Edit `REPO_ID` in the config cell to the repo you just pushed.
3. Run the cells: installs deps + ffmpeg → HF login → mounts Drive (checkpoint
   persistence) → `lerobot-train` on `cuda`, batch 64.

To try other policies, change `--policy.type` / `--policy.objective` in the train
cell (smolvla needs an A100; add `--policy.optimizer_lr=1e-5` if finetuning
`smolvla_base` — its default LR diverges on small batches).

## 2b. Or train on RunPod (bigger GPU than Colab's free T4)

The free T4 (15 GB) caps the DiT at ~batch 8 — no better than local. For a real
step-up (batch 64, smolvla, π0) rent a bigger GPU:

1. **Create a pod**: RunPod → Deploy → GPU (**RTX 4090 24 GB ~$0.4/h** for the DiT;
   **A100 40/80 GB** for smolvla/π0) → **PyTorch** template → attach a **Volume
   mounted at `/workspace`** so checkpoints survive a stop.
2. **Connect** (web terminal or SSH), upload `runpod_setup.sh` (or paste it), then:
   ```bash
   export HF_TOKEN=hf_xxx
   export REPO_ID=<your-hf-username>/pick_place
   bash runpod_setup.sh dit          # multi_task_dit  (or: smolvla ; or no arg = setup only)
   BATCH=64 bash runpod_setup.sh smolvla   # smolvla_base finetune on an A100
   ```
   Installs deps + ffmpeg, verifies CUDA, logs into HF, and trains. `BATCH`/`STEPS`
   are overridable env vars.
3. **Get the checkpoint back**: push to the Hub, or `runpodctl send
   /workspace/out/dit/checkpoints/last` — then run inference on your Mac.

Per-run cost is a few dollars (4090 ~$0.4/h, A100 ~$1.2–1.9/h).

## 3. Inference back on your Mac

Download the checkpoint (from Drive, or push the policy to the Hub and pull), then:
```bash
.venv/bin/mjpython -m pick_place.run_infer --viz live --checkpoint <ckpt>/pretrained_model
```
The MuJoCo sim/viz runs locally; only training goes to the cloud.

## Notes
- **Colab caveats**: sessions disconnect (~12 h free / idle 90 min) and storage is
  ephemeral → mount Drive (the notebook does) and use `lerobot-train`'s resume to
  continue after a drop.
- **This DiT is data-limited** (plateaued locally at ~300 demos) — cloud buys
  speed + headroom (bigger batch, scaling to more data / smolvla / pi0), not a
  free quality jump.
