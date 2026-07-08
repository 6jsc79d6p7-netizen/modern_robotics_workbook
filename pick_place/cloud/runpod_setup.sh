#!/usr/bin/env bash
# RunPod: fresh PyTorch pod  ->  ready to train the pick-place policies.
#
# The pod only TRAINS: the dataset comes from the HF Hub (push it first from your
# Mac: `python -m pick_place.cloud.push_dataset --repo-id <you>/pick_place`) and
# `lerobot-train` is a CLI — no repo code needed here. Do inference back on your Mac.
#
# On the pod (web terminal or SSH):
#   export HF_TOKEN=hf_xxx                 # WRITE access (pull dataset + push model)
#   export REPO_ID=<your-hf-username>/pick_place
#   export OUT_REPO=<your-hf-username>/pick_place-dit   # where to push the trained policy
#   bash runpod_setup.sh                   # setup only, prints train commands
#   bash runpod_setup.sh dit              # setup + train multi_task_dit
#   bash runpod_setup.sh smolvla          # setup + finetune smolvla_base
# GPU->BATCH:  4090/24GB ~32   |   A100 40/80GB ~64   (set BATCH=... to override)
set -euo pipefail

MODE="${1:-setup}"
BATCH="${BATCH:-32}"
STEPS="${STEPS:-20000}"
OUT="${OUT:-/workspace/out}"                 # put /workspace on a persistent Volume
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

VE="${VE:-/opt/lr}"   # py3.12 venv (RunPod's image is py3.11; lerobot 0.6.0 needs 3.12)

echo "[1/3] ffmpeg + curl ..."
apt-get update -qq && apt-get install -y -qq ffmpeg git curl >/dev/null

echo "[2/3] python 3.12 + lerobot 0.6.0 via uv (0.5+/multi_task_dit require py3.12) ..."
# Pin 0.6.0: newer PyPI lerobot dropped multi_task_dit (ships groot/xvla/sarm).
# uv adds a standalone py3.12 + fresh CUDA torch, so any RunPod image works.
command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh
uv venv --python 3.12 "$VE"
uv pip install --python "$VE/bin/python" 'lerobot[dataset,training,multi_task_dit,smolvla]==0.6.0'
"$VE/bin/python" - <<'PY'
import torch
assert torch.cuda.is_available(), "CUDA not visible — check the pod's driver!"
print("torch", torch.__version__, "| GPU:", torch.cuda.get_device_name(0),
      f"| {torch.cuda.get_device_properties(0).total_memory/1e9:.0f} GB")
PY

echo "[3/3] HF auth ..."
# huggingface_hub reads $HF_TOKEN directly (no `hf login` needed) for the private
# dataset pull and the model push. export it before running this script.
if [ -n "${HF_TOKEN:-}" ]; then
  export HF_TOKEN
  echo "  using HF_TOKEN from env."
else
  echo "  WARNING: HF_TOKEN not set — private dataset pull / model push will fail."
  echo "  export HF_TOKEN=hf_xxx  (WRITE access) and re-run."
fi

REPO_ID="${REPO_ID:-<your-hf-username>/pick_place}"
COMMON="--dataset.repo_id=$REPO_ID --policy.device=cuda \
 --batch_size=$BATCH --steps=$STEPS --num_workers=8 --log_freq=100 --wandb.enable=false"
# push the trained policy to the Hub (per-checkpoint + at end) so you just pull it home.
PUSH() { echo "--policy.push_to_hub=true --policy.repo_id=$1 --save_checkpoint_to_hub=true"; }

train_dit() {
  "$VE/bin/lerobot-train" $COMMON $(PUSH "${OUT_REPO:-$REPO_ID-dit}") \
    --policy.type=multi_task_dit --policy.objective=flow_matching \
    --save_freq=$((STEPS/5)) --output_dir="$OUT/dit"
}
# smolvla_base finetune needs ~A100 40GB+. LR 5e-5 is conservative for the
# cross-embodiment finetune; the reference recipe is 1e-4 @ batch 64.
train_smolvla() {
  "$VE/bin/lerobot-train" $COMMON $(PUSH "${OUT_REPO:-$REPO_ID-smolvla}") \
    --policy.path=lerobot/smolvla_base --policy.optimizer_lr=5e-5 \
    --save_freq=$((STEPS/5)) --output_dir="$OUT/smolvla"
}
# Diffusion Policy (vision+state, NO language — grounding is in the mask). ~278M
# params (heavier than ACT); on a 4090/24GB start BATCH=64, drop to 32 if OOM.
# horizon/n_action_steps match the 15 Hz masked data; DDIM+10 keeps deploy fast.
train_diffusion() {
  "$VE/bin/lerobot-train" $COMMON $(PUSH "${OUT_REPO:-$REPO_ID-diffusion}") \
    --policy.type=diffusion --policy.horizon=32 --policy.n_action_steps=8 \
    --policy.noise_scheduler_type=DDIM --policy.num_inference_steps=10 \
    --save_freq=$((STEPS/5)) --output_dir="$OUT/diffusion"
}

case "$MODE" in
  dit)       echo "=> training multi_task_dit (batch $BATCH, $STEPS steps)"; train_dit ;;
  smolvla)   echo "=> finetuning smolvla_base (batch $BATCH, $STEPS steps)"; train_smolvla ;;
  diffusion) echo "=> training diffusion policy (batch $BATCH, $STEPS steps)"; train_diffusion ;;
  *)
    cat <<EOF

Setup complete. Now (edit REPO_ID if not exported):

  REPO_ID=$REPO_ID BATCH=$BATCH bash runpod_setup.sh diffusion  # Diffusion Policy
  REPO_ID=$REPO_ID BATCH=$BATCH bash runpod_setup.sh dit        # multi_task_dit
  REPO_ID=$REPO_ID BATCH=64  bash runpod_setup.sh smolvla      # smolvla_base (A100)

The trained policy is auto-pushed to the Hub (${OUT_REPO:-$REPO_ID-<policy>}),
so just pull it home:  mjpython -m pick_place.run_infer --viz live \\
  --checkpoint ${OUT_REPO:-$REPO_ID-dit}
(run_infer / from_pretrained accept a Hub repo id directly.) Local copies are also
in $OUT/... if you kept /workspace on a Volume.
EOF
    ;;
esac
