"""Launch a RunPod GPU pod that trains a policy — driven from your machine.

⚠️  YOU run this; it spends money and needs YOUR RunPod API key. It cannot run
    from the assistant's sandbox. I could not test the RunPod calls — verify the
    GPU name / image tag against current RunPod docs (they change), and STOP the
    pod when training finishes or it keeps billing.

Setup:
    pip install runpod
    export RUNPOD_API_KEY=...          # runpod.io -> Settings -> API Keys
    export HF_TOKEN=hf_...             # WRITE access (pulls dataset + pushes model)

Run:
    .venv/bin/python -m pick_place.cloud.runpod_launch \
        --dataset-repo <you>/pick_place --policy dit \
        --gpu "NVIDIA GeForce RTX 4090" --batch 32

The trained policy is pushed to the Hub (`--out-repo`, default
`<dataset-repo>-<policy>`), both per-checkpoint and at end of training — so you
just pull it on your Mac; no fishing files off the pod. Watch logs in the console.
"""
import argparse
import base64
import os

# RunPod GPU type ids (verify current names in the console): e.g.
#   "NVIDIA GeForce RTX 4090"  (24GB, ~$0.4/h)  — DiT
#   "NVIDIA A100 80GB PCIe"    (80GB, ~$1.5/h)  — smolvla / pi0
DEFAULT_IMAGE = "runpod/pytorch:2.4.0-py3.11-cuda12.4.1-devel-ubuntu22.04"
# Pin: multi_task_dit exists in 0.6.0 but was dropped in newer PyPI lerobot
# (which lists groot/xvla/sarm instead). Must match the local training version.
LEROBOT = "lerobot[dataset,training,multi_task_dit,smolvla]==0.6.0"

POLICY_ARGS = {
    "dit": "--policy.type=multi_task_dit --policy.objective=flow_matching --batch_size={batch}",
    "smolvla": "--policy.path=lerobot/smolvla_base --policy.optimizer_lr=5e-5 --batch_size={batch}",
    # Diffusion Policy: vision+state, no language (grounding is in the mask). ~278M
    # params; on a 4090/24GB use --batch 64 (drop to 32 if OOM). horizon/n_action_steps
    # match the 15 Hz masked data; DDIM+10 inference steps keeps deploy responsive.
    "diffusion": ("--policy.type=diffusion --policy.horizon=32 --policy.n_action_steps=8 "
                  "--policy.noise_scheduler_type=DDIM --policy.num_inference_steps=10 "
                  "--batch_size={batch}"),
}


def build_script(args):
    """The bash the pod runs on boot. Quotes/brackets are fine here — it gets
    base64-encoded before it touches RunPod's (unescaped) GraphQL dockerArgs."""
    train = POLICY_ARGS[args.policy].format(batch=args.batch)
    out_repo = args.out_repo or f"{args.dataset_repo}-{args.policy}"
    ve = "/opt/lr"  # RunPod's pytorch image is py3.11; lerobot 0.6.0 needs >=3.12.
    # uv gives us a standalone py3.12 + a fresh CUDA torch (PyPI default), so we
    # stay on the known-good deploying image instead of chasing a py3.12 tag.
    # $HF_TOKEN is read from the pod env at runtime (passed via create_pod(env=...)).
    # push_to_hub -> weights + pre/post processors at end; save_checkpoint_to_hub ->
    # every checkpoint as insurance (pod is ephemeral). Both target policy.repo_id.
    return "\n".join([
        "set -e",
        "apt-get update -qq && apt-get install -y -qq ffmpeg curl",
        "curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh",
        f"uv venv --python 3.12 {ve}",
        f"uv pip install --python {ve}/bin/python '{LEROBOT}'",
        # No `hf login`: huggingface_hub reads $HF_TOKEN (set via create_pod env)
        # automatically for the private dataset pull and the model push. Avoids the
        # huggingface-cli -> hf CLI churn that aborted the boot under `set -e`.
        "export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True",
        (f"{ve}/bin/lerobot-train --dataset.repo_id={args.dataset_repo} --policy.device=cuda "
         f"{train} --steps={args.steps} --save_freq={args.steps // 5} "
         "--num_workers=8 --log_freq=100 --wandb.enable=false "
         f"--policy.push_to_hub=true --policy.repo_id={out_repo} "
         "--save_checkpoint_to_hub=true --output_dir=/workspace/out"),
        f"echo '=== TRAINING DONE - policy pushed to {out_repo}; stop this pod ==='",
    ])


def build_command(args):
    # RunPod's SDK interpolates dockerArgs into GraphQL WITHOUT escaping
    # (mutations/pods.py: f'dockerArgs: "{docker_args}"'), so any " or [ in the
    # command breaks the query. base64 keeps dockerArgs to [A-Za-z0-9+/=] + pipes.
    b64 = base64.b64encode(build_script(args).encode()).decode()
    return f"bash -lc 'echo {b64} | base64 -d | bash'"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset-repo", required=True, help="<you>/pick_place on the Hub")
    ap.add_argument("--out-repo", default=None,
                    help="Hub repo to push the trained policy to (default <dataset-repo>-<policy>)")
    ap.add_argument("--policy", choices=list(POLICY_ARGS), default="dit")
    ap.add_argument("--gpu", default="NVIDIA GeForce RTX 4090", help="RunPod GPU type id")
    ap.add_argument("--cloud-type", choices=["COMMUNITY", "SECURE", "ALL"], default="COMMUNITY",
                    help="COMMUNITY has the most consumer-GPU (4090) capacity; SECURE is pricier/steadier")
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--steps", type=int, default=20000)
    ap.add_argument("--volume-gb", type=int, default=20)
    ap.add_argument("--container-disk-gb", type=int, default=30,
                    help="lower this if 'machine does not have the resources' — widens machine options")
    ap.add_argument("--list-gpus", action="store_true",
                    help="print deployable GPU ids + availability and exit")
    args = ap.parse_args()

    import runpod
    runpod.api_key = os.environ["RUNPOD_API_KEY"]

    if args.list_gpus:
        # get_gpus() is only the catalog; get_gpu(id) adds cloud + live on-demand
        # price (a non-null price ~= deployable now). Show >=16GB GPUs, cheapest first.
        rows = []
        for g in runpod.get_gpus():
            mem = g.get("memoryInGb") or 0
            if mem < 16:
                continue
            try:
                d = runpod.get_gpu(g["id"])
            except Exception:
                d = {}
            od = (d.get("lowestPrice") or {}).get("uninterruptablePrice")
            rows.append((od if od is not None else 1e9, g["id"], mem,
                         d.get("communityCloud"), d.get("secureCloud"), od))
        for _, gid, mem, comm, sec, od in sorted(rows):
            price = f"${od}/h" if od is not None else "no on-demand stock"
            print(f"{gid:34} {mem:>3}GB  community={comm}  secure={sec}  {price}")
        return

    pod = runpod.create_pod(
        name=f"pickplace-{args.policy}",
        image_name=args.image,
        gpu_type_id=args.gpu,
        cloud_type=args.cloud_type,
        gpu_count=1,
        volume_in_gb=args.volume_gb,
        container_disk_in_gb=args.container_disk_gb,
        volume_mount_path="/workspace",
        env={"HF_TOKEN": os.environ["HF_TOKEN"]},
        docker_args=build_command(args),
        ports="8888/http,22/tcp",
    )
    out_repo = args.out_repo or f"{args.dataset_repo}-{args.policy}"
    print(f"launched pod {pod.get('id')} ({args.policy} on {args.gpu}).")
    print(f"Watch logs in the RunPod console. Policy -> https://huggingface.co/{out_repo}")
    print("⚠️  STOP the pod when the log says TRAINING DONE, or it keeps billing.")


if __name__ == "__main__":
    main()
