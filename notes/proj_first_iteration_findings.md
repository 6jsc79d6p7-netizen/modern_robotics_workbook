# Pick-Place — First-Iteration Findings (food for thought)

*Written 2026-07-11, after the first real train→rollout→diagnose loop on the
`pick_place_masked_v2` dataset (DP / ACT / DiT-flow policies). Not conclusions —
a snapshot of what the evidence is pointing at, and what to change next.*

## TL;DR

The task is **not hard for the methods** — single-object tabletop pick-and-place
in sim is RoboMimic/LIBERO-class, which DP/ACT/flow routinely solve to 90–95%.
Our best (~60%) is a **pipeline gap, not a difficulty ceiling.** The evidence
increasingly points at the **data generator (the scripted expert)** as the
bottleneck, not the model or the control loop.

## What the three-model spread actually told us

Same data, three architectures:

| Policy | Behaviour | Read |
|---|---|---|
| **DP** (CNN-UNet diffusion, mask, no language) | accurate-ish (~60%) but slow | best framing of the task; 256M UNet is oversized |
| **DiT flow** (CLIP + transformer, language) | behaviourally right, ~5/5 live once given budget | under-fit (36k<60k), not weak — *misjudged at first* |
| **ACT** (deterministic regression) | inaccurate | **the tell** |

The **ranking is diagnostic**. ACT regresses to one action per state; DP/flow
model a *distribution*. When the deterministic one breaks while the generative
ones look right, that's the signature of **multimodal / inconsistent
demonstrations** — the same state maps to conflicting actions in the data, so
ACT averages the modes into mush. This isn't three unrelated failures; it's one
data property showing up three ways.

## The measurement traps we fell into (fix these first)

1. **Decision budget was strangling everything.** These policies are slow to
   finish (successes land at decisions ~200–370). With `--max-decisions 160`
   they *timed out mid-approach*. Bumping to 400 took DP from 1/15 → 6/15, and
   the DiT from an apparent "1/10" to 5/5 live. Several "weak model" verdicts
   were just a tight cap.
2. **The sampler is stochastic and was unseeded.** `--seed` fixes only the
   *scene* (`np.random.default_rng`); diffusion/flow draw fresh `torch.randn`
   noise every chunk, so *the same experiment gives different numbers.* At
   p≈0.4, n=15, one SE ≈ ±2 episodes — so 5/15 vs 6/15 is **noise.** Most of our
   small single-run gaps (fp16 vs fp32, pipeline vs sync) were under-evidenced.

**Eval improvements to make before optimizing anything:**
- **Seed the sampler** per episode (`torch.manual_seed(seed+ep)` + `torch.mps.manual_seed`).
  Makes runs reproducible *and* makes A/B comparisons genuinely paired (shared
  noise cancels → a difference is causal at small n).
- **Report k/n over ≥30 episodes with a rough CI**, not single 10–15 ep counts.
- **Generous `--max-decisions`** so slow-but-correct runs aren't scored as fails.
- **GT-replay@N** — feed the dataset's own expert actions through the deploy
  loop. It bounds what *any* model can achieve. (Ours fails not because the loop
  is broken but because the **script itself runs out of bounds** — see below.)
- **Single-episode overfit test** — train on 1–3 episodes, eval on those exact
  scenes. Can't reproduce its own demos → representation/action-space/loop bug.
  Can → the gap is data coverage/quantity. Cleanly separates "debug model" from
  "debug data" in ~20 min.

## The likely root cause: the scripted expert

Two concrete, self-inflicted data problems (both in our control):

- **Too fast in the key areas.** The script blitzes the grasp/place contact
  zones — so the policy learns *not to slow down where precision matters* →
  overshoots the grasp → lands off-distribution → dithers to recover. That is
  the entire 200–370-step cascade in one sentence, and it's a data-generation
  velocity bug, not a model flaw. Fix: **velocity-profile the expert**
  (ease-in/ease-out, hard speed cap in the approach/contact phase, scale speed
  by distance-to-target).
- **Out-of-bounds trajectories.** GT-replay reveals the script sometimes drives
  OOB. Those bad trajectories are *also in the training set*. Fix: workspace
  bounds / collision-aware waypoints so the oracle never fails, then re-audit.

And a mixing problem: **script (fast) + teleop DAgger (slow, some episodes too
long) = conflicting velocity styles for similar states** → exactly the
multimodality that punishes ACT. A script's value is *consistency*; teleop's is
*recovery coverage*. Mixed carelessly you lose both. Curate: a clean, slowed,
consistent script backbone + tight, targeted teleop recovery only.

## Action space: the embodiment-gap question

We avoided absolute / base-frame EE targets to "reduce the embodiment gap." Half
true, worth revisiting:
- The gap argument is real **for cross-embodiment** (RT-X / Open-X / VLAs use
  relative EE deltas because absolute poses are tied to a robot's workspace). We
  have *one fixed Franka* → no gap to close yet, so we're paying deltas'
  **integration drift** (the whole reference-vs-measured saga) for transfer we
  aren't using.
- The legit embodiment-*independent* reason to like deltas is **translation
  invariance** (generalizes across randomized object positions). That one does
  apply.
- Counterweight: the Diffusion Policy paper found **absolute/position targets
  often *more precise*** (no drift) — directly relevant to our overshoot.
- Middle path: output a delta but apply it as an absolute target off the **last
  commanded pose** (not the laggy measured pose → 0/6; not a free-running
  reference → drift).

→ Once eval is trustworthy, **A/B absolute-in-base vs delta.** Stop treating "no
absolute EE" as a rule.

## Food for thought: no expert script? <50 teleop demos?

Most real tasks have **no scripted oracle** — you get human teleop, and not much
of it. Can diffusion/flow reach good results on **<50 teleop demos**?

**Yes, and this is well-trodden.** ALOHA/ACT hit strong real-robot results on
~50 demos; LeRobot users routinely train ACT/diffusion/SmolVLA on 20–50 teleop
episodes per task. The conditions that make it work are exactly what our
scripted data *violated*:
- **Consistency** — a human teleoperating with a steady, deliberate style gives
  cleaner action labels than a fast, inconsistent script. *Fewer, cleaner demos
  beat more, contradictory ones.* Our own results hint at this: 50 consistent
  teleop episodes might outperform 300 flawed scripted ones.
- **Good observation coverage** — wrist camera especially; it's what makes
  precise grasping learnable from few demos.
- **Tight scope** — single task. Multi-task + language conditioning (our setup)
  raises the demo count.
- **Pretrained priors** — the real unlock for low-data real-world: fine-tune a
  **pretrained VLA / visuomotor backbone** (SmolVLA, π0, etc.) on 20–50 demos.
  Transfer slashes the from-scratch data requirement by an order of magnitude.

The irony worth sitting with: our *scripted expert*, meant to be the data
advantage, may be the liability — because it optimized for quantity and
throughput over the human-like consistency small-data IL actually rewards. The
low-data regime isn't a limitation to escape; it's a forcing function for demo
quality.

## Next, in order

1. Eval hygiene: seed the sampler, N≥30 + CI, generous budget.
2. GT-replay@N + single-episode overfit → locate the bottleneck precisely.
3. Fix the script: velocity-profile the contact phase + workspace bounds; re-audit.
4. Curate script/teleop into one consistent style.
5. A/B absolute-in-base vs delta action space.
6. Only then: model/recipe tuning (EMA, longer DiT training, image aug, narrower UNet).

Model choice (DP vs flow) stops mattering once 1–5 are done — which is the sign. 
the bottleneck has moved off our pipeline and onto the frontier, where it belongs.

Generate 50 teleop episode dataset - create 4 from same examples on 2 axis - abs vs delta. Illumination vs Non Illumination (for VLA-JEPA or multistep DIT)
Try VLA-JEPA without illumination trick - unique architecture well suited for few shot training.