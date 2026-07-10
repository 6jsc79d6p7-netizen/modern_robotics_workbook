# The robot-learning iteration loop — methodology note

The core loop of imitation-learning robot development, distilled from the
pick-place debugging arc (empty-gripper contamination → clean data → ACT's first
rollout win). Written so future-me pulls the *right* lever fast instead of
whack-a-moling.

## The loop, sharpened

**Train → roll out → diagnose the failure *precisely* → pull the *smallest right*
lever → re-eval the whole suite → repeat.**

The naive version is "train → see it fail → add data → repeat." The trap is step
3: **"add data" is only one lever, and usually not the right one.** What the loop
actually hinges on is the **diagnosis** — it tells you which lever.

## Diagnosis is the skill (and the loss lies)

- **Rollout is ground truth. Training loss is not.** In this project the loss was
  low/flat while the policy was (a) broken by a deploy bug, (b) trained on
  poisoned data, and (c) merely undertrained. Never close the loop on loss —
  close it on rollouts.
- For a **delta / EE-action** policy specifically, low L1 loss is nearly
  meaningless: per-step deltas are tiny, so predicting ~0 already scores well.
- The average loss is dominated by the **easy majority** of frames (approach,
  transport). The **sparse, task-critical** frames (the grasp/rotation moment)
  barely move it — so behaviour keeps improving under a flat loss.

## The lever menu (diagnose which one, then pull it)

From this project — none of the first four is "collect more episodes":

| Failure mode | Right lever |
|---|---|
| Empty-gripper "placements" saved as success | Data **quality** — *remove* contaminated episodes (see [[pickplace-empty-gripper-audit]]) |
| "Every policy fails" | **Deploy-loop bug** — measured-vs-running-reference EE-delta integration ([[pickplace-deploy-loop-bug]]) |
| Places with an empty gripper | **Observation design** — add commanded+measured gripper to `observation.state` |
| Hardly moves early (0.19× delta scale at 10k) | **Nothing — just undertrained**, keep training |
| Reaches target, misses the grasp (Δrot stuck ~0.2) | **Training recipe** — phase-weight the sparse grasp/rotation frames |
| Off-distribution states at deploy | **Targeted data (DAgger)** — collect demos *from the failure states* |

Full menu: `{more data, better data, remove data, fix the deploy stack, change
the observation, change the training recipe, just train longer}`.

## Diagnostic instruments built in this repo

- **`pick_place/audit_dataset.py`** — data-quality gate. Flags empty-at-placement
  episodes from the gripper signal (`w@release < 0.010`), distinguishes genuine
  failures from teleop fumble-then-clean-placement, writes an exclude list.
- **`pick_place/replay_gt.py`** — deploy-loop check. Feeds ground-truth actions
  through the deploy integration to isolate "model broken" vs "deploy stack
  broken" (measured 0/6 vs reference 5/6 was the smoking gun).
- **Delta-magnitude probe** (currently ad-hoc; run inline) — load a checkpoint,
  run it on real dataset frames, compare `|Δpos|`/`|Δrot|` of prediction vs GT.
  **The single most useful signal for "which phase is undertrained."**
  Trajectory this run: Δpos 0.19→0.68→0.73 (saturates fast); Δrot 0.10→0.20→0.25
  (crawls — grasp/rotation is the bottleneck, a *sampling* problem not a
  step-count one). *TODO: promote to `pick_place/diag_action_scale.py`.*

Signals worth remembering: `gripper_width` vs `gripper_cmd` = grasp/hold state
(also the deploy-time `grasp_lost` detector); `w@release` = empty-vs-held at
placement; Δrot ratio = grasp-orientation readiness.

## Guardrails

- **DAgger** is the formal name for "roll out → collect corrective demos from the
  failure states → retrain." Teleop fumble-recover episodes were accidental
  DAgger data (kept on purpose).
- **Avoid whack-a-mole.** Fixing only the latest failure can regress earlier ones
  (distribution shift / forgetting). Keep a **fixed eval suite** and track
  **overall success rate every iteration**, not just "did the last bug vanish."
- **Smallest lever first.** Prefer a config/observation/recipe change or a data
  *removal* before a fresh collection — cheaper and less likely to introduce new
  distribution shift.

## How it scales

Same loop, different meaning of "augment against failure":
- **VLAs (π0 / OpenVLA):** finetune the generalist on the failure cases.
- **RL:** the policy generates its own data → "augment against failure" becomes
  reward shaping + more environment interaction (faster loop, harder to steer).
- **Frontier:** automating the *diagnosis* and *targeted collection* — the
  human-in-the-loop diagnose-and-collect step is the current bottleneck.

Bottom line: **the job is the "understand exactly why" step.** Nail the diagnosis
and the fix is usually small and cheap.
