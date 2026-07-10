# LLM Planner + MCP-Orchestrated Skill Models — design note (PARKED)

**Status: PARKED / not implemented.** Priority is training a *working ACT model*
for the single pick-place task first. This note captures the architecture and its
nuances so we can resume without re-deriving. Nothing here is built yet — the
current `pick_place/` code is a single skill (policy + sim eval harness), not yet
a skill exposed to a planner.

## The idea

Instead of one monolithic multi-task policy (or a single big VLA), use a
**hierarchy**:

- **High level — a multimodal LLM/VLM orchestrator.** Takes the natural-language
  task + scene, decomposes it into subtasks, decides *which skill* to call and
  *with what parameters*, and re-plans on failure.
- **Low level — a library of specialized skill policies** (one ACT/DP per skill:
  `pick_place`, `wipe`, `open_drawer`, …), each an independently trained expert.
- **Glue — MCP.** Each skill (and each perception module) is exposed as a tool the
  LLM can call. Uniform `detect / ground → act → report` interface.

Lineage (not fringe): SayCan → Code as Policies → Inner Monologue → today's
hierarchical VLAs (Figure **Helix** System-1/System-2; **π0** flow-matching action
head). MCP is just the modern, clean transport.

## Why this is attractive for *our* situation

It quietly resolves two problems that bit the monolithic approach:

- **Catastrophic forgetting (the "finetune over and over" worry) → gone.** A new
  task is a *new skill model*, trained independently. Improving `pick` never
  disturbs `wipe`. No replay/co-training needed.
- **Multiple tasks (dishes + table) → composition, not one giant net.** New tasks
  are new *sequences* of existing skills — often zero retraining.

Other wins: right tool per timescale (LLM = long-horizon reasoning; ACT/DP =
50 Hz motor control); interpretability (see which skill was chosen, localize the
failure); perception can also be MCP tools (grasp detector, VLM `detect`).

## The hard parts (MCP is the *easy* part)

1. **Grounding at the boundary is the real problem.** MCP is schema + transport.
   The difficulty is turning language + pixels into skill parameters (which mug?
   what pose/mask?). The perception problem doesn't vanish — it moves to the
   LLM↔skill seam.
2. **Keep the LLM out of the control loop.** LLM calls are 100s ms–seconds: fine
   at *skill-transition* frequency (System 2), fatal in the tight loop. Skills must
   run their whole segment autonomously (System 1).
3. **Reliability compounds.** Long-horizon success = product of per-skill rates
   (5 skills @ 90% = 59%). The orchestrator only pays off if skills **report
   success/failure** and the LLM **re-plans** on failure.
4. **Hand-off states.** Skill B assumes a starting distribution; if A leaves the
   robot somewhere B never trained on, B fails. Need overlapping initial-state
   coverage or explicit resets. This seam is where these systems break in practice.

## Grounding: two *separable* axes

Don't conflate representation with update rate.

- **Representation: mask vs coordinate.**
  - *Mask* — highlight target pixels. Strong, direct signal; no camera calibration
    (all pixel-space); best for occlusion + grasp precision. But heavy.
  - *Coordinate* — target xy in the state. Clean symbolic MCP interface
    (`pick_place(obj_xy, bin_xy)`); needs **camera→robot calibration** (hand-eye:
    pixel → ray via intrinsics → 3D via depth/table-plane → base frame via
    extrinsics). Weaker grasp signal → **lean on the wrist cam** for precision.
    Train with **coordinate noise** injected to match noisy real detections.
- **Update rate: one-shot vs continuous.**
  - The **object moves** (arm picks it up, gripper occludes it), so a mask can't be
    one-shot — it needs a **per-frame tracker** through motion + occlusion.
  - A **coordinate** only needs the object *at rest* (to decide what to grab) + the
    (static) bin; after grasp, location is implicit in the gripper + wrist cam.

Key realization: **continuous ≠ mask.** A *continuously-tracked coordinate*
(tracker emits updated xy per frame) captures most of the recovery benefit with a
lighter, numeric interface. The mask's extra value is occlusion-robustness +
grasp precision, **not** slip-recovery per se.

Sweet spot: **tracked coordinate + wrist cam**, mask reserved for when occlusion /
grasp precision demand it.

## Recovery: two homes for it

When the object **slips out mid-carry** (our `held_cleanly = False` case):

- **In-skill (System 1, reactive):** policy sees the slip via continuous grounding
  and re-grasps in the same call. Fast/seamless, but needs continuous perception
  **and** slip-recovery episodes in training (note: we currently *filter these
  out* — reactive recovery would need the opposite data on purpose).
- **At the orchestrator (System 2):** skill *detects* the loss, reports failure,
  LLM re-plans (`locate` again → retry). Coarser/slower but skill stays simple and
  the LLM can *reason* ("rolled off the table → ask for help").

Crucial split: **failure *detection* is cheap & grounding-agnostic** (gripper /
`held_cleanly` signal); only **re-localization** needs re-grounding. For a
mostly-static, short pick-place, orchestrator-level abort→re-locate→retry is
plenty; reserve in-skill reactivity for genuinely dynamic tasks (handovers,
conveyors).

Rule of thumb: **dynamic env / long horizon → continuous grounding + in-skill
reactivity; static / short skills → one-shot-or-tracked-coord + orchestrator
retry.**

## Interface style: discrete (MCP) vs continuous (latent)

- **Discrete (our MCP tool-call plan):** symbolic, typed params. Maximally
  modular, interpretable, testable in isolation. Weakness: symbolic bottleneck —
  LLM must fully specify intent in words/params; fine reactive nuance is lost.
- **Continuous (Helix-style):** VLM emits an embedding conditioning the low-level
  policy. More reactive, handles ambiguity better; loses modularity/interpretability.

For us (hobbyist, sim→SO-100, want debuggability): **start discrete/MCP.** Move a
hot path to a latent interface later only if the symbolic bottleneck bites.

## Worked example: `pick_place` as a skill

Target shape of the skill once we resume:

```
locate(object_description) -> (obj_xy, bin_xy)          # perception MCP tool (VLM + calib)
pick_place(obj_xy, bin_xy) -> SkillResult(status, ...)  # ACT policy wrapped
    status ∈ {success, grasp_lost, timeout}
```

What the current code already gives us (see `pick_place/run_infer.py`,
`env.py`, `recorder.py`):

- **The policy has NO 'done' signal** (`run_infer.py` docstring): it emits
  EE-deltas forever, never says complete/failed.
- **The only success signal today is privileged** `env.success()` (sim ground
  truth); failure is inferred only by **timeout**. Neither is deployable as-is.

To make it a real skill (no retraining):

- **`success`** — replace privileged `env.success()` with a **deployable verifier**
  (VLM "is the object in the target bin?" / bin sensor). Can itself be an MCP
  perception tool.
- **`grasp_lost`** — FREE and non-privileged: `gripper_width` vs `gripper_cmd`
  (both already in the 18-d state). Commanded-closed but width collapsed ⇒ lost the
  object. This is the **deploy-time twin of the `held_cleanly` training gate** —
  the same gripper signals do double duty (data curation *and* runtime failure
  detection). Fires the instant it slips; no waiting for timeout.
- **Termination lives in the wrapper, not the model.** Wrap `rollout` into
  `run_skill(...) -> SkillResult`: stop on verifier success / grasp-lost monitor /
  timeout. Model stays a pure motor controller. (Optional later: add a "done"
  prediction head + labels if we want the model itself to signal completion.)

## How our current design already anticipates this

- **The mask/highlight grounding = the LLM↔skill interface.** On hardware the
  target highlight would come from a VLM (Grounding-DINO/SAM); that's exactly the
  pixel-level hand-off from planner to motor skill.
- **`held_cleanly` / gripper signals = the runtime failure detector** the
  orchestrator needs to re-plan.
- **Success detection = the orchestrator's feedback signal.**

So the pick-place skill is already close to a well-formed MCP tool: a grounding
input and (once wrapped) an outcome output.

## Deferred TODOs (when we resume)

1. Decide grounding representation for the real skill: **tracked coordinate +
   wrist cam** (recommended) vs mask+tracker. Affects the dataset (coords route
   needs re-collection with coords-in-state + clean scene cam + coord noise).
2. Refactor `rollout` → `run_skill(...) -> SkillResult(status, reason, final_pose)`
   with a pluggable success verifier (privileged in sim, VLM/sensor slot for
   hardware) and a `grasp_lost` gripper monitor.
3. Sketch the `locate` / `pick_place` MCP tool schemas.
4. Hand-off / initial-state coverage between skills; orchestrator re-plan loop.
5. Camera→robot (hand-eye) calibration for the coordinate route on SO-100.
