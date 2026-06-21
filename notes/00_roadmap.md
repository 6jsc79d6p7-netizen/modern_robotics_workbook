# 00 — Roadmap: from Modern Robotics to language-conditioned robots

This note connects the book (Phase 0) to the **north star**: deep-learning-based
robots that take a natural-language task and execute it. Two target tasks:
**language-conditioned pick-and-place** ("pick up the apple and put it in the
bin") and **semantic navigation** of a room.

MR is the *geometric + control language* every learned policy consumes (poses,
frames, action spaces) and the controller substrate it sits on. So the book is
not a detour — Steps 3 and 6 of the pick-place pipeline below are literally
Chapters 3, 4, 5, 6, 11.

---

## Chapter plan (full version in CLAUDE.md)

- **Tier 1 — go deep:** Ch 3 rigid-body motions (most important), 4 FK, 5 Jacobian, 6 IK.
- **Tier 2 — concepts, skip derivations:** Ch 2 C-space, 11 control (esp. impedance), 8 dynamics (no Lagrangian), 13 wheeled mobile robots.
- **Ch 7 closed chains — special interest** (delta robots): teach with full care.
- **Ch 9 / 10 / 12 — gist + SOTA + toy:** get the *gist* of the theory from MR,
  then **discuss the SOTA learned approaches**, then **implement small toy
  examples**. Don't do full guided-exercise passes on these; the value is the
  modern replacement, not the classical derivation.

---

## The pick-and-place pipeline (SOTA, modular view)

Reference decomposition of "pick up the apple and put it in the bin." Real
systems are hybrids of this and end-to-end VLAs.

1. **Language / task planning** — VLM/LLM → grounded subtask plan + referents
   ("apple", "bin"). *SayCan, Code-as-Policies, VoxPoser, GPT-4o/Gemini.*
2. **Open-vocab perception** — find them in the image. *Grounding-DINO / OWL-ViT
   → SAM2; CLIP.*
3. **Lift to 3D — pixels → pose** *(← Ch 3 SE(3))* — depth + back-project to
   point cloud; camera→robot **extrinsic transform `T ∈ SE(3)`**; optional 6-DoF
   pose (*FoundationPose / MegaPose*). This is where Ch 3 stops being abstract.
4. **Grasp synthesis** — point cloud → ranked 6-DoF grasp poses `T_grasp ∈ SE(3)`.
   *Contact-GraspNet / 6-DoF GraspNet / AnyGrasp.*
5. **Motion planning** *(Ch 2 C-space, Ch 10)* — collision-free path
   pre-grasp→grasp→lift→over-bin→release. *cuRobo (GPU) / OMPL-RRT\* / MπNets.*
6. **Kinematics & control** *(Ch 4 FK, 5 Jacobian, 6 IK, 11 control)* — IK or
   operational-space (Jacobian) to reach poses; **impedance/compliance control**
   for force-limited contact (don't crush the apple); gripper close w/ force fb.
7. **Closed-loop feedback & recovery** — re-perceive after grasp, visual
   servoing, retry. *(See "closed loop" note below — MR covers the control-level
   loop, not the perception-level loop.)*

**End-to-end alternative (VLA):** one model maps `image(s) + language → actions`
(*RT-2, OpenVLA, Octo, π0, Gemini Robotics*). Internalizes 1–7, but its **output
action space is still SE(3)/joint deltas through a low-level controller** — MR
*is* the action space.

---

## Milestone builds

Sequenced to interleave with the reading.

- **M0 (now):** phone **SLAM app** — get basic mapping working, then push toward
  *semantic* mapping (*ConceptGraphs / VLMaps*) so language queries resolve to
  metric goals. Bridges SLAM → semantic navigation.
- **M1 (after Ch 3–6):** **MuJoCo apple-in-bin sim** with the modular pipeline
  *scripted* — mask → point cloud → grasp pose → IK → impedance grasp → place.
  Goal: feel every MR concept as running code in one task.
- **M1.5 (after Ch 13):** **mobile manipulator** — arm on a wheeled base in
  MuJoCo: *navigate-to-table → pick apple → navigate-to-bin → drop*. Adds SLAM/
  localization (`map→base` is what M0's app estimates), Ch 13 base kinematics +
  nonholonomic constraints, semantic map → goal pose, Nav2-style global+local
  planning, and the full **transform tree** (Ch 3 payoff: "apple in gripper
  frame while the base moves"). Sequential nav-then-manipulate first; whole-body
  later.
- **M2:** swap the scripted policy for a **learned** one — imitation learning
  (Diffusion Policy / ACT) on demos collected in M1's sim.
- **M3:** a **VLA** (OpenVLA first — open + documented) conditioned on the
  language instruction.
- **M4 (frontier):** **humanoid loco-manipulation** — walk to apple, pick, walk
  to bin, place. *Different physics:* the robot is underactuated and can fall.
  Replaces Ch 13 wheeled kinematics with **legged locomotion + balance** (CoM/
  ZMP/capture point, whole-body QP control) — material **beyond MR**. But the
  **floating base is an SE(3) element (Ch 3)** and floating-base dynamics is the
  spatial-vector/twist machinery of Ch 3/8 taken further (Featherstone). SOTA =
  RL walking in Isaac + sim-to-real, upper-body VLA, emerging humanoid foundation
  models (GR00T, Helix). If humanoids become the real goal, **go deeper on Ch 3
  and Ch 8 (toward floating-base dynamics) and down-weight Ch 13.**

---

## Two notions of "closed loop" (important)

1. **Control-level feedback** — feedback on robot state (joint/EE/force).
   **MR covers this well:** Ch 11 (error dynamics, PD/PID, computed-torque,
   force & impedance control), Ch 6 (iterative/closed-loop IK), Ch 13 (mobile
   trajectory tracking). This is the foundation.
2. **Task/perception-level feedback** — re-perceive after a failed grasp, visual
   servoing, reactive replanning. **MR does *not* cover this**; it's the
   SOTA/learning side (visual servoing → Corke "Robotics, Vision & Control";
   reactive policies → diffusion/VLA literature).

---

## Beyond the book — supporting stack

- **Perception foundation models:** SAM2, CLIP, DINOv2, Grounding-DINO/OWL-ViT, Depth Anything.
- **VLA frontier:** RT-1/2, OpenVLA, Octo, π0, Gemini Robotics (read OpenVLA first).
- **Sim + RL:** MuJoCo, Isaac Lab/Sim, domain randomization, sim-to-real.
- **3D vision math:** intrinsics/extrinsics, multi-view geometry (light H&Z) — pairs with Ch 3.
- **ML prereqs:** transformers, probability, optimization, and **linear algebra**
  (the weak spot) — the same SE(3)/Jacobian/SVD fluency the learned methods demand.
