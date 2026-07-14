# 12b — Learned grasping: from force-closure math to a network that eyeballs a grasp

> The **SOTA replacement** for Chapter 12's classical machinery. 12a gave the
> *criterion* (force closure, the friction cone, grasp quality = margin in wrench
> space). 12b is how modern robots **stop computing that criterion by hand** and
> instead let a network look at a depth image / point cloud of a *never-seen*
> object and directly emit ranked **6-DOF gripper poses `T_grasp ∈ SE(3)`**.
> Special-handling chapter (gist → SOTA → toy): this note is the SOTA + a toy
> antipodal-grasp detector you'll build by hand. This is **step 4 of the
> pick-place pipeline** in `../00_roadmap.md`.

---

## 1. The big picture — why the classical pipeline died for real robots

The classical recipe (12a) is: *know the object's geometry → place candidate
contacts → build the grasp map `G` → check force closure → maximize the
wrench-space margin.* It's mathematically clean and it works — **if you have a
CAD model of the object and its pose.**

Real manipulation almost never does. You get a **single depth image** of a
*novel* object in clutter — partial, noisy, no model, no known pose. Analytic
grasp planning falls apart here:

- You don't have the full geometry (depth sees one side only).
- Even with geometry, searching 6-DOF grasp space + friction-cone checks per
  candidate is slow.
- It says nothing about *reachability, collisions, or which grasps a real
  gripper physically achieves.*

So the field flipped the problem. Instead of *computing* whether a grasp is
force-closure, **learn a function** `depth/point-cloud → ranked grasp poses`.
The force-closure math from 12a didn't disappear — it moved into the **training
signal**: it's how millions of grasps get *labeled* good/bad, so the network
learns to output grasps that would pass the 12a test, on objects it has never
seen. The network is a fast, generalizing amortization of the 12a criterion.

**Tie to your build:** your Franka pick-place uses a parallel-jaw gripper. Every
system below outputs exactly what your stack needs at roadmap step 4: a gripper
pose `T_grasp ∈ SE(3)` (where to put the hand) + a width (how far to open),
which then feeds IK/impedance (steps 5–6) and the close-and-lift.

---

## 2. What a "grasp" *is* to these networks (the output representation)

For a parallel-jaw gripper, a grasp is a full **6-DOF pose in SE(3)** plus a
scalar:

- **position** `p ∈ ℝ³` — where the gripper's grasp center sits,
- **orientation** `R ∈ SO(3)` — built from three physical axes:
  - **approach** `a` — the direction the hand moves *in* along (usually roughly
    the wrist/palm axis),
  - **baseline / closing** `b` — the axis the two fingers close along (the line
    connecting the fingertips — this is the **antipodal line** from 12a!),
  - **binormal** `a × b` — completes the right-handed frame,
- **width** `w` — the finger opening at grasp (must fit the object + gripper max).

That's it. A grasp = *"put the gripper frame here, oriented like this, opened
this wide, then close."* This is a direct payoff of Ch 3: the network's job is
literally to regress an element of SE(3). Older top-down methods (Dex-Net v1)
cheated it down to **4-DOF** — a planar `(x, y, z, θ)` overhead grasp — which is
simpler but can't grasp a horizontally-lying mug by the rim. Modern methods are
full **6-DOF**, which is what clutter and arbitrary object poses demand.

---

## 3. The landscape — three ways to learn a grasp

There are three architectural families. Know the shape of each; you'll pick
Contact-GraspNet or AnyGrasp in practice.

### (a) Discriminative / sample-and-score — **Dex-Net + GQ-CNN** (2017, the bridge)
The historically important one because it shows the analytic→learned handoff:

1. **Generate** a huge synthetic dataset: sample objects, sample antipodal
   grasps, and **label each grasp's quality with the analytic 12a metric**
   (robust force-closure / wrench-space margin under perturbations). Dex-Net 2.0
   = 6.7M grasps.
2. **Train** a CNN (GQ-CNN) to predict that quality score from a depth patch.
3. **At runtime:** sample many candidate grasps, score each with the CNN, pick
   the argmax.

Great for **bin-picking** (top-down, 4-DOF). Limitation: you must *propose*
candidates, and it's overhead-grasp-shaped.

### (b) Generative — **6-DOF GraspNet** (2019, NVIDIA)
Instead of scoring given candidates, **sample grasps from a learned
distribution.** A **CVAE** (conditional variational autoencoder) maps the object
point cloud → a distribution over successful 6-DOF grasps; you draw diverse
samples, then a separate **evaluator** network refines/scores them (gradient
steps in grasp space). Handles full 6-DOF and diverse grasps, but the
sample-then-refine loop is heavier.

### (c) Contact-point regression — **Contact-GraspNet** (2021, NVIDIA) ← the workhorse
The clever reframing that made 6-DOF grasping fast and robust, and the one
`../00_roadmap.md` names for your pipeline. Key idea: **every observed 3D point is a
potential contact point**, so anchor grasps to the point cloud instead of
searching free space.

- Input: a scene **point cloud** (from depth). Backbone: PointNet++.
- For **each point**, the net predicts: is it a good contact? + the grasp's
  approach direction, the baseline direction, and the grasp width. Because one
  contact point + those directions fully determine the 6-DOF pose, the enormous
  6-DOF search collapses to a **per-point prediction** over points you actually
  observed.
- Output: a dense field of scored 6-DOF grasps blanketing the visible surface,
  in ~real time, generalizing to novel objects in clutter.

This is the sweet spot: 6-DOF, fast, point-cloud-native, plugs straight into a
segmented object mask.

### (d) Production-grade — **AnyGrasp** (2023) & **GraspNet-1Billion** (2020)
- **GraspNet-1Billion**: the field's **ImageNet moment** — a massive *real*
  benchmark (1.1B grasp annotations over real scenes) that made learned 6-DOF
  grasping reproducible and comparable. When people say "SOTA grasp detector,"
  they usually mean *trained/evaluated on this.*
- **AnyGrasp**: real-time, **dense**, **temporally consistent** 6-DOF grasping in
  heavy clutter, including *moving* objects and dynamic tracking. Effectively the
  commercial state of the art (from the GraspNet group). If you want an
  off-the-shelf grasp brain for a MuJoCo→real pick-place, this is the reference
  bar.

**One-line map:** Dex-Net = *analytic labels + score samples (4-DOF, bins)*;
6-DOF GraspNet = *generate + refine (6-DOF, CVAE)*; Contact-GraspNet = *regress a
grasp per observed point (6-DOF, fast)*; AnyGrasp/GraspNet-1B = *dataset + product
that made it all real-time and reproducible.*

---

## 4. The linear algebra you need here — how a network emits a rotation

This is the LA that bites in *every* learned method that outputs a pose (grasp
nets **and** your VLA action head), so it's worth internalizing. The question:
**how does a network output an orientation `R ∈ SO(3)`?**

Naively you'd have the net regress the 9 numbers of `R` directly. **This works
badly**, and the reason is geometric. Recall `SO(3)` (from 3a): valid rotation
matrices are a *curved 3-dimensional surface* sitting inside 9-dimensional
matrix space — the ones with **orthonormal columns and det +1**. A network
outputs arbitrary 9 numbers, which almost never land exactly on that surface, so
you must "snap" them back (via SVD / Gram–Schmidt). The trouble: the map from
"free numbers → nearest rotation" has **discontinuities** — near certain
rotations a tiny change in the target requires a huge jump in the raw output.
Networks hate discontinuous targets (same reason Euler angles and quaternions,
which have their own wraparound/double-cover jumps, train poorly).

**The fix everyone uses — the 6D continuous representation** (Zhou et al. 2019):
have the network output just **two 3-vectors** `a, b` (six numbers), then build a
clean rotation from them by Gram–Schmidt:

1. `r₁ = a / ‖a‖`               (normalize the first axis)
2. `r₂ = (b − (r₁·b) r₁)`,  then `r₂ = r₂/‖r₂‖`   (subtract off `r₁`'s
   component from `b` so it's ⟂ to `r₁`, then normalize)
3. `r₃ = r₁ × r₂`               (cross product completes the right-handed frame)
4. `R = [r₁ | r₂ | r₃]`

This map is **smooth and surjective onto SO(3)** — every rotation is reachable
and small target changes need only small output changes, so it trains cleanly.
Geometrically it's exactly the two-axes-and-cross-product construction of §2:
the grasp's **approach** and **baseline** directions *are* those two vectors, and
the binormal `a × b` is the completing cross product. Contact-GraspNet outputs
precisely this — approach + baseline per point — for exactly this reason. So the
"how does a net emit an SE(3) pose" answer is: **regress a translation + two
direction vectors, Gram–Schmidt them into `R`.** Remember this when you build the
ACT/diffusion action head — it's the same trick.

*(Vocabulary, plainly: **orthonormal** = unit-length and mutually perpendicular;
**Gram–Schmidt** = the "straighten the second vector to be ⟂ the first" procedure
above; **surjective onto SO(3)** = every rotation has a preimage, nothing is
unreachable.)*

---

## 5. How it plugs into *your* pick-place stack

Grasp detection is object-agnostic — it finds good grasps on *whatever geometry
it sees.* Language/perception decides **which** object; the grasp net decides
**where** on it. The modular flow (roadmap steps 2→6):

```
RGB-D  ──▶ open-vocab detector + SAM2  ──▶ mask of "the apple"
depth  ──▶ back-project to point cloud (Ch 3 extrinsics T_cam→base)
                    │
                    ▼
        Contact-GraspNet / AnyGrasp  ──▶ dense scored 6-DOF grasps
                    │  (keep only grasps whose contact points fall on the apple mask)
                    ▼
        filter: reachable? (IK exists) collision-free? width ok?
                    │
                    ▼
        pick argmax-score grasp  ──▶  T_grasp ∈ SE(3)
                    │
        Ch 6 IK / Ch 5 Jacobian  ──▶ joint targets for pre-grasp → grasp
        Ch 11 impedance control   ──▶ compliant approach + close w/ force limit
                    ▼
                 close, lift, place
```

Two things to notice:

- **The mask is the language hook.** "Pick up the apple" becomes: segment the
  apple → run the grasp net → *keep only grasps landing on the apple's points* →
  execute the best. The grasp net never needs to know what an apple is. This is
  the clean, debuggable seam a scripted M1 build uses.
- **12a lives in the filter.** Force closure / width / reachability filtering is
  the analytic 12a criterion acting as a safety net on the learned proposals.

### Where end-to-end VLAs differ
An OpenVLA / π0 / RT-2-style policy **skips explicit grasp detection entirely** —
it maps `image(s) + language → gripper action deltas` directly, and "where to
grasp" is *implicit* in the learned weights. Trade-off: VLAs are more flexible
and need no perception plumbing, but the grasp is a black box you can't inspect
or filter, and they're data-hungry. The modular grasp-net stack is more
debuggable and reuses off-the-shelf grasp brains; the VLA is the frontier. Your
roadmap deliberately does **modular first (M1), learned/VLA later (M2–M3)** — and
the modular grasp pose is *the same SE(3) target* the VLA is implicitly hitting,
so building the modular version teaches you what "good" looks like.

---

## 6. The toy we'll build (replaces the guided exercises)

Per the special-handling rule, instead of book exercises we implement a **small
toy** that makes the classical→learned bridge concrete: a **2D antipodal grasp
detector.**

- Take a simple object as a point cloud with surface normals (a blob outline).
- **Sample pairs of points**, and score each pair by the 12a force-closure test:
  the line joining them must lie **inside both friction cones** (equivalently:
  each point's inward normal is within angle `α = arctan μ` of the grasp line).
- Rank pairs, visualize the best few grasps and a rejected (glancing) one.

This is *exactly the analytic labeler* that Dex-Net used to generate its training
data — so by building it you're building the thing a grasp network learns to
imitate. Then we can discuss swapping the brute-force search for a learned
per-point predictor (the Contact-GraspNet reframing). We'll write it together
after you've read this.

---

## 7. Gotchas / intuition checks

- **The 12a math didn't vanish — it became the label.** Learned grasping is fast
  *amortized* force-closure: the wrench-space quality metric trains the net, then
  the net generalizes it to unseen objects in real time.
- **Grasp = SE(3) pose + width.** Nothing exotic — a Ch 3 frame (approach ×
  baseline × binormal) plus an opening. The "baseline" axis *is* 12a's antipodal
  line.
- **Nets don't regress raw rotation matrices.** They output translation + two
  direction vectors and Gram–Schmidt into `R` (6D representation) — because
  `SO(3)` is a curved surface and naive 9-number regression has discontinuities.
  Same trick you'll use in your VLA/ACT action head.
- **Contact-GraspNet's trick = one grasp per observed point.** Anchoring grasps
  to real point-cloud points collapses the 6-DOF search into a fast per-point
  prediction. That's why it's the practical default.
- **Perception picks *which*, grasp net picks *where*.** Language/segmentation
  selects the object; the (object-agnostic) grasp net finds the pose. The mask
  filter is your language hook.
- **VLA vs modular:** VLA hides the grasp inside learned weights (flexible, opaque,
  data-hungry); modular grasp-nets output an inspectable, filterable SE(3) pose
  (debuggable, reuses off-the-shelf models). Build modular first.
- **Dex-Net(4-DOF top-down) vs 6-DOF nets:** top-down is fine for bins of parts;
  arbitrary poses in clutter need full 6-DOF.

---

## FAQ
_(to be filled from discussion)_
