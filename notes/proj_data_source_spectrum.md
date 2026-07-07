# Project note — The robot-learning data-source spectrum

> **What this is.** A strategy-level map of *where manipulation training data comes
> from* and the tradeoffs between sources — prompted by the wave of startups
> collecting **egocentric human data**, and how that relates to **our** teleop-in-sim
> data ([`proj_phone_teleop.md`](proj_phone_teleop.md)). Data is the real bottleneck
> in robot learning; knowing the spectrum tells you *why* our high-fidelity sim data
> is valuable and where it sits relative to the scale-chasing crowd.

---

## 1. The central tension: scale/cost vs the embodiment gap

Every manipulation dataset trades off two things:

- **Scale & diversity & cost** — how many demos, how varied, how cheaply?
- **Embodiment gap** — how far is the recorded "action" from the *robot's own*
  action space? The smaller the gap, the more directly the data trains a policy.

These pull against each other. The cheapest, most scalable data (humans doing
tasks with their hands) has the **biggest** embodiment gap; the most directly
usable data (teleoperating the actual robot) is the **most expensive** to collect.
Everything in the field is an attempt to get *both* — scale *and* small gap.

```
  pure human video → hand-tracked human → handheld robot-gripper → teleop ROBOT
   (Ego4D scale)        (gloves/MoCap)         (UMI / DexCap)        (ALOHA / ours)
  ◄──────────────────────  MORE SCALE, CHEAPER  ───────────────────
                           SMALLER EMBODIMENT GAP, MORE DIRECTLY USABLE  ──────►
```

| | **Teleop-robot** (ALOHA, DROID, **ours**) | **Egocentric human** (the startups) |
|---|---|---|
| recorded | robot obs + **robot action** | head/wrist video + **human hand** trajectory |
| embodiment gap | **none** — labels already robot-native | **large** — a hand is not a gripper |
| cost / scale | expensive, slow, robot+rig per demo | cheap, massively scalable, diverse |
| action label quality | perfect | must be **inferred / retargeted** |

---

## 2. The embodiment gap = two gaps

Human data can't be behavior-cloned onto a robot directly because of **two**
distinct mismatches:

1. **Action gap** — the "action" is a *human hand* moving, not robot joint/EE
   commands. The hand has far more DoF than a gripper, and lives in a different
   action space.
2. **Observation gap** — a head-cam egocentric view ≠ the robot's wrist/base
   camera, and it often shows a *human arm*, which the robot doesn't have. Visual
   domain mismatch.

Closing *both* gaps is what every human-data method is really engineering.

---

## 3. How human data is made usable (the bridging tricks)

- **SE(3) retargeting.** Track the human **wrist as a 6-DoF pose** → map to a
  **robot EE pose** (`SE(3) → SE(3)` — *the exact mapping in our phone-teleop note
  §4*), and **fingers → gripper open/close**. Wrist pose retargets cleanly (both
  are rigid poses); the many finger DoF are the lossy part. *Closes the action gap.*
- **Make the human use robot-like hardware (the cleverest fix → UMI, §5).** Hand
  the human a **handheld copy of the actual robot gripper with a wrist camera** —
  now observation and action are *already* in the robot's space. *Collapses both
  gaps at the source.*
- **Observation alignment.** Use **wrist cams** (human arm out of frame) instead of
  head cams; **inpaint/mask the human arm** from body-visible views (or *render the
  robot arm in*); train for viewpoint robustness. *Closes the observation gap.*
- **Division of labor (pretrain-then-ground).** Use *huge* human data to learn
  **visual representations, affordances, sub-goals** ("what's graspable, what the
  task looks like"), and a *small* slice of **robot teleop** to ground the actual
  **action head**. The dominant way weak-label, big-scale data (Ego4D) gets used.

---

## 4. A sub-spectrum *within* human data

"Human data" isn't one thing — it's a gradient by how close the label is to the
robot:

| tier | what's captured | action label | mainly used for |
|---|---|---|---|
| **pure egocentric video** (Ego4D) | head-cam video, no instrumentation | *none* (no robot-space action) | representation pretraining, affordances, world models |
| **+ hand tracking** (gloves / MoCap / vision) | video + human hand/wrist trajectory | retargetable hand poses | retargeted actions, co-training |
| **handheld robot-gripper** (UMI, DexCap) | wrist cam + gripper SE(3) pose + width | **near-robot-native** | directly trainable policies |

Weakest label + most scale on top; strongest label + less scale at the bottom.

---

## 5. UMI — the key reference (and why it mirrors our setup)

**UMI (Universal Manipulation Interface)** is the cleanest gap-closer, and it's
worth knowing in detail because **its data format is structurally identical to
ours**:

- A **handheld gripper** (a copy of the robot's gripper) with a **single
  wrist-mounted fisheye camera**. **No head cam.**
- The camera does **double duty**: it's the **observation** *and* — via **visual
  SLAM + IMU (VIO)** — the source of the **6-DoF gripper pose** (the action label).
  Gripper **width** is read from a little **mirror** that sees the fingers.
- The human arm is **behind the camera, out of frame** → **little/no inpainting
  needed** (a big reason wrist-mounting wins).
- Deploy: put the **same wrist cam + gripper** on the robot → observation gap
  closed; train **Diffusion Policy** on the data.

> **The UMI ↔ ours equivalence (the satisfying part):** both reduce to
> **`(camera observation, EE-pose action, gripper)`**. The *only* difference is
> *where* obs and pose come from:
> - **UMI:** one camera gives obs *and* (via SLAM) pose; you must physically match
>   the camera mount at deploy.
> - **Ours:** the **phone** gives the pose (VIO, retargeted), the **sim** gives the
>   cameras — both *known exactly*, so **no SLAM on our side** and the observation
>   gap is closed **for free** (we render from the robot's own cameras).
>
> So our **phone teleop is literally a retargeting interface** solving the same
> human→robot `SE(3)` problem UMI solves — phone tracker + sim gripper vs. handheld
> tracker + real gripper — using the same VIO. Note §4 *is* the bridge.

---

## 6. Multi-view & heterogeneous co-training (the frontier)

You can combine views — and people do (**ALOHA**: overhead + wrist cams). A policy
just runs one image encoder per view and fuses features. E.g. capture **UMI wrist
+ head cam (inpainted)** together:

- **wrist cam** → close-up manipulation detail, always on the action, + the pose;
- **head/scene cam** → wide context, sees objects before contact, search/approach.

Two real constraints to respect:

1. **Deploy-time camera correspondence.** *Every training view must have a
   counterpart at deploy*, in roughly the same arrangement, or the policy gets an
   input it never learned to read. Wrist→wrist is easy. A **head cam** needs a
   deploy twin: a **humanoid** has a head (natural); a **fixed-base arm** does not,
   so the "head cam" becomes a *fixed external cam* — a **different viewpoint
   distribution** (a human's head cam moves with egomotion; a bolted cam doesn't).
   So a moving head cam is most useful when deploy *also* has a moving head, or when
   used for **context/representation** over precise control, or with viewpoint-
   robustness training.
2. **Heterogeneous co-training + missing views.** Real generalist policies (**Octo,
   π0, OpenVLA** on **Open X-Embodiment**) train on *many* datasets with **different
   cameras and action spaces**, handling missing/variable views via **masking /
   dropout** and per-dataset embeddings. "Train with both" is the toy version of
   "train with everything, robust to whatever's missing."

---

## 7. Where OUR data sits — and its advantages

We're at the **gold-fidelity end**: by capturing observations and actions **inside
the robot's own sim**, our data is **robot-native with zero embodiment gap**. What
we *lack* vs the startups is **scale**. Specific sim advantages:

- **Render any cameras for free** (base, wrist, "head," multiple) — and **ablate
  which actually help**, a luxury human-data rigs pay dearly for.
- **Ground-truth pose & state** — no SLAM, no retargeting error; perfect labels.
- **Observation gap auto-closed** — obs are rendered from the robot's own cameras.
- **Privileged-state expert option** (scripted/planner teacher) for cheap volume
  alongside teleop quality.

The one discipline for later **sim-to-real**: sim cameras must mirror the real
robot's actual camera placement (the deploy-correspondence rule, applied to us).

> **The frontier (what π0-class systems do):** **co-train** massive *human-scale*
> data (breadth) **+** a smaller *robot-grounded* set (precise action mapping, like
> ours). The startups build the high-scale end; we build the high-fidelity end; the
> strongest systems **fuse both**.

---

## 8. Cheat-sheet

- **Two axes of every dataset:** scale/cost ⟂ embodiment gap; they trade off.
- **Embodiment gap = action gap + observation gap.** Close both.
- **Bridges:** SE(3) retargeting · handheld robot-gripper (UMI) · wrist-cam +
  arm-masking · pretrain-on-human / ground-on-robot.
- **UMI data ≡ our format:** `(camera, EE-pose, gripper)`; UMI derives pose from its
  camera (SLAM), we get it from the phone + render obs from sim.
- **Multi-view is normal**, but every view needs a **deploy twin**; moving head cam
  → viewpoint gap on a fixed base.
- **Frontier = heterogeneous co-training** (masking/dropout) of human-scale +
  robot-grounded data.
- **Our edge:** zero embodiment gap + free multi-view + GT pose; **our gap = scale.**
