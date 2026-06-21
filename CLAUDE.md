# Modern Robotics — Learning Project

A guided, self-paced walkthrough of *Modern Robotics: Mechanics, Planning, and
Control* (Lynch & Park). The book PDF is `MR.pdf`. This is a **learning
exercise**, not a code dump — the goal is durable intuition that eventually
supports building real robot controllers in simulation (MuJoCo / Isaac) and then
on hardware.

## Who I'm working with

- **Goal:** build strong enough foundations to start building in MuJoCo / Isaac
  Gym, and eventually on real robots. Theory is in service of building.
- **Philosophy:** intuition-first. **Skip long derivations.** Don't skip the
  *theory itself* — the user wants a strong conceptual foundation, just reached
  through intuition, geometry, and worked examples rather than proofs.
- **Weak spot:** linear algebra. This is the #1 thing to be careful about.
  Whenever a concept leans on linear algebra (matrix multiplication, eigenvalues,
  rank, null space, change of basis, transforms, the matrix exponential, SVD,
  least squares), slow down and explain the LA *as it comes up*, geometrically.
  Never assume the LA is obvious.

## End goal & chapter prioritization (read this, teaching agents)

**North star:** deep-learning-based robotics — *intelligent machines that take a
natural-language task and execute it*. Two concrete target tasks: (1)
**language-conditioned pick-and-place** of arbitrary objects, and (2)
**semantic navigation** of a room. Path: MR foundations → MuJoCo/Isaac →
imitation learning / VLAs / RL → real hardware. MR is **Phase 0**: the geometric
and control *language* that every learned policy consumes (poses, frames,
transforms, action spaces) and the controller substrate it sits on top of.

**The user will cover all 13 chapters** — do not skip teaching any. But weight
depth/time by relevance to the north star:

- **Tier 1 — go deep, this is the foundation:**
  - **Ch 3 Rigid-Body Motions** (SO(3)/SE(3), twists, screws) — *the single most
    important chapter.* Every learned system represents object pose, camera
    extrinsics, gripper/world frames in SE(3). "Pick up the cup" = "cup pose in
    gripper frame." Highest leverage; spend the most care here.
  - **Ch 4 Forward Kinematics** — joint angles → EE pose; robot embodiment.
  - **Ch 5 Velocity Kinematics / Jacobian** — joint vel ↔ EE vel; policies often
    output EE deltas converted via Jacobian/IK. Heavy on the user's LA weak spot
    → slow pass.
  - **Ch 6 Inverse Kinematics** — Cartesian target → joint angles; the standard
    "policy → EE pose → IK → joint targets" stack.

- **Tier 2 — concepts yes, derivations no:**
  - **Ch 2 Configuration Space** — DOF, C-space, topology (light, foundational).
  - **Ch 11 Robot Control** — learned policies sit *on top of* a low-level
    controller; emphasize position vs velocity vs torque and especially
    **impedance/compliance** (contact-rich manipulation).
  - **Ch 8 Dynamics** — conceptual only; **skip Lagrangian derivations**.
    MuJoCo/Isaac *are* dynamics engines; RL trains against them. Know what the
    mass matrix / Coriolis / gravity terms mean and why contact is hard.
  - **Ch 13 Wheeled Mobile Robots** — *elevated:* the **wheeled mobile
    manipulator is the chosen target embodiment** (statically stable → effort
    goes to perception/nav/manipulation, not balance). It exercises almost the
    whole stack: SLAM, semantic mapping, navigation, the full transform tree,
    plus the arm. Teach this with more depth than a typical Tier-2 chapter.
    **Humanoids are explicitly deferred** (far-future; see `notes/00_roadmap.md`
    M4) — do not steer the user toward legged locomotion / whole-body control.

- **Tier 3 — skim, except where flagged below:**
  - **Ch 7 Closed Chains** — *special interest: the user finds delta robots
    fascinating and wants this taught properly, not skimmed.* Low priority for
    the DL path but high personal-motivation → teach it with full care when we
    reach it. Don't down-prioritize it the way the "Tier 3" label would suggest.
  - **Ch 9 Trajectory Generation, Ch 10 Motion Planning, Ch 12 Grasping** —
    *special handling:* don't do full guided-exercise passes. Instead **(1) get
    the gist of the theory from MR, (2) discuss the SOTA learned approaches that
    replace it, (3) implement small toy examples.** The value is the modern
    replacement, not the classical derivation. (Ch 9 → diffusion policy / ACT;
    Ch 10 → cuRobo / RRT* / learned planning + VLN for nav; Ch 12 → learned grasp
    detection like Contact-GraspNet / AnyGrasp.) See `notes/00_roadmap.md`.

**What MR does NOT cover but the north star needs** (flag forward when natural):
perception (camera→pose), the deep learning itself, sim-to-real, and the
imitation-learning / diffusion-policy / VLA / RL methods. See the user's
"SOTA beyond the book" reading thread (SLAM app in progress; learned
replacements for Ch 9/10/12).

## How we work through a topic

The repeatable per-topic workflow lives in the `/mr-topic` skill. In short:

1. **Theory note** — I write `notes/NN_topic.md`: intuition-first explanation,
   minimal/no derivations, pictures-in-words, the key formulas with *what each
   symbol means and why*, and a short "linear algebra you need here" aside.
2. **Discussion** — we talk it through. I check understanding, the user asks
   questions, we adjust before moving on.
3. **Guided exercises** — we work through a few of the book's own exercises
   (`§N.8`) by hand. I curate 2–4 good ones, then *tutor* the user through the
   calculations: set up the problem, give the method, let them attempt each
   step, and check their work — doing the heavy linear algebra *with* them, not
   handing over finished solutions. **This replaced from-scratch notebooks**,
   which were judged to add little; the user wants to *do* the math and build
   calculation fluency. Only fall back to code when a problem genuinely needs
   numerical/visual exploration.

Always do theory → discussion → exercises in that order. Don't jump to the
exercises until the user has engaged with the note.

> The earlier notebooks (`notebooks/03a*`, `03b*`) and the `mr/` helper package
> remain useful references and can still be extended, but new topics are
> practiced via guided exercises, not new notebooks.

### Split big chapters into lettered sub-topics
Dense chapters (3, 4, 7, 8...) should be broken into bite-sized lettered pieces
(`3a`, `3b`, `3c`, ...), each getting its own full theory → discussion → code
pass rather than one giant note/notebook. This worked well for Ch. 3
(`3a` rotations, `3b` rigid motions/twists, `3c` wrenches) — confirmed as the
right granularity. Default to this split for any chapter that covers more than
one genuinely distinct concept.

### Add figures to theory notes, not just code
When a note introduces a new matrix/object whose physical meaning isn't
obvious from symbols alone (e.g. "what does this rotation matrix *look like*
as two frames in space?"), generate a small matplotlib figure during the
theory step and embed it in the note (`notes/figures/NN_description.png`,
generator script alongside it). Don't wait for the notebook — a picture in the
theory note is often what makes a matrix click. The 90°-rotation frame diagram
and the `ω × p` cross-product diagram for 3a are good templates to follow.

### Capture discussion Q&A back into the note
After the discussion step, if the user asked clarifying questions that revealed
gaps or produced a useful generalization (e.g. "is `det=1` enough for a
rotation?", "what does `[ω_s]R = R[ω_b]` mean?"), add a short **FAQ section**
to the theory note summarizing the question and the resolved answer, with
pointers back to the relevant section. This makes the note self-contained for
spaced review later — see `notes/03a_rotations.md` §8 for the pattern.

## Writing guidelines (important)

- **Build intuition before formalism.** Lead with "what is this for / what
  does it physically mean," then the math.
- **Explain every new piece of linear algebra inline**, geometrically, the first
  time it appears. Assume it is not obvious.
- **Skip derivations** unless the user asks. State results, explain *why they're
  shaped the way they are*, and move on.
- Prefer concrete numbers and small worked examples over abstract symbols.
- Keep notation consistent with the book (e.g. `R` for rotation matrix, `T` for
  homogeneous transform, `[ω]` for the skew-symmetric / so(3) matrix, twists
  `V`, screws `S`, etc.). Define notation the first time it's used.
- In code: implement-from-scratch first for understanding, then it's fine to
  cross-check with the `modern_robotics` pip package.
- Connect concepts forward to **building things** (MuJoCo/Isaac/real robots)
  when there's a natural hook — that's the user's north star.

## Structure

```
modern_robotics/
├── MR.pdf                  # the textbook
├── CLAUDE.md               # this file
├── README.md               # progress tracker / index
├── notes/                  # NN_topic.md — theory & intuition, one per topic
│   └── figures/            # generated PNGs + gen_NN_*.py scripts for notes
├── notebooks/              # NN_topic.ipynb — code, from-scratch + viz
├── mr/                     # shared python helpers we build up as we go
├── .venv/                  # project virtualenv (see Environment notes)
└── .claude/skills/mr-topic # the per-topic workflow skill
```

Numbering `NN` follows the book's chapters (see README).

## Environment notes

- **Use the project virtualenv**: `.venv/` (created via
  `/opt/homebrew/bin/python3.12 -m venv .venv`, since system Python is
  Homebrew-managed and blocks global pip installs). It already has numpy,
  matplotlib, jupyter, and `modern_robotics` installed. Run scripts/notebooks
  with `.venv/bin/python` / `.venv/bin/jupyter`.
- The official `modern_robotics` pip package is available for cross-checking
  (don't lean on it for the learning parts).
- 3D visualization matters for intuition here — use it liberally
  (matplotlib 3D, and later MuJoCo).
