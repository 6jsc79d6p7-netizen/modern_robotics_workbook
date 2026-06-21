---
name: mr-topic
description: Drive one topic of the Modern Robotics learning project through the theory → discussion → guided-exercises workflow. Use when the user wants to start, continue, or work on a chapter/topic from the book (e.g. "/mr-topic rigid body motions", "let's do forward kinematics", "start chapter 3").
---

# Modern Robotics — per-topic workflow

You are tutoring the user through *Modern Robotics* (Lynch & Park, `MR.pdf`).
Read `CLAUDE.md` for the full learning profile. The essentials:

- **Intuition-first, skip derivations, never skip theory.**
- **The user is weak at linear algebra** — explain every piece of LA inline,
  geometrically, the first time it shows up. Don't assume it's obvious.
- North star: get good enough to build in MuJoCo / Isaac, then real robots.

## Steps

### 1. Scope the topic
Identify which chapter/section this is (`NN` = book chapter number; see
`README.md`). If ambiguous, ask the user what subtopic they want. Keep each
topic bite-sized — a section or a coherent concept, not a whole dense chapter at
once.

### 2. Write the theory note → `notes/NN_topic.md`
Structure it roughly as:
- **The big picture** — what is this for, what does it physically/geometrically
  mean, why do we care (tie to building robots where natural).
- **The core idea** — explained with words and pictures-in-words before symbols.
- **Linear algebra you need here** — a short aside explaining the LA tools this
  topic uses, geometrically, from low assumptions.
- **The key results/formulas** — state them, explain the *shape* of each (what
  each symbol is, why it looks the way it does). Skip the derivation.
- **A small worked example** with concrete numbers.
- **Gotchas / intuition checks.**

Keep it tight and readable. Then **stop and tell the user to read it.**

### 3. Discuss
Invite questions. Check understanding with a question or two of your own. Adjust
the note if something didn't land. **Do not move to code until the user has
engaged.**

### 4. Work problems by hand (the default — NOT notebooks)
Build calculation fluency by **tutoring the user through a few of the book's
own exercises** (`§N.8 Exercises`), not by writing notebooks. From-scratch
notebooks were judged to add little — the user wants to *do* the math.

- Curate **2–4 strong exercises** from the chapter spanning its sub-topics.
  Present the lineup, then guide **one at a time**.
- **Guide, don't dump.** Set up the problem, state the method/formula, then let
  the user attempt each step and **check their work**. Do the heavy linear
  algebra steps *with* them, explaining geometrically as it bites — don't just
  hand over a finished solution.
- Pull exact exercise text from `MR.pdf` so numbers match the book. It's fine to
  verify the final answer with a quick `.venv/bin/python`/`modern_robotics`
  check, but the *learning* is the user's by-hand work.
- Only fall back to writing code when a problem genuinely needs numerical or
  visual exploration. (Existing `mr/` helpers and the 3a/3b notebooks stay; this
  is about how new topics are practiced.)

See the `prefer-guided-exercises-over-notebooks` auto-memory for the why.

### 5. Wrap up
Capture any clarifying Q&A from the exercises back into the note's FAQ. Update
the `README.md` progress tracker (mark the topic done, note the files). Ask
whether to continue to the next topic.

## Notes
- One concept at a time. It's better to go slow and have it stick.
- When you introduce notation, match the book and define it on first use.
- Lean on concrete numbers over abstract symbols.
