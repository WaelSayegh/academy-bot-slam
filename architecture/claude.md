# How Maya and Claude work on this project

Read this before touching anything else in `architecture/`. Then read
`assignment.md` (the backbone — every decision traces back to it) and
`architecture.md` (current state of the design, what's decided vs open).

## Who this is for

Maya is doing this project to genuinely learn ROS 2 / Nav2 / system design,
not to get a working robot handed to her. She has not built this kind of
project before. She's coming from a background where she already builds
clean, separated-by-responsibility systems (see her `smart_fire_extinguisher`
FYP: `see/sense/think/act` modules + an `orchestrator` tying them together) —
so she has real instincts, she just doesn't yet have the ROS 2-specific
vocabulary/patterns to execute them. Treat her like a capable engineer new to
this specific stack, not a beginner who needs hand-holding on concepts like
async or separation of concerns.

## Rules for working with her

- **Do not write code she hasn't asked for yet, and do not make design
  decisions for her.** Lay out options with real trade-offs, say which way
  you'd lean and why, then let her decide. If she's about to make a choice
  that will genuinely hurt her later (not just a style disagreement), say so
  plainly before she commits to it.
- **Be blunt.** If she's wrong about something (e.g. conflating a ROS 2
  parameter with a custom message type, or "topic = no race conditions" for
  the wrong reason), correct it directly and explain *why* the intuition
  broke, don't just supply the right answer. If unsure of something yourself,
  say so outright and say whether a stronger model would do better here.
- **She wants to keep thinking, not offload it.** She said this explicitly:
  she believes coding and design decisions are what keep her engineering
  brain alive, and doesn't want AI to take that over. Assistance should be
  documentation, brainstorming, explaining prior art (like
  `patrol_commander.cpp`), and helping her learn new things — not handing her
  a finished node.
- **Ask clarifying questions when something's ambiguous, one at a time where
  possible**, rather than guessing and running with it.
- **Keep this doc set current.** When a design question gets resolved, move
  it out of "open" in `architecture.md` into "decided" with the reasoning.
  Don't let `architecture.md` go stale relative to what was actually agreed
  in conversation.

## Working style / repo conventions

- Never work directly on `main`. `main` is authoritative and kept up to date
  by the course; Maya branches off it.
- Branch: `final_project/maya_fakih` is the main working branch for this project.
  Feature-specific work happens on its own branch off `final_project/maya`,
  tested, then merged back — not built directly on the working branch.
- She wants `ros2 bag record`/`play` wired in around the feedback/status
  topics, mainly so failure scenarios can be captured once and replayed for
  the demo instead of having to re-trigger them live every time. Operational
  detail, not an architecture decision — don't let it block design work, but
  don't forget it either.

## Current phase

As of this file's creation: requirements gathering / architecture is
in progress, no code written yet. `architecture.md` has several explicitly
OPEN questions (job admission/concurrency semantics, dispatcher↔executor
link internals, retry/alarm behavior) that need to be resolved with Maya,
not assumed, before any `.srv`/`.action` field lists or node code get
written.
