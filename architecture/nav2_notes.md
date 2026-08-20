# Nav2 + Behavior Tree — working notes

Status: conceptual research notes, not yet architecture decisions. This is
"what Nav2 does and why," so that when we design the courier's dispatcher/
executor on top of it, we know exactly what we're driving and what it
already handles vs what we have to handle ourselves. Cross-reference
`architecture.md` for the actual courier design.

Corrections from the original brain-dump are marked **[corrected]** — two
concepts were slightly off, everything else checked out.

---

## The pipeline, in order

**Map → Localization → Costmap → Planning → Control → Recovery → Command**

### 1. Map
Static, built once with SLAM Toolbox in Session 2, loaded from a `.pgm` +
`.yaml` pair at launch. Doesn't change at runtime — this is the "where walls
are" ground truth everything else is measured against.

### 2. Localization (AMCL)
Takes the static map + live sensor data and answers "where am I on this map
right now" — specifically, it corrects **odometry drift**. Odometry
(wheel/IMU-integrated position estimate) drifts over time because small
errors accumulate; AMCL uses a particle filter against the map + scans to
pull the estimate back to the true pose.

At sim start: the robot has to be told where it is (`2D Pose Estimate` in
RViz — sets AMCL's initial particle cloud mean + covariance, including
orientation, not just x/y). Without this, AMCL doesn't know where to start
searching.

**[corrected]** AMCL has a **global localization** capability for full "I am
completely lost, re-seed everywhere" recovery — real and useful in general,
deliberately out of scope here: adds real complexity and would need its own
recovery-tree entry if wired into a BT. Skipping it is a reasonable call for
a 2-3 day project, not a gap that needs defending.

### 3. Costmap
Built from the static map plus **live LIDAR scans** layered on top — this is
how the robot knows about things that aren't in the original map (people,
moved furniture, an obstacle dropped in Gazebo for the demo).

**[corrected] Cost values, the actual polarity:**
- `0` = free space
- `1–252` = increasing cost (inflation layer decay — cost falls off with
  distance from an obstacle)
- `253` = inscribed radius — robot's center here means guaranteed collision
  given its footprint
- `254` = lethal obstacle
- `255` = unknown

RViz has a topic to visualize this as a color gradient over the map — worth
having this up during the demo/presentation, it's the most legible way to
show "the robot understands the world," and it's explicitly something we
should screenshot/show live per the assignment ("RViz on screen helps").

### 4. Planning (`planner_server`)
Computes the best path from current pose to goal pose, **statically** —
i.e., it computes a full path once, given the costmap at that moment; it
doesn't continuously replan every tick unless something triggers a
replan (a recovery, or a costmap update that invalidates the current plan).
This is the "how do I get from A to B given what I currently believe about
the world" step — separate package/server from...

### 5. Control (`controller_server`)
Takes the static plan and turns it into actual `cmd_vel` commands, tick by
tick, in real time — this is where the plan meets reality. Includes a
**smoother**, which takes the raw computed next-move velocity and smooths it
before it's sent, rather than jerking the robot around with unsmoothed
setpoints. This is also where **critics** live — on every control cycle, Nav2
evaluates multiple candidate trajectories and scores them ("critics"), and
executes whichever rolls out best against the plan + costmap.

Open item for us, not Nav2's problem to solve, ours to *know about*: what's
our current velocity, and do we ever need to read it back (e.g. for our own
feedback message's "how far to go" — do we compute that from remaining path
distance, or from something else)? Not answered yet, revisit when drafting
the `.action` feedback fields.

### 6. Recovery (`behavior_server`)
Handles fallback behavior when something on the trajectory goes wrong.
Either produces a new plan or repairs the current one. Main recovery
behaviors: **spin** (rotate in place to gather new scans — useful when the
issue is localization confidence, since spinning gives AMCL fresh
observations from multiple angles), **back up**, and **wait** (drive-on-
heading is paused while a transient obstacle clears, e.g. something briefly
crossing the path).

Moving-obstacle prediction (tracking velocity of dynamic obstacles to plan
around them, not just react to where they are *now*) is a real thing Nav2
*can* do but is mathematically deeper — flagged as out of scope, correctly.

### 7. Collision Monitor
**Not part of the Nav2 planning/control pipeline** — a separate safety layer
that watches `cmd_vel` against live scan data and can issue a **hard,
immediate stop**, overriding whatever the controller wanted to do. This
exists specifically because "avoid collision" has to be guaranteed even if
something upstream (planner, controller, even a recovery) is momentarily
wrong — it's the last line, not a recovery *option* the BT chooses among.

### Lifecycle management
Nav2 servers (`map_server`, `amcl`, `planner_server`, `controller_server`,
`behavior_server`, `bt_navigator`, ...) are **lifecycle nodes** —
configure → activate, in a specific dependency order, managed by the
**lifecycle manager**. Each node must be configured (params bound, publishers/
subscribers/topics set up) *before* activation; if any one node in the chain
fails to activate, the whole chain should not proceed halfway — a partially-
activated stack means a missing step downstream (e.g. controller active but
no valid costmap yet), which is worse than not starting at all.

Ordering constraint worth remembering: **map → localization must be
confirmed correct before costmap/planning/navigation are allowed to
proceed** — Nav2 has a deliberate startup delay here so AMCL has time to
converge before anything downstream trusts its output. Garbage localization
in means garbage planning out; the lifecycle manager's job is partly to
prevent that from happening silently.

**Action item, already flagged by Maya: the presentation must show the
lifecycle explicitly** (which nodes, what order, what "configured" vs
"active" means) as part of demonstrating the system actually starts up
correctly — not just that it eventually works.

---

## Why action, not service, for the delivery — the live-verification note

Confirmed (not just theoretical) reason actions fit requirement 3: an action
gives us **goal, ongoing feedback, and a result**, plus the ability to check
"is the actual goal reached" continuously rather than just fire-and-forget.
That maps directly onto what we need: goal checker running (candidate: once
a second, matching requirement 4's "at least once a second" feedback cadence)
and the ability to **trigger a recovery action** — including, from the BT
side later, kicking off a distinct recovery *subtree* — if something like
localization confidence drops mid-leg. This is exactly the same shape Nav2
itself uses `navigate_to_pose` for, one level up.

---

## Behavior Trees

An XML tree of nodes. Two core composite types:

- **Sequence** — children evaluated left to right, **AND** semantics: every
  child must succeed for the sequence to succeed; first failure aborts the
  sequence.
- **Fallback** (a.k.a. Selector) — children evaluated left to right, **OR**
  semantics: first child to succeed ends evaluation as success; only fails
  if every child fails.

Leaf nodes are actions/conditions with their own identifiers, and — like
regular code — can take arguments/ports. BT.CPP (already present as a Nav2
dependency) is the library; trees are authored as XML so they're editable
without recompiling, per the assignment's bonus track.

**Tooling note for later:** there are XML→tree visualizers for BT.CPP
(Groot / Groot2 is the standard one) that draw the tree from the XML and let
you sanity-check the logic visually rather than only reading raw XML — worth
using once we're actually authoring the courier's tree, both to build it and
to verify sequence/fallback nesting is doing what we think it's doing.

**Scope note, already decided in `architecture.md` section 6:** we are
*not* designing our leaf-node boundaries around a BT yet. This document is
purely "understand what Nav2's BT does and how BT.CPP works," so that when
we do port the courier's mission logic (pickup leg → dropoff leg → retry →
fail-cleanly) into a tree later, we're not learning the tool and designing
the tree's shape at the same time.

---

## Open threads this session surfaced, not yet resolved

- How do we compute "how far to go" for our own feedback message — remaining
  path length from the costmap/plan, or straight-line distance to goal, or
  something Nav2 already exposes we can just relay? Not decided.
- Do we ever need our own recovery trigger distinct from Nav2's built-in
  recovery behaviors, or do we just let Nav2 exhaust its own recoveries and
  treat "Nav2 aborted the leg" as our single failure signal (requirement 7)?
  Leaning toward the latter for simplicity, not decided.
