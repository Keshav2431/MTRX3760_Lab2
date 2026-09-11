# Lab 2 — report answers (personal working notes)

Not the report. Draft answers + raw material for every question in the lab
brief, A0–A3, to pull from when writing the actual `.pdf`.

- **[FILL]** = needs your / your partner's real input; the text is a placeholder to adapt.
- Technical answers (A3 on A1→A2 and on the A2 code) are solid — lifted from the
  actual design and `docs/superpowers/specs/`.
- Cover page reminder: **SIDs + practical section only, no names.**

---

## A0 — Team Setup [10 marks]

Write each member's answer to each question. Keep it concrete and short; the
markers want evidence you actually thought about how to work together, not a
perfect team.

### 1. Each member's main strength relevant to this lab

- **You (Davin) — [FILL]**  e.g. *"Object-oriented design and C++ structure —
  comfortable planning class hierarchies and interfaces before coding, and
  keeping to a consistent style."*
- **[Partner] — [FILL]**  e.g. *"Debugging and geometry / control maths —
  good at reasoning about why the simulation behaves the way it does and
  turning that into a working control law."*

*(Pick strengths that actually map to the work: OO design, C++ style, the
sensor/steering maths, raylib/rendering, git workflow, writing.)*

### 2. Each member's main area for improvement relevant to this lab

- **You (Davin) — [FILL]**  e.g. *"Resisting the urge to start coding before
  the design is settled; I want to get better at finishing the class diagram
  first."*
- **[Partner] — [FILL]**  e.g. *"C++ const-correctness and header hygiene —
  knowing exactly what needs to be `const` and which includes are actually
  needed."*

### 3. How you will arrive at decisions / resolve disagreements

**[FILL — adapt this draft]**

> We make design decisions together at the start of each part, sketching the
> class diagram on paper or a whiteboard before any code is written. For small
> decisions during implementation, whoever owns that file decides and notes the
> choice in a comment. If we disagree on something structural, we each state the
> trade-off we care about, try to find a third option, and if we are still
> stuck we default to the simpler design (YAGNI) and revisit only if it causes
> real problems. The unit's style checklist and the "encapsulate everything /
> is-a vs has-a" rules act as a tie-breaker.

### 4. How you will keep on schedule

**[FILL — adapt, put in real dates]**

> The lab is due before our Week [6/7] lab session. We work backwards from
> that:
> - Week 1: A0 written, A1 designed (class diagram agreed) and implemented.
> - Week 2, first half: A2 designed and implemented, both robots running.
> - Week 2, second half: A3 written, A4 ROS tutorials + House plugin, report
>   assembled, code appendix generated, buffer for the screenshot/console runs.
> We check in [e.g. every 2 days / after each part] and keep a short TODO list
> in the repo so nothing is discovered late. Revision control means either of us
> can pick up where the other stopped.

### 5. Will you use revision control and code review? Why / why not?

**[FILL — this is basically already true, adapt]**

> Yes to both. We use git with a branch per person and short-lived feature
> branches, so our work never blocks each other and we can always get back to a
> known-good state (we tag the finished A1 as `a1-baseline`). Every change is
> looked at by the other person before it lands — for a design-and-style unit,
> a second pair of eyes catches style-checklist slips (magic numbers, missing
> `const`, unnecessary includes) and design smells far more cheaply than the
> marker will. It also forces us to keep each commit small and explainable.

---

## A1 — Wall Follower: what the report must contain

Not questions — deliverables. Checklist:

| Item | Where / status |
|---|---|
| UML class diagram, simplified standard, **no members/methods** | Make an A1-only version: `CWallFollowerRobot` + `CRoom` + `CRangeSensor` + `CDriveTrain` + `CLoopReader` + `CRender` + `CPose`. It's the pre-refactor shape — no `CRobot`/`CLoopShape`/`CSimulation`. Draw it in PlantUML like `docs/A2-UML.md`. |
| End-of-run screenshot: trail all the way around `SimpleWalls.map` | **TODO** — run `A1/WallFollower` on a desktop session, let the wall follower complete the loop, screenshot. |
| Console output pasted, showing any collisions | **TODO** — capture the run summary (`Updates completed`, `Total collisions`) and any `Collision #n` lines. Target 0, must be ≤ 10. |

Design points to mention in the A1 write-up (they carry marks even though A1's
code is shared with A2):

- raylib is firewalled behind `CRender`; nothing else includes `raylib.h`.
- Fixed simulated timestep (`mTimeStep = 0.05 s`), never real elapsed time.
- `CDriveTrain::Advance` is a pure function: pose in → pose out, no stored pose.
- Collisions are edge-triggered (`mWasColliding`) — counted once on first
  contact, not every frame.
- Steering: two-mode proportional controller — corner mode (45° sensor sees a
  wall closing in → steer left) overrides wall-hold mode (hold the 90° sensor
  at `mTargetWallDistance`). Error is clamped so a lost (max-range) reading
  can't demand a violent turn.

---

## A2 — Line Follower: what the report must contain

| Item | Where / status |
|---|---|
| UML class diagram, simplified standard, **no members/methods** | `docs/A2-UML.md` (PlantUML) — export to PNG/SVG. |
| End-of-run screenshot: **both** robots' trails complete, `SimpleWalls.map` + `SimpleLine.map` | **TODO** — run `A2/WallFollower`, let both loops close, screenshot. |

Design points to mention in the A2 write-up:

- Two small inheritance hierarchies, each isolating one axis of variation:
  `CRobot` (how a robot senses + steers) and `CLoopShape` (what a closed loop
  *means* — a wall you hit vs. a line you sense).
- `CRobot::Update()` is a template method: `Sense()` → `SteerFromSensors()` →
  advance one timestep → reject the move on collision → extend the trail. The
  two subclasses override only `Sense()` and `SteerFromSensors()`.
- `CLineSensor` is deliberately **not** related to `CRangeSensor`: one is
  mounted by angle and returns a distance, the other by position and returns a
  bool. They share nothing but a purpose.
- Line control law: bang-bang on the two wheel speeds. Inner sensor over the
  line, outer sensor on the interior (right) side, just off the paint. Four
  sensor combinations → {straight, right, right-harder, left}. Corrections run
  at `mTurnSpeed` (below base speed) so a sharp vertex is taken as a near-pivot
  and the line is never lost. `SimpleLine.map` runs clockwise with the interior
  on the right, so five corners bend right and the one concave corner (vertex
  2) bends left; a lost line always means it's now to the left, so recovery
  turns left.
- `CSimulation` owns the room, the line and both robots (as
  `vector<unique_ptr<CRobot>>`), so `main` is ~15 lines.

---

## A3 — Post-Mortem [10 marks]

Plain-English drafts. Lift and tweak; the `[FILL]` bits need your real input.

### Reflection on A0 — each member answers on their own

**What did each team member contribute to the submission?**

- **You (Davin) — [FILL with your real split]**  Example: "Worked out the class
  design with [partner] (splitting the code into `CRobot`, `CLoopShape` and
  `CSimulation`), wrote and tuned the line-following rules against
  `SimpleLine.map`, sorted the code into the `A1/` and `A2/` folders, cleaned
  up the style of the new A2 files, drew the class diagram, and wrote the
  README and notes."
- **[Partner] — [FILL]**  Example: "Built the A1 wall follower and its steering,
  wrote the distance-measuring helper functions, ran the program and took the
  screenshots, wrote the A0 and A3 sections, and did the ROS work for A4."

*(Split this honestly — the markers compare the two answers.)*

**What is one thing you would improve about the Team Setup in future?**

- **You (Davin) — [FILL]**  Example: "Agree the class diagram and stick to it
  before writing any code. We changed the robot classes after we had started,
  which meant redoing `main` and `CRoom`."
- **[Partner] — [FILL]**  Example: "Set a small deadline for each part instead
  of one deadline for the whole lab, so we notice early if we are falling
  behind."

### Reflection on A1 — one answer you both agree on

**What is an important part of the A1 design you had to change to make A2 work?**

> In A1, two classes were doing too much. `CWallFollowerRobot` held everything
> about the robot: its position, its wheels, its trail, the collision checks,
> the update step, the drawing — *and* the wall sensors and the steering rules.
> `CRoom` also mixed two jobs: the maths for measuring distance to a wall, and
> the idea of a wall you can crash into. Because everything was bundled
> together, a line-following robot could not reuse any of it.
>
> To make A2 work we split both classes. The shared robot parts moved into a
> new base class, `CRobot`. It runs the update step and calls two functions,
> `Sense()` and `SteerFromSensors()`, that each type of robot fills in its own
> way. The shared maths moved into a new base class, `CLoopShape`, that any map
> file can be built on top of. `CWallFollowerRobot` and the new
> `CLineFollowerRobot` are now short classes on top of `CRobot`; `CRoom` and
> the new `CFloorLine` are short classes on top of `CLoopShape`. We also moved
> the setup and the main loop out of `main()` and into a new `CSimulation`
> class, because the program now has to run two robots and two maps at once.

**What is an important part of the A1 design that turned out well when building A2?**

> Three choices from A1 needed no change at all. `CDriveTrain` was already
> written as a plain function: give it a position and two wheel speeds, and it
> hands back a new position while storing nothing itself, so both robots use it
> as-is. `CLoopReader` already did not care what it was reading — it just loads
> a closed loop of points — so `SimpleLine.map` loaded with no new code. And
> `CRangeSensor` was given the robot's position every time it was asked to
> sense, instead of keeping its own copy, so it dropped straight into the new
> robot class. Keeping those three classes unaware of the bigger picture is
> what made the rework fast.

### Reflection on A2 — one answer you both agree on

**One part of the program's structure you are especially happy with**

> The `CRobot` base class. To add a new kind of robot you only write two
> functions — one to read its sensors, one to set its wheel speeds. Everything
> else (the fixed time step, the collision handling, the trail, the drawing,
> the end-of-run summary) is shared and behaves the same for both robots.
> `CLoopShape` does the same for the world: `CRoom` asking "am I hitting a
> wall?" and `CFloorLine` asking "am I on the line?" are each one line of code
> on top of a shared distance check.

**One part of the program's structure you would most want to improve**

> The `CLoopShape` base class carries a function that only one of its two
> children needs. `RangeAlongRay` — measure the distance to the nearest wall
> along a given direction — is only ever used for the room, by the range
> sensors. Nothing ever fires a ray at the floor line. So `CFloorLine`
> inherits a function it never uses, which is a sign the base class has been
> given slightly too much. Only the shorter "distance to the nearest segment"
> check is truly shared (the room uses it for collisions, the line uses it for
> sensing). Moving `RangeAlongRay` down into `CRoom` would leave the base with
> exactly what both classes need and nothing more.

**One part of the program's style you are especially happy with**

> No magic numbers, and comments that say *why*. The line width is one named
> value and everything else is worked out from it; the "no hit" result in the
> ray test has the name `NoIntersection` instead of being a bare `-1`; every
> tuning value is a named constant. The comments explain the reasoning rather
> than the obvious — for example, `CLineFollowerRobot.cpp` starts with the full
> table of what to do for each pair of sensor readings, and why the side sensor
> is mounted where it is, so the steering code itself barely needs comments.

**One part of the program's style you would most want to improve**

> Being consistent across the whole submission. The new A2 files are tidy, but
> we chose not to touch the older A1 files, so `CWallFollowerRobot.cpp` still
> has an `#include` it no longer uses and a couple of constants sitting outside
> the class where the newer code puts them inside. The same idea ends up
> written two ways in one zip. Also, the block of build instructions at the top
> of `main.cpp` is longer than the code below it and repeats what the README
> already says — only one of them should be the real source.

---

## Self-Assessment Bonus (rough, for later)

Fill the real numbers near submission. My hedged estimate given the code as it
stands:

| Component | Estimate | Reasoning |
|---|---|---|
| Functionality | Pass | Both robots complete their loops from sensors; wall follower ≤ 10 collisions. |
| A0 | [FILL] / 10 | Depends entirely on how concrete the written answers are. |
| Design | ~24–27 / 30 | Clean two-hierarchy split, template method, clear ownership; small dents: `Color` leak, triple robot-loop, `CPose`/`Vec2D` public data. |
| Code | ~24–27 / 30 | Consistent style, no magic numbers, const-correct, comments explain why; dents: A1-heritage inconsistency, one redundant assignment kept for parallelism, long `main` comment header. |
| A3 | ~8–9 / 10 | Technical reflections are specific and honest; A0 reflections need real per-member content. |

Do not include the self-assessment bonus itself in the estimate. Within 5 of
the real total earns 5 marks.
