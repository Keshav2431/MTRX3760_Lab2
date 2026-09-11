# MTRX3760 Lab 2 — Robot Simulator

Coursework for **MTRX3760 (Mechatronic System Design)** at the University of Sydney.
Lab 2 is worth 10% of the final mark and is done in pairs. It is an exercise in
object‑oriented design and modern C++ style, built with concepts from Weeks 1–4
only. All rendering is done with the [raylib](https://www.raylib.com/) library,
accessed **only** through a `CRender` wrapper class.

The code is split into two self-contained programs:

- **`A1/`** — the Wall Follower on its own, frozen as submitted for A1.
- **`A2/`** — the Wall Follower **and** the Line Follower in one program, after
  the A1 classes were refactored into shared base classes. This is the active
  codebase.
- **`A5/`** — the noise bonus: a copy of A2 with a randomised start pose,
  per‑step wheel slip, and 20 of each robot type running at once.

Each folder builds independently (see [Building and running](#building-and-running)).
The vendored raylib files at the repo root are shared reference only. See
[Status](#status) below.

---

## The assignment at a glance

| Part | Marks | What it asks for |
|------|-------|------------------|
| **A0** — Team Setup | 10 | Written team‑charter answers (strengths, decision‑making, schedule, revision control). |
| **A1** — Wall Follower | Design 30 / Code 30 (shared with A2) | A disc robot that follows the wall on its right‑hand side all the way around `SimpleWalls.map`, using two range sensors (90° and 45° to the right) and a two‑wheel differential drive. Fixed simulated timestep, persistent trail, console collision reports, end‑of‑run summary. |
| **A2** — Line Follower | (as above) | A second robot in the *same* program that follows a floor line (`SimpleLine.map`) using two line sensors (one over the line, one beside it). Both robots run at once and do not interact. |
| **A3** — Post‑Mortem | 10 | Reflection questions on A0/A1/A2. |
| **A4** — ROS 2 Tutorials | 20 | Work through the ROS 2 *Jazzy* beginner tutorials (C++), then extend the `pluginlib` polygon example with a `House` (square + triangle, equal `side_length`). Submit `polygon_plugins.cpp`. |
| **A5** — Noise Bonus | +10 | Copy of A2 with randomised start pose and per‑step wheel noise, 20 of each robot type at once. |

### Minimum Functionality Hurdle

To be graded at all, the code must build with the standard compile command, run,
and drive **each robot from its sensors** (no fixed paths). Both robots must get
around their circuits leaving a trail, and the wall follower must finish with
**≤ 10 collisions** (0 is the design target). Once cleared, marks are for design,
code and process quality — not path‑following accuracy.

### Style rules the unit enforces (Weeks 1–4)

`std::` not `using namespace`; informative names; one statement / one variable per
line; consistent defensive bracing and indentation; `const` instead of `#define`;
meaningful comments only; all data `private`; objects take care of themselves;
`is‑a` → inheritance, `has‑a` → composition; initialise every variable; validate
arguments; no magic numbers; correct constant types; minimal `#include`s; pass by
`const` reference; const‑correctness; "encapsulate everything" (maximise locality —
consts, enums, data and functions live inside classes).

---

## Repository layout

```
A1/                 wall follower only, frozen as submitted for A1
A2/                 wall + line follower in one program (active)
A5/                 noise bonus: copy of A2, randomised, 20 of each robot
docs/superpowers/   design spec and the A1→A2 change log
raylib.h, ...       vendored raylib, shared reference (not submitted)
```

### A5 — files added or changed on top of A2

`A5/` is a **standalone copy** of `A2/`, as the brief requires ("make a copy of
your A2 program and augment it"). A2 is untouched by it.

Only **one** new class (`CNoiseSource`); everything else is a small change to an
existing A2 file.

| File | Responsibility |
|------|----------------|
| `CNoiseSource.h/.cpp` | **New.** The only class that produces random numbers: holds the generator and the three noise sizes, and offers `ScatterStartPose()` (shift a start pose) and `AddWheelSlip()` (slip one wheel's step). One instance is made in `CSimulation` and shared by all 40 robots, so one seed fixes the whole run. |
| `CDriveTrain.h/.cpp` | `Advance()` now works out the **distance each wheel moves this step**, adds a slip to each, then combines them (average = forward, difference = turn). Same motion as A2 with the slip removed. This is the only place the wheel noise enters. |
| `CRobot.h/.cpp` | Takes the shared noise source. Adds a plain **distance‑based lap test** (`CheckLap()`: the robot must drive >150 units from its start, then return within 30). Parks once its lap is done. Trail is **sampled by distance** — an every‑step trail for 40 robots would be tens of thousands of segments per frame. Per‑collision console lines dropped (A5 doesn't count collisions; 40 robots would flood the console) — the counts are still kept and reported per type. `Draw()` split into `DrawTrail()` / `DrawBody()` for a two‑pass draw. |
| `CRender.h/.cpp` | New `static Color Lighten( colour, fraction )` — mixes a colour toward white. Each robot in a group of 20 gets its own shade of the one family colour, so the trails can be told apart. |
| `CSimulation.h/.cpp` | Holds two `std::vector<std::unique_ptr<CRobot>>` groups. Builds 40 robots with scattered start poses, owns the noise source, and ends the run itself once both groups have finished (or the step cap is hit), holding the finished picture for the screenshot. |

`CLoopShape` and every other A2 file are copied across unchanged. The changes to
the A1‑heritage files (`CDriveTrain`, `CRender`, `CWallFollowerRobot`) are the
functional minimum for the noise/shade feature, kept in the heritage style — no
comment rewrite. (`A5/CRender.cpp` also gets a real window‑title string in place
of the placeholder.)

### A2 — files added or changed on top of A1

| File | Responsibility |
|------|----------------|
| `CSimulation.h/.cpp` | Owns the room, the floor line and both robots; runs the fixed‑step loop over them; prints a per‑robot summary. `main.cpp` just builds one and calls `Run()`. |
| `CLoopShape.h/.cpp` | **New base class.** A closed loop of segments loaded from a `.map`; all the ray/point geometry that used to live in `CRoom`. `RangeAlongRay`, `DistanceToNearestSegment`. |
| `CRobot.h/.cpp` | **New base class.** The common half of every robot: pose, drive train, trail, counts, the `Update()` template method, `Draw()`. Subclasses fill in `Sense()` and `SteerFromSensors()`. |
| `CFloorLine.h/.cpp` | `CLoopShape` subclass. `IsLineUnder(point)` — true within half the 5‑unit line width of a segment. |
| `CLineSensor.h/.cpp` | A point sensor mounted at a fixed `(forward, lateral)` offset in the robot frame. `Sense(pose, line)` → `bool`. |
| `CLineFollowerRobot.h/.cpp` | `CRobot` subclass. Two `CLineSensor`s (one over the line, one beside it); a bang‑bang steering law. |
| `CRoom.h/.cpp` | Now a `CLoopShape` subclass — just `IsColliding` and wall drawing. |
| `CWallFollowerRobot.h/.cpp` | Now a `CRobot` subclass — just the two range sensors and the two‑mode steering law. |
| `CRangeSensor.cpp` | One call renamed (`RangeToWall` → inherited `RangeAlongRay`). |
| `CLoopReader.cpp` | `M_PI` → a named `kPi` constant (portability under strict `-std=c++17`). |

### A1 — source written for the lab

| File | Responsibility |
|------|----------------|
| `main.cpp` | Entry point. Loads a room, creates the render window and the robot, runs the fixed‑step sense→draw loop until the window closes, prints the run summary. |
| `CRender.h/.cpp` | Thin wrapper hiding raylib. The rest of the program never includes `raylib.h`. Exposes `Vec2D`, window control and `DrawCircle` / `DrawLine`. |
| `CLoopReader.h/.cpp` | Parses a `.map` (loop) file into a name, a `CPose` start pose, and a vertex list. Handles `#` comments and blank lines; reports the offending line on a parse error. Defines `CPose` (position + heading in radians). |
| `CRoom.h/.cpp` | Turns the loop's vertices into a closed chain of wall segments. Answers `RangeToWall` (ray‑vs‑segment cast) and `IsColliding` (disc‑vs‑segment), and draws the walls. |
| `CRangeSensor.h/.cpp` | One ray sensor mounted at a fixed angle relative to the robot heading. `Sense(pose, room)` returns the distance to the nearest wall along its beam, capped at a max range. |
| `CDriveTrain.h/.cpp` | Two‑wheel differential drive. Told a left/right wheel speed, `Advance(pose, dt)` returns the new pose. Pure function of its inputs — holds no pose itself. |
| `CWallFollowerRobot.h/.cpp` | The robot. Owns its pose, drive train and two range sensors. `Update()` senses, steers, moves one timestep and checks collisions; `Draw()` renders body, heading indicator and trail. Tracks update and collision counts. |

### Provided data

| File | Purpose |
|------|---------|
| `SimpleWalls.map` | The A1 test room: a rectangle with a notch in the top edge (one concave corner). Start pose near the top‑left, facing right. |
| `SimpleLine.map` | The A2 test line: a closed 7‑sided floor loop inside the room, with one concave corner. Not used by A1. |

### Vendored raylib (do **not** submit these)

At the repo root, shared by both folders: `raylib.h`, `raymath.h`, `rlgl.h`,
`raylib.pc`, `raylib-config*.cmake`, `libraylib.a`. The `.a` here is a
**Linux/ELF** build — see the Windows notes below. The build commands do not
reference these; they rely on an installed raylib, as the assignment assumes.

---

## Architecture

```
main
 ├─ CRoom ──────── CLoopReader        (room owns the reader; reader owns CPose + vertices)
 ├─ CRender                            (raylib firewall; everyone draws through it)
 └─ CWallFollowerRobot
      ├─ CPose            (has‑a: current pose)
      ├─ CDriveTrain      (has‑a: differential drive)
      ├─ CRangeSensor  ×2 (has‑a: side sensor @ 90°, forward sensor @ 45°)
      └─ const CRoom&     (uses, does not own: senses and collides against it)
```

Key design points:

- **raylib is firewalled** behind `CRender`. No other translation unit includes
  `raylib.h`; drawing goes through `Vec2D` + `DrawCircle`/`DrawLine`.
- **Fixed simulated timestep.** `CWallFollowerRobot::mTimeStep` (0.05 s) advances
  the sim each frame; real elapsed time is never read.
- **`CDriveTrain::Advance` is a pure function** — takes a pose, returns a pose,
  mutates nothing. The robot decides what to do with the result (accept it, or on
  collision keep only the heading change and reject the translation).
- **Collisions are edge‑triggered.** Counted/printed once when the robot first
  touches a wall (`mWasColliding`), not every frame it stays in contact.

### Wall‑following control law (`SteerFromSensors`)

Two‑mode proportional controller on the two wheel speeds around a base speed:

1. **Corner mode** — if the 45° forward‑right sensor reads below
   `mCornerThreshold` (70), a wall is closing in ahead; steer left, harder the
   closer it is (`mCornerGain` 1.4).
2. **Wall‑hold mode** — otherwise hold the 90° side sensor at
   `mTargetWallDistance` (60): too far → steer toward the wall, too close → steer
   away (`mSteeringGain` 0.9).

Steering error is clamped (`kMaxSteeringError` 100) so a lost reading (max range)
can't demand a violent turn.

### Map / loop file format

One instruction per line; `#` to end‑of‑line is a comment; blank lines ignored.

```
loop <name>               # exactly one per file
startpose <x> <y> <deg>    # a point on/near the loop and a heading
vertex <x> <y>             # a corner; the last vertex joins back to the first
```

Coordinates are renderer space: x right, y down, in an 800 × 600 window.
Headings are degrees clockwise from +x (0 right, 90 down), converted to radians on
load.

---

## Building and running

`cd` into `A1/` or `A2/` first — each folder is a complete program.

### Linux (the "standard compile command" the unit assumes)

Install raylib (`sudo apt install libraylib5-dev`, or build 5.5 from source into
`$HOME/raylib` on a lab machine — see the comment block in `CRender.h`). Then:

```bash
# A2 (wall + line follower)
cd A2
g++ -Wall -Wextra -std=c++17 \
    main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp \
    CRangeSensor.cpp CLineSensor.cpp CDriveTrain.cpp CRobot.cpp \
    CWallFollowerRobot.cpp CLineFollowerRobot.cpp CLoopReader.cpp CRender.cpp \
    -lraylib -o WallFollower

./WallFollower                                   # SimpleWalls.map + SimpleLine.map
./WallFollower SimpleWalls.map SimpleLine.map

# A1 (wall follower only)
cd A1
g++ -Wall -Wextra -std=c++17 \
    main.cpp CRoom.cpp CRangeSensor.cpp CDriveTrain.cpp \
    CWallFollowerRobot.cpp CLoopReader.cpp CRender.cpp \
    -lraylib -o WallFollower

# A5 (noise bonus: 20 of each robot type)
cd A5
g++ -Wall -Wextra -std=c++17 \
    main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp \
    CRangeSensor.cpp CLineSensor.cpp CNoiseSource.cpp CDriveTrain.cpp \
    CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp \
    CLoopReader.cpp CRender.cpp \
    -lraylib -o NoiseBonus

./NoiseBonus
```

For A1 and A2, close the window to end the run; the per‑robot summary (updates
completed, total collisions) prints to the console.

A5 instead **ends by itself** once every robot has finished its lap (or at the
2500‑step cap), prints its summary at that moment, and holds the finished
picture on screen — so the console can be read and the screenshot taken while
the window is still up. Close the window when done.

### Windows (MSYS2 / UCRT64) — what applies on this machine

The bundled `libraylib.a` is a Linux build and will not link here. Install the
native raylib package and link the Windows system libraries:

```bash
# once, from an MSYS2 shell (or: C:\msys64\usr\bin\pacman ...)
pacman -S --needed mingw-w64-ucrt-x86_64-raylib

# then, per folder, using the UCRT64 g++ (C:\msys64\ucrt64\bin\g++), add:
#   -I/c/msys64/ucrt64/include -L/c/msys64/ucrt64/lib
#   -lopengl32 -lgdi32 -lwinmm
# to the matching command above, and use -o WallFollower.exe
```

> A GUI window needs a desktop session — it won't display over a plain SSH/agent
> shell.

---

## Status

- [x] **A1 — Wall Follower**: implemented, frozen in `A1/`.
- [x] **A2 — Line Follower**: code complete. `CRobot` / `CLoopShape` base
      classes, `CSimulation`, `CLineSensor`, `CLineFollowerRobot` with a tuned
      bang‑bang control law. Both programs build clean with
      `-Wall -Wextra -std=c++17`. A Week 1–4 style pass has been done on the
      **new** A2 classes (short class comments, a comment on every header
      function, ctors/dtors declared in the header and defined in the `.cpp`);
      the six A1‑heritage files are left as they were, on purpose — their known
      rough edges are the A3 "style to improve" answer. Remaining: end‑of‑run
      screenshot + console capture for the report.
- [ ] **A0 / A3** report sections. See `docs/superpowers/specs/` for the design
      spec and the A1→A2 change log (A3 material).
- [ ] **A4 — ROS 2** tutorials + `House` plugin.
- [x] **A5 — Noise bonus**: code complete in `A5/` — one new class
      (`CNoiseSource`) plus small changes to five A2 files. Builds clean with
      `-Wall -Wextra -std=c++17`. Verified headlessly (stub raylib, see
      `docs/HANDOFF.md`): at the committed seed all **20 of 20** wall followers
      and **20 of 20** line followers finish their loops in ~1900 steps, and
      that holds across six seeds. Remaining: the end‑of‑run screenshot and
      console capture from a real desktop run.

### Submission reminders

- Submit **code only, no binaries** in the `.zip` (exclude `libraylib.a`, the
  raylib headers and any compiled `WallFollower`).
- The report is a `.pdf` with a cover page of **SIDs + practical section, no
  names**, a UML class diagram (unit's simplified standard — **no members
  shown**), an end‑of‑run screenshot, and pasted console output.
- Append all `.cpp`/`.h` as formatted text (not images).
