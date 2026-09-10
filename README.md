# MTRX3760 Lab 2 — Robot Simulator

Coursework for **MTRX3760 (Mechatronic System Design)** at the University of Sydney.
Lab 2 is worth 10% of the final mark and is done in pairs. It is an exercise in
object‑oriented design and modern C++ style, built with concepts from Weeks 1–4
only. All rendering is done with the [raylib](https://www.raylib.com/) library,
accessed **only** through a `CRender` wrapper class.

This repository currently contains the **Part A1 (Wall Follower)** implementation
plus the unmodified starter code. See [Status](#status) below.

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

### Source written for the lab (A1)

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

`raylib.h`, `raymath.h`, `rlgl.h`, `raylib.pc`, `raylib-config*.cmake`,
`libraylib.a`. The `.a` here is a **Linux/ELF** build — see the Windows notes
below.

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

### Linux (the "standard compile command" the unit assumes)

Install raylib (`sudo apt install libraylib5-dev`, or build 5.5 from source into
`$HOME/raylib` on a lab machine — see the comment block in `CRender.h`). Then:

```bash
g++ -Wall -Wextra -std=c++17 \
    main.cpp CRoom.cpp CRangeSensor.cpp CDriveTrain.cpp \
    CWallFollowerRobot.cpp CLoopReader.cpp CRender.cpp \
    -lraylib -o WallFollower

./WallFollower                 # defaults to SimpleWalls.map
./WallFollower SimpleWalls.map
```

Close the window to end the run; the summary (updates completed, total collisions)
prints to the console.

### Windows (MSYS2 / UCRT64) — what applies on this machine

The bundled `libraylib.a` is a Linux build and will not link here. Install the
native raylib package and link the Windows system libraries:

```bash
# once, from an MSYS2 shell (or: C:\msys64\usr\bin\pacman ...)
pacman -S --needed mingw-w64-ucrt-x86_64-raylib

# from the repo, using the UCRT64 g++ (C:\msys64\ucrt64\bin\g++)
g++ -Wall -Wextra -std=c++17 \
    main.cpp CRoom.cpp CRangeSensor.cpp CDriveTrain.cpp \
    CWallFollowerRobot.cpp CLoopReader.cpp CRender.cpp \
    -I/c/msys64/ucrt64/include -L/c/msys64/ucrt64/lib \
    -lraylib -lopengl32 -lgdi32 -lwinmm -o WallFollower.exe

./WallFollower.exe
```

> A GUI window needs a desktop session — it won't display over a plain SSH/agent
> shell.

---

## Status

- [x] **A1 — Wall Follower**: implemented (this repo).
- [ ] **A2 — Line Follower**: not started. Needs a line sensor, a line‑follower
      robot, both robots running in one program, and a shared abstraction (the A3
      questions expect the A1 design to have been refactored for A2).
- [ ] **A0 / A3** report sections.
- [ ] **A4 — ROS 2** tutorials + `House` plugin.
- [ ] **A5 — Noise** bonus.

### Submission reminders

- Submit **code only, no binaries** in the `.zip` (exclude `libraylib.a`, the
  raylib headers and any compiled `WallFollower`).
- The report is a `.pdf` with a cover page of **SIDs + practical section, no
  names**, a UML class diagram (unit's simplified standard — **no members
  shown**), an end‑of‑run screenshot, and pasted console output.
- Append all `.cpp`/`.h` as formatted text (not images).
