# A1 changes required for A2 (living doc)

Tracks every change made to A1 code to make A2 work, and every A1 design
choice that carried over untouched. This is the source material for the A3
post-mortem questions:

- *"What's an important aspect of the design of A1 that you had to change in
  order to make A2 work?"*
- *"What's an important aspect of the design of A1 that turned out well when
  building A2?"*

A1 baseline is pinned at git tag **`a1-baseline`** (commit `d3bae7e`).
Update this file as implementation proceeds — mark each row Done when the
change lands.

---

## A1 files that must change

| File | Change | Reason | Status |
|---|---|---|---|
| `CRoom.h/.cpp` | Becomes `CRoom : public CLoopShape`. Delete `CWallSegment`, `mLoopReader`, `mWalls`, `LoadFromFile`, `GetStartPose`, `RangeToWall`, `DistancePointToSegment`, `RayIntersectSegment` — all move to the new base. Keep `IsColliding` (now calls base `DistanceToNearestSegment`) and `Draw` (now `override`, calls base `DrawSegments`). | The line follower needs the same segment-chain geometry against a different loop. Sharing it via a base is an is-a: a room and a floor line are both closed loops of segments. | **skeleton done** — no body changes pending |
| `CWallFollowerRobot.h/.cpp` | Becomes `CWallFollowerRobot : public CRobot`. Delete the common half — `mPose`, `mDriveTrain`, `mTrail`, `mUpdateCount`, `mCollisionCount`, `mWasColliding`, `mrRoom`, geometry consts `mRadius/mAxleWidth/mBaseSpeed/mTimeStep`, `Update()`, `Draw()`, `GetUpdateCount()`, `GetCollisionCount()`, `HasCollided()` — all move to `CRobot`. Keep the 2 `CRangeSensor`s, the readings, the 5 tuning consts, and `SteerFromSensors()` (now `override`). Add `Sense() override`, `Name() override`. Ctor delegates to `CRobot`. | The wall follower and line follower differ only in their sensors and steering law; everything else was duplicated if left here. | **skeleton done** — steering law carried over unchanged |
| `CRangeSensor.cpp` | `arRoom.RangeToWall(...)` → `arRoom.RangeAlongRay(...)` (inherited from `CLoopShape`, same args). | `RangeToWall` moved to the base under a shape-neutral name. | **done** |
| `CLoopReader.cpp` | Replace `float( M_PI )` with a named `const float kPi`. | `M_PI` is not declared under the assignment's standard `-std=c++17` command (only `-std=gnu++17`); the A1 `.exe` in the repo was built with GNU extensions. A named, correctly-typed constant is what the style rules ask for anyway. | **done** |
| `main.cpp` | Shrinks to: parse `argv`, build `CSimulation`, `LoadMaps`, create `CRender`, `Sim.Run(Render)`. The room/robot construction and the while-loop move into `CSimulation`. | One program now runs two robots and draws two loops; that orchestration deserves its own class rather than a fatter `main`. | **skeleton done** |

## A1 files that carry over untouched

| File | Why it needed no change |
|---|---|
| `CDriveTrain.h/.cpp` | Already a pure function: given a pose and wheel speeds, returns a new pose, holds no pose of its own. Both robots reuse it as-is. |
| `CLoopReader.h` (+ `CPose`) | Interface already shape-agnostic — it reads *a* closed loop and start pose, knowing nothing about walls vs lines. `SimpleLine.map` parses with zero changes. (`CLoopReader.cpp` has the one portability tweak noted above, no logic change.) |
| `CRender.h/.cpp` | The raylib firewall was already complete; drawing a second loop and a second robot needs nothing new from it. |
| `CRangeSensor.h` | Interface unchanged — it was already handed a pose per call rather than owning one, so it composes into the new `CRobot` hierarchy unmodified (only the internal call name changes, in the `.cpp`). |

## One-line summary for A3

**Changed:** the two monolithic classes `CWallFollowerRobot` and `CRoom` had
to be split into `CRobot` / `CLoopShape` base classes plus thin derived
classes, and orchestration pulled out of `main` into `CSimulation`.

**Turned out well:** `CDriveTrain` as a pure pose-in/pose-out function,
`CLoopReader` being shape-agnostic, and `CRangeSensor` taking a pose per call
— all three dropped into the new structure with no design change.
