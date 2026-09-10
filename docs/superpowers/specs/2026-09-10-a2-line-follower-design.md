# A2 — Line Follower: Design

MTRX3760 Lab 2. Date: 2026-09-10.

## Goal

Add a second robot to the simulator that follows the floor line in
`SimpleLine.map` using two line sensors, running in the same program and the
same window as the A1 wall follower. Both robots run at once, do not interact,
and each must get all the way around its own circuit leaving a persistent
trail. The wall follower must still finish with 10 or fewer collisions.

The work is a refactor as much as an addition: the parts of
`CWallFollowerRobot` and `CRoom` that are not specific to wall following move
into shared base classes so the line follower can reuse them.

## Constraints carried over from A1

- Weeks 1-4 C++ only. `std::` prefix, one statement / one variable per line,
  all data private (protected where a derived class genuinely needs it, with a
  comment saying why), `const` not `#define`, no magic numbers, validate
  arguments, pass by `const` reference, const-correct, initialise every
  variable, minimal `#include`s.
- All raylib access stays behind `CRender`. No other translation unit includes
  `raylib.h`.
- Fixed simulated timestep. Real elapsed time is never read.
- One header per major class, `C`-prefixed class names, `a`/`ar`/`m` parameter
  and member prefixes, matching the existing files.
- No exceptions — failures are reported by `bool` return and a console
  message, as in A1.

## Architecture

```
main
 └─ CSimulation                      owns the room, the line, both robots
      ├─ CRoom            : CLoopShape      walls; answers IsColliding
      ├─ CFloorLine       : CLoopShape      floor line; answers IsLineUnder
      └─ std::vector<std::unique_ptr<CRobot>>
           ├─ CWallFollowerRobot : CRobot   2x CRangeSensor, corner/wall-hold law
           └─ CLineFollowerRobot : CRobot   2x CLineSensor,  bang-bang law

CRobot (abstract)   owns CPose, CDriveTrain, trail, counts; runs Update()
CLoopShape (abstract) owns the segment chain + all ray/point geometry
```

Unchanged from A1: `CDriveTrain`, `CLoopReader` (+ `CPose`), `CRender`,
`CRangeSensor` (one internal call retargeted, see below).

New files: `CLoopShape.h/.cpp`, `CFloorLine.h/.cpp`, `CLineSensor.h/.cpp`,
`CRobot.h/.cpp`, `CLineFollowerRobot.h/.cpp`, `CSimulation.h/.cpp`.

## Components

### CLoopShape (new, abstract base)

A closed loop of straight segments loaded from a `.map` file. Holds everything
`CRoom` currently does about geometry, so `CFloorLine` gets it for free.

- `struct CSegment { Vec2D mStart; Vec2D mEnd; };` (nested, was `CWallSegment`).
- `bool LoadFromFile( const std::string& arFilename )` — reads the loop with a
  `CLoopReader`, builds the closed segment chain (last vertex joins the first).
  Returns `false` on open/parse failure or an empty vertex list.
- `const CPose& GetStartPose() const` — forwards the reader's start pose.
- `float DistanceToNearestSegment( Vec2D aPoint ) const` — min point-to-segment
  distance over all segments.
- `float RangeAlongRay( Vec2D aOrigin, float aAngle, float aMaxRange ) const` —
  nearest ray/segment hit from `aOrigin` along `aAngle`, capped at `aMaxRange`.
- `virtual void Draw( CRender& arRender ) const = 0;` — makes the class
  abstract; each subclass draws its own chain with its own thickness/colour
  via the protected helper below.
- `void DrawSegments( CRender& arRender, float aThickness, Color aColour ) const`
  — protected; the shared draw loop.
- `virtual ~CLoopShape() = default;`
- Private statics moved verbatim from `CRoom`: `DistancePointToSegment`,
  `RayIntersectSegment`.
- Private data: `CLoopReader mLoopReader;`, `std::vector<CSegment> mSegments;`.

### CRoom : CLoopShape (shrinks)

- `bool IsColliding( Vec2D aPosition, float aRadius ) const` —
  `DistanceToNearestSegment( aPosition ) < aRadius`.
- `void Draw( CRender& arRender ) const override` — `DrawSegments` with the
  existing wall thickness (`2.0f`) and colour (`RAYWHITE`).
- Loses `LoadFromFile`, `GetStartPose`, `RangeToWall`, both geometry statics,
  `CWallSegment`, `mLoopReader`, `mWalls` — all now in the base.
- `RangeToWall` callers switch to the inherited `RangeAlongRay` (only
  `CRangeSensor` calls it; see below).

### CFloorLine : CLoopShape (new)

- `static const float kHalfWidth = 2.5f;` — the line is 5 units wide for
  sensing.
- `bool IsLineUnder( Vec2D aPoint ) const` —
  `DistanceToNearestSegment( aPoint ) <= kHalfWidth`.
- `void Draw( CRender& arRender ) const override` — `DrawSegments` at ~`5.0f`
  thickness in a colour distinct from the walls and both robot bodies
  (proposed `ORANGE`).

### CRangeSensor (one-line change)

`Sense` currently calls `arRoom.RangeToWall(...)`. Retarget to
`arRoom.RangeAlongRay(...)` (same arguments, inherited from `CLoopShape`).
Signature and everything else unchanged.

### CLineSensor (new, standalone)

Deliberately not sharing a base with `CRangeSensor`: it is mounted by
position, not angle, and returns `bool`, not a distance.

- `CLineSensor( float aForwardOffset, float aLateralOffset )` — mount point in
  the robot frame: `aForwardOffset` along the heading, `aLateralOffset` to the
  robot's right (heading + 90 degrees). Both `const` members.
- `bool Sense( const CPose& arRobotPose, const CFloorLine& arLine ) const`:
  - world point =
    `pos + forward*(cos h, sin h) + lateral*(-sin h, cos h)`
  - return `arLine.IsLineUnder( worldPoint )`.

### CRobot (new, abstract base — the common half of A1's robot)

Owns pose, drive, trail, counts, collision handling, drawing, and the
`Update()` template method. Sensing and steering are the only hooks.

- `CRobot( const CPose& arStartPose, const CRoom& arRoom, Color aBodyColour )`.
- `void Update()` — the A1 sequence, unchanged in behaviour:
  1. `Sense();` (pure virtual)
  2. `SteerFromSensors();` (pure virtual — sets wheel speeds)
  3. `CPose Tentative = mDriveTrain.Advance( mPose, mTimeStep );`
  4. `if( HasCollided( Tentative ) )` → keep only `Tentative.mHeading`,
     edge-triggered collision count + console line (`mWasColliding`).
     `else` → accept `Tentative`, clear `mWasColliding`.
  5. `mTrail.push_back( mPose.mPosition ); ++mUpdateCount;`
- `void Draw( CRender& arRender ) const` — trail (`GRAY`), body circle in
  `mBodyColour`, heading line (`BLACK`). Moved verbatim from A1 with the
  colour taken from the member.
- `int GetUpdateCount() const;  int GetCollisionCount() const;`
- `virtual std::string Name() const = 0;` — for the run summary line.
- `virtual ~CRobot() = default;`
- Pure virtual hooks: `virtual void Sense() = 0;`,
  `virtual void SteerFromSensors() = 0;`
- Protected, for derived classes only, each commented:
  - `const CPose& Pose() const` — sensors need the current pose.
  - `const CRoom& Room() const` — the wall follower's range sensors sense
    against it.
  - `void SetWheelSpeeds( float aLeft, float aRight )` — forwards to the
    drive train; the only way a subclass drives.
  - `float BaseSpeed() const` — the straight-line wheel speed both laws build
    on.
- Private: `mPose`, `mDriveTrain`, `mTrail`, `mUpdateCount`,
  `mCollisionCount`, `mWasColliding`, `mBodyColour`, `const CRoom& mrRoom`,
  and the geometry/tuning consts `mRadius (15)`, `mAxleWidth`, `mTimeStep`.
  `bool HasCollided( const CPose& ) const` → `mrRoom.IsColliding(...)`.

### CWallFollowerRobot : CRobot (the wall-specific half of A1)

- Adds `CRangeSensor mSideSensor` (90 deg), `CRangeSensor mForwardSensor`
  (45 deg), `float mSideReading`, `float mForwardReading`, and the tuning
  consts `mMaxSensorRange`, `mTargetWallDistance`, `mSteeringGain`,
  `mCornerThreshold`, `mCornerGain` (all values unchanged from A1).
- `void Sense() override` — fill the two readings from the sensors against
  `Room()`.
- `void SteerFromSensors() override` — the A1 two-mode proportional law
  verbatim, ending in `SetWheelSpeeds( Left, Right )`.
- `std::string Name() const override` → `"Wall follower"`.
- Body colour `SKYBLUE` (as A1).

### CLineFollowerRobot : CRobot (new)

- Adds `CLineSensor mOnLineSensor` (forward = `mRadius`, lateral = `0`: over
  the line) and `CLineSensor mOffLineSensor` (forward = `mRadius`, lateral =
  `+kSensorOffset`: to the robot's right, beside the line), `bool mOnLine`,
  `bool mOffLine`, and tuning consts `kSensorOffset`, `kSteerDelta`.
- `const CFloorLine& mrLine` — the line it follows (not owned).
- `void Sense() override` — `mOnLine = mOnLineSensor.Sense( Pose(), mrLine );`
  and likewise `mOffLine`.
- `void SteerFromSensors() override` — bang-bang around `BaseSpeed()`, keeping
  the line just to the robot's right:
  - `mOnLine && !mOffLine` → centred → straight.
  - `mOnLine && mOffLine` → line curving right → steer right by `kSteerDelta`.
  - `!mOnLine && mOffLine` → drifted left, line now only under the side
    sensor → steer right harder.
  - `!mOnLine && !mOffLine` → line lost (overshot, or a sharp left corner) →
    steer left to bring it back.
  ends in `SetWheelSpeeds(...)`.
- `std::string Name() const override` → `"Line follower"`.
- Body colour distinct from the walls, the line and `SKYBLUE` (proposed
  `LIME`).
- The exact offsets, `kSteerDelta`, and the lost-line recovery are tuned
  during the build against `SimpleLine.map` — the concave corner is the hard
  case. Fallback if two prescribed sensors tune poorly: a symmetric straddle
  pair (`lateral = -d` and `+d`), still "one over, one beside" at 5-unit
  width, with a left/right turn when exactly one sees the line.

### CSimulation (new)

- `CSimulation()` — trivial; members not holding maps yet.
- `bool LoadMaps( const std::string& arWallsMap, const std::string& arLineMap )`
  — loads `mRoom` then `mLine`; on success constructs the two robots (each
  needs its shape's start pose) and pushes them into `mRobots`. Returns
  `false` if either load fails.
- `void Run( CRender& arRender )` — the main loop:
  `while( !arRender.WindowShouldClose() )` → `Update()` every robot →
  `BeginDrawing`, `mRoom.Draw`, `mLine.Draw`, `Draw()` every robot,
  `EndDrawing`. After the loop, `arRender.CloseWindow()` then print the
  summary: one line per robot with `Name()`, `GetUpdateCount()`,
  `GetCollisionCount()`.
- Data: `CRoom mRoom;`, `CFloorLine mLine;`,
  `std::vector<std::unique_ptr<CRobot>> mRobots;`.
- Robots are held as `unique_ptr<CRobot>` (not by value) so they are built
  only after the maps load, and so `Update`/`Draw`/summary iterate them
  polymorphically — the one place the `CRobot` base earns its keep at
  runtime, and the seam A5 ("20 of each") would widen.

### main.cpp (shrinks)

```
parse optional argv[1], argv[2]  (default SimpleWalls.map, SimpleLine.map)
CSimulation Sim;
if( !Sim.LoadMaps( WallsMap, LineMap ) ) { message; return 1; }
CRender Render;                       // window stays owned by main, as in A1
Sim.Run( Render );
return 0;
```

`CRender` stays in `main` (not `CSimulation`) so the simulation is
constructible without opening a window.

## Data flow (one frame)

```
CSimulation::Run
  for each robot:
    robot.Update()
      Sense()              wall: 2 rays vs CRoom     line: 2 points vs CFloorLine
      SteerFromSensors()   -> SetWheelSpeeds -> CDriveTrain
      CDriveTrain::Advance -> tentative CPose
      CRoom::IsColliding?  -> accept pose, or keep heading only + count
      trail.push_back, ++updateCount
  render: CRoom::Draw, CFloorLine::Draw, each robot.Draw
after window closes: per-robot summary to console
```

## Error handling

- Map open/parse failure: `CLoopReader` already prints the offending line;
  `CLoopShape::LoadFromFile` returns `false`; `CSimulation::LoadMaps` returns
  `false`; `main` prints which map failed and returns 1. No window opens.
- Empty vertex list: treated as a load failure (message + `false`), matching
  A1's `CRoom` behaviour.
- Lost sensor reading: range sensor returns `mMaxSensorRange` and the A1 law
  clamps the steering error; line sensor returns `false` and the "line lost"
  branch of the bang-bang law recovers.
- Zero-length segment: the geometry helpers already guard with an epsilon —
  unchanged.
- `SetWheelSpeeds` is the only drive path; `Advance` stays a pure function, so
  a rejected move cannot corrupt state.

## Testing

The core is the geometry in `CLoopShape` and the coordinate maths in
`CLineSensor` — both pure and cheap to check in isolation.

- `TestGeometry.cpp` (a standalone `main`, not linked into `WallFollower`,
  mirroring the provided `TestRender.cpp`):
  - `CLoopShape::DistanceToNearestSegment` — `0` on a vertex, half the gap
    midway between two parallel segments, correct for a point off the end of a
    segment.
  - `RangeAlongRay` — hits the expected wall for an axis-aligned ray, returns
    `aMaxRange` for a ray pointing away.
  - `CLineSensor::Sense` — `true` for an offset that lands on a known line
    vertex, `false` for one that lands well outside, with the robot at a
    non-zero heading (checks the rotation).
- Visual acceptance (the assignment's own bar), from a normal run:
  - both trails close their loops;
  - wall follower ends with <= 10 collisions (0 is the target);
  - the end-of-run screenshot shows both trails complete;
  - console shows the per-robot summary and any collisions.

## Build

Standard command gains the new `.cpp` files:

```
g++ -Wall -Wextra -std=c++17 \
    main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp \
    CRangeSensor.cpp CLineSensor.cpp CDriveTrain.cpp \
    CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp \
    CLoopReader.cpp CRender.cpp \
    -lraylib -o WallFollower
```

(Windows/MSYS2 adds `-I/c/msys64/ucrt64/include -L/c/msys64/ucrt64/lib
-lopengl32 -lgdi32 -lwinmm`, as in the README.)

## Repository layout — decided

Two self-contained folders:

- `A1/` — the wall follower frozen as submitted for A1 (extracted from git tag
  `a1-baseline`, plus the `M_PI` → `kPi` portability fix so it builds under the
  standard `-std=c++17` command).
- `A2/` — the refactored two-robot program (all files above).

Vendored raylib stays at the repo root as shared reference; the build commands
do not reference it (they assume an installed raylib, as the assignment does),
so no `-I../` is needed. `docs/` and this spec stay at the root.

## Out of scope

- A5 noise bonus — a separate copy of the finished A2, not designed for here.
- Any interaction, avoidance or awareness between the two robots.
- Changing the `.map` file format or `CLoopReader`.
```
