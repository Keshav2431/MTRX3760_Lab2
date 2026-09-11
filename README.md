# MTRX3760 Lab 2 — Robot Simulator

A simple robot simulator in C++, using raylib. A wall follower and a line
follower drive themselves around a room using only their own sensors, no
fixed path. A5 is a noise bonus: a copy of A2 with a bit of randomness added,
running 20 robots of each type at once.

## Files

| Folder | Description |
|---|---|
| `A1/` | Wall follower only |
| `A2/` | Wall follower + line follower, running together |
| `A5/` | Noise bonus — a copy of A2 with random noise and 20 robots of each type |
| `docs/` | Design notes, UML, and the report's code appendix |

## Build

Each folder is a full, separate program. raylib must be installed first. From
inside the folder:

```
# A1
g++ -Wall -Wextra -std=c++17 main.cpp CRoom.cpp CRangeSensor.cpp CDriveTrain.cpp CWallFollowerRobot.cpp CLoopReader.cpp CRender.cpp -lraylib -o WallFollower

# A2
g++ -Wall -Wextra -std=c++17 main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp CRangeSensor.cpp CLineSensor.cpp CDriveTrain.cpp CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp CLoopReader.cpp CRender.cpp -lraylib -o WallFollower

# A5
g++ -Wall -Wextra -std=c++17 main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp CRangeSensor.cpp CLineSensor.cpp CNoiseSource.cpp CDriveTrain.cpp CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp CLoopReader.cpp CRender.cpp -lraylib -o NoiseBonus
```

## Run

```
./WallFollower       # A1 or A2
./NoiseBonus          # A5
```

A1 and A2 run until the window is closed. A5 stops itself once every robot
has finished its lap.

## Example Output

```
--- Run summary ---
Wall follower: updates completed 1842, total collisions 0
Line follower: updates completed 1842, total collisions 0
```
