//-----------------------------------------------------------------------------
// main.cpp
//
// MTRX3760 Lab 2, A5: Noise Bonus.
//
// A copy of A2 - the wall follower and the line follower running side by side
// - with some randomness added in, in the three places the brief asks for:
//
//   - a small random shift to each robot's starting position and heading,
//   - a small random slip in how far each wheel actually turns each step, and
//   - twenty of each kind of robot running at the same time.
//
// All of the randomness comes from one CNoiseSource, owned by CSimulation and
// shared with every robot, so one seed decides the entire run. Neither robot's
// steering rule is changed from A2 - each one still steers purely on what its
// own sensors tell it, and that's exactly what lets a noisy robot find its way
// around anyway.
//
// The run stops itself once every robot has finished its lap (or a step limit
// is reached), prints its summary, and keeps showing the finished picture on
// screen, so the screenshot and the console output can both be captured at
// the same time. Close the window to quit.
//
//
// BUILDING, IF YOU INSTALLED RAYLIB WITH APT (on your own machine)
//
//     g++ -Wall -Wextra -std=c++17 main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp CRangeSensor.cpp CLineSensor.cpp CNoiseSource.cpp CDriveTrain.cpp CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp CLoopReader.cpp CRender.cpp -lraylib -o NoiseBonus
//
//
// BUILDING, IF YOU BUILT RAYLIB FROM SOURCE (on a lab machine)
//
//     g++ -Wall -Wextra -std=c++17 main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp CRangeSensor.cpp CLineSensor.cpp CNoiseSource.cpp CDriveTrain.cpp CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp CLoopReader.cpp CRender.cpp -I$HOME/raylib/include -L$HOME/raylib/lib -lraylib -o NoiseBonus
//
//
// RUNNING
//
//     ./NoiseBonus
//     ./NoiseBonus SimpleWalls.map SimpleLine.map
//-----------------------------------------------------------------------------

#include "CRender.h"
#include "CSimulation.h"

#include <iostream>
#include <string>

//-----------------------------------------------------------------------------
// Reads the map filenames off the command line, falling back to the two
// defaults if none were given, then builds and runs the simulation.
int main( int argc, char* argv[] )
{
    int ExitCode = 0;

    std::string WallsMapFilename = "SimpleWalls.map";
    std::string LineMapFilename = "SimpleLine.map";

    if( argc > 1 )
    {
        WallsMapFilename = argv[1];
    }
    if( argc > 2 )
    {
        LineMapFilename = argv[2];
    }

    CSimulation Simulation;

    if( !Simulation.LoadMaps( WallsMapFilename, LineMapFilename ) )
    {
        std::cout << "Could not load maps ('" << WallsMapFilename
                  << "', '" << LineMapFilename << "')" << std::endl;
        ExitCode = 1;
    }
    else
    {
        CRender Render;
        Simulation.Run( Render );
    }

    return ExitCode;
}
