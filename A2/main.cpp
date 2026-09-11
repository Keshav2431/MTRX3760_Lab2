//-----------------------------------------------------------------------------
// main.cpp
//
// MTRX3760 Lab 2, A2: Wall Follower + Line Follower.
//
// Builds a CSimulation, loads the two map files (SimpleWalls.map and
// SimpleLine.map by default), and runs the fixed-timestep loop until the
// window is closed. Both robots run side by side and never interact.
// Close the window to end the run and print the summary for each robot.
//
//
// BUILDING, IF YOU INSTALLED RAYLIB WITH APT (on your own machine)
//
//     g++ -Wall -Wextra -std=c++17 main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp CRangeSensor.cpp CLineSensor.cpp CDriveTrain.cpp CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp CLoopReader.cpp CRender.cpp -lraylib -o WallFollower
//
//
// BUILDING, IF YOU BUILT RAYLIB FROM SOURCE (on a lab machine)
//
//     g++ -Wall -Wextra -std=c++17 main.cpp CSimulation.cpp CLoopShape.cpp CRoom.cpp CFloorLine.cpp CRangeSensor.cpp CLineSensor.cpp CDriveTrain.cpp CRobot.cpp CWallFollowerRobot.cpp CLineFollowerRobot.cpp CLoopReader.cpp CRender.cpp -I$HOME/raylib/include -L$HOME/raylib/lib -lraylib -o WallFollower
//
//
// RUNNING
//
//     ./WallFollower
//     ./WallFollower SimpleWalls.map SimpleLine.map
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
