//-----------------------------------------------------------------------------
// main.cpp
//
// MTRX3760 Lab 2, A2: Wall Follower + Line Follower.
//
// Builds a CSimulation, loads the two loop files (SimpleWalls.map and
// SimpleLine.map by default), and runs the fixed simulated time step loop
// until the window closes. Both robots run at once and do not interact.
// Close the window to end the run and print the per-robot summary.
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
int main( int argc, char* argv[] )
{
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
        return 1;
    }

    CRender Render;
    Simulation.Run( Render );

    return 0;
}
