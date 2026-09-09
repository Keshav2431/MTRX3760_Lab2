//-----------------------------------------------------------------------------
// main.cpp
//
// MTRX3760 Lab 2, A1: Wall Follower.
//
// Loads a room from a loop file (SimpleWalls.map by default) and simulates a
// disc-shaped robot driving around it, following the wall on its right-hand
// side. The simulation advances by a fixed simulated time step each frame; it
// is not driven by real elapsed time. Close the window to end the run and
// print the summary.
//
//
// BUILDING, IF YOU INSTALLED RAYLIB WITH APT (on your own machine)
//
//     g++ -Wall -Wextra -std=c++17 main.cpp CRoom.cpp CRangeSensor.cpp CDriveTrain.cpp CWallFollowerRobot.cpp CLoopReader.cpp CRender.cpp -lraylib -o WallFollower
//
//
// BUILDING, IF YOU BUILT RAYLIB FROM SOURCE (on a lab machine)
//
//     g++ -Wall -Wextra -std=c++17 main.cpp CRoom.cpp CRangeSensor.cpp CDriveTrain.cpp CWallFollowerRobot.cpp CLoopReader.cpp CRender.cpp -I$HOME/raylib/include -L$HOME/raylib/lib -lraylib -o WallFollower
//
//
// RUNNING
//
//     ./WallFollower
//     ./WallFollower SimpleWalls.map
//-----------------------------------------------------------------------------

#include "CRender.h"
#include "CRoom.h"
#include "CWallFollowerRobot.h"

#include <iostream>
#include <string>

//-----------------------------------------------------------------------------
int main( int argc, char* argv[] )
{
    std::string MapFilename = "SimpleWalls.map";
    if( argc > 1 )
    {
        MapFilename = argv[1];
    }

    CRoom Room;
    bool RoomLoaded = Room.LoadFromFile( MapFilename );

    if( !RoomLoaded )
    {
        std::cout << "Could not load room from '" << MapFilename << "'" << std::endl;
        return 1;
    }

    CRender Render;
    CWallFollowerRobot Robot( Room.GetStartPose(), Room );

    while( !Render.WindowShouldClose() )
    {
        Robot.Update();

        Render.BeginDrawing();
        Room.Draw( Render );
        Robot.Draw( Render );
        Render.EndDrawing();
    }

    Render.CloseWindow();

    std::cout << "--- Run summary ---" << std::endl;
    std::cout << "Updates completed: " << Robot.GetUpdateCount() << std::endl;
    std::cout << "Total collisions:  " << Robot.GetCollisionCount() << std::endl;

    return 0;
}
