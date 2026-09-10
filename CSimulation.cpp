//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Implements CSimulation. LoadMaps loads the room then the line and, only if
// both succeed, builds the two robots from their start poses. Run is the
// fixed-step loop lifted from A1's main(), now iterating a list of robots and
// drawing the line as well as the room.
//-----------------------------------------------------------------------------

#include "CSimulation.h"
#include "CWallFollowerRobot.h"
#include "CLineFollowerRobot.h"

#include <iostream>

//-----------------------------------------------------------------------------
CSimulation::CSimulation()
{
}


//-----------------------------------------------------------------------------
bool CSimulation::LoadMaps( const std::string& arWallsMap, const std::string& arLineMap )
{
    bool Loaded = mRoom.LoadFromFile( arWallsMap );

    if( Loaded )
    {
        Loaded = mLine.LoadFromFile( arLineMap );
    }

    if( Loaded )
    {
        mRobots.clear();
        mRobots.push_back( std::make_unique<CWallFollowerRobot>( mRoom.GetStartPose(), mRoom ) );
        mRobots.push_back( std::make_unique<CLineFollowerRobot>( mLine.GetStartPose(), mRoom, mLine ) );
    }

    return Loaded;
}


//-----------------------------------------------------------------------------
void CSimulation::Run( CRender& arRender )
{
    while( !arRender.WindowShouldClose() )
    {
        for( const std::unique_ptr<CRobot>& rRobot : mRobots )
        {
            rRobot->Update();
        }

        arRender.BeginDrawing();
        mRoom.Draw( arRender );
        mLine.Draw( arRender );
        for( const std::unique_ptr<CRobot>& rRobot : mRobots )
        {
            rRobot->Draw( arRender );
        }
        arRender.EndDrawing();
    }

    arRender.CloseWindow();

    std::cout << "--- Run summary ---" << std::endl;
    for( const std::unique_ptr<CRobot>& rRobot : mRobots )
    {
        std::cout << rRobot->Name()
                  << ": updates completed " << rRobot->GetUpdateCount()
                  << ", total collisions " << rRobot->GetCollisionCount() << std::endl;
    }
}
