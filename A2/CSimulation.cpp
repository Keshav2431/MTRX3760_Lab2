//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// LoadMaps loads the room, then the line, and only builds the two robots once
// both have loaded successfully. Run is the same fixed-step loop A1's main()
// used to run, just moved in here, now walking a list of robots and drawing
// the line as well as the room.
//-----------------------------------------------------------------------------

#include "CSimulation.h"
#include "CRender.h"
#include "CWallFollowerRobot.h"
#include "CLineFollowerRobot.h"

#include <iostream>

//-----------------------------------------------------------------------------
// Nothing needs setting up yet - the room, the line and the robot list all
// start out empty, and the maps are loaded afterwards, in LoadMaps().
//-----------------------------------------------------------------------------
CSimulation::CSimulation()
{
}


//-----------------------------------------------------------------------------
// Loads the room, then the line, stopping early if either one fails. Only
// once both are in place does it build the two robots, since each robot needs
// a start pose that comes from its own map.
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
// Keeps stepping and redrawing every robot for as long as the window stays
// open, then prints the summary once the window is closed.
void CSimulation::Run( CRender& arRender )
{
    while( !arRender.WindowShouldClose() )
    {
        UpdateRobots();

        arRender.BeginDrawing();
        DrawFrame( arRender );
        arRender.EndDrawing();
    }

    arRender.CloseWindow();
    PrintSummary();
}


//-----------------------------------------------------------------------------
void CSimulation::UpdateRobots()
{
    for( const std::unique_ptr<CRobot>& rRobot : mRobots )
    {
        rRobot->Update();
    }
}


//-----------------------------------------------------------------------------
void CSimulation::DrawFrame( CRender& arRender ) const
{
    mRoom.Draw( arRender );
    mLine.Draw( arRender );

    for( const std::unique_ptr<CRobot>& rRobot : mRobots )
    {
        rRobot->Draw( arRender );
    }
}


//-----------------------------------------------------------------------------
void CSimulation::PrintSummary() const
{
    std::cout << "--- Run summary ---" << std::endl;

    for( const std::unique_ptr<CRobot>& rRobot : mRobots )
    {
        std::cout << rRobot->Name()
                  << ": updates completed " << rRobot->GetUpdateCount()
                  << ", total collisions " << rRobot->GetCollisionCount() << std::endl;
    }
}
