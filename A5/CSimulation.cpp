//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// LoadMaps loads the room, then the line, and only builds the two groups of
// robots once both have loaded successfully. Run steps both groups forward
// every frame until every robot has finished (or the step limit is reached),
// then stops stepping and just keeps redrawing the finished picture.
//-----------------------------------------------------------------------------

#include "CSimulation.h"
#include "CRender.h"
#include "CWallFollowerRobot.h"
#include "CLineFollowerRobot.h"

#include <iostream>

//---Twenty of each robot type, as the brief asks for.-----------------------
const int CSimulation::kRobotsPerType = 20;

//---A lap of either loop takes well under a thousand steps, so this leaves
//   plenty of room to spare. It only gets reached by a robot that has been
//   thrown so far off course it never makes it back, and it stops that one
//   robot from holding up the other thirty-nine forever.--------------------
const int CSimulation::kMaxSteps = 2500;

//---Any fixed number would do here; what matters is that it's fixed, so the
//   report screenshot can be produced again later. A different seed just
//   gives a different, equally fair, spread of paths.------------------------
const unsigned int CSimulation::kNoiseSeed = 20260911u;

//---The last robot in a group is lightened this far towards white; the first
//   one isn't lightened at all. Kept below 1 so even the palest robot is
//   still clearly its own family colour.-------------------------------------
const float CSimulation::kMaxLighten = 0.7f;


//-----------------------------------------------------------------------------
// Builds the noise source using the fixed seed. The room, the line, and the
// two robot groups all start out empty and are filled in once the maps load.
//-----------------------------------------------------------------------------
CSimulation::CSimulation()
    :
        mNoise( kNoiseSeed ),
        mStepCount( 0 ),
        mSummaryPrinted( false )
{
}


//-----------------------------------------------------------------------------
// Loads the room, then the line, stopping early if either one fails. Only
// once both are in place does it build the robots, since each one needs a
// start pose that comes from its own map.
bool CSimulation::LoadMaps( const std::string& arWallsMap, const std::string& arLineMap )
{
    bool Loaded = mRoom.LoadFromFile( arWallsMap );

    if( Loaded )
    {
        Loaded = mLine.LoadFromFile( arLineMap );
    }

    if( Loaded )
    {
        BuildRobots();
    }

    return Loaded;
}


//-----------------------------------------------------------------------------
void CSimulation::BuildRobots()
{
    mWallFollowers.clear();
    mLineFollowers.clear();

    for( int Index = 0; Index < kRobotsPerType; ++Index )
    {
        float Shade = ShadeForRobot( Index );

        // Each robot gets the map's start pose with its own random scatter
        // added, so the twenty robots don't all start stacked on one spot.
        CPose WallStart = mNoise.ScatterStartPose( mRoom.GetStartPose() );
        CPose LineStart = mNoise.ScatterStartPose( mLine.GetStartPose() );

        mWallFollowers.push_back(
            std::make_unique<CWallFollowerRobot>( WallStart, mRoom, mNoise, Shade ) );

        mLineFollowers.push_back(
            std::make_unique<CLineFollowerRobot>( LineStart, mRoom, mLine, mNoise, Shade ) );
    }
}


//-----------------------------------------------------------------------------
float CSimulation::ShadeForRobot( int aIndex ) const
{
    // Spreads the group evenly: 0 for the first robot, kMaxLighten for the
    // last one, and everything in between spaced out evenly.
    float Position = static_cast<float>( aIndex ) / static_cast<float>( kRobotsPerType - 1 );

    return Position * kMaxLighten;
}


//-----------------------------------------------------------------------------
// Keeps the window open and redrawing every frame. While the run isn't
// finished yet, each frame also steps both groups of robots forward once.
// Once it is finished, stepping stops but the last frame keeps being redrawn,
// which is what holds the finished picture on screen, and the summary gets
// printed the moment that first happens.
void CSimulation::Run( CRender& arRender )
{
    mNoise.PrintNoiseLevels();

    while( !arRender.WindowShouldClose() )
    {
        if( RunFinished() )
        {
            PrintSummaryOnce();
        }
        else
        {
            StepGroup( mWallFollowers );
            StepGroup( mLineFollowers );
            ++mStepCount;
        }

        arRender.BeginDrawing();
        DrawFrame( arRender );
        arRender.EndDrawing();
    }

    arRender.CloseWindow();

    // Only prints here if the window was closed before the run finished
    // itself - otherwise the summary has already been printed above.
    PrintSummaryOnce();
}


//-----------------------------------------------------------------------------
void CSimulation::StepGroup( CRobotGroup& arGroup )
{
    for( const std::unique_ptr<CRobot>& rRobot : arGroup )
    {
        rRobot->Update();
    }
}


//-----------------------------------------------------------------------------
void CSimulation::DrawFrame( CRender& arRender ) const
{
    mRoom.Draw( arRender );
    mLine.Draw( arRender );

    // Every trail is drawn first, then every body, across both groups, so no
    // robot's body ends up hidden underneath a trail drawn afterwards.
    DrawTrails( mWallFollowers, arRender );
    DrawTrails( mLineFollowers, arRender );

    DrawBodies( mWallFollowers, arRender );
    DrawBodies( mLineFollowers, arRender );
}


//-----------------------------------------------------------------------------
void CSimulation::DrawTrails( const CRobotGroup& arGroup, CRender& arRender ) const
{
    for( const std::unique_ptr<CRobot>& rRobot : arGroup )
    {
        rRobot->DrawTrail( arRender );
    }
}


//-----------------------------------------------------------------------------
void CSimulation::DrawBodies( const CRobotGroup& arGroup, CRender& arRender ) const
{
    for( const std::unique_ptr<CRobot>& rRobot : arGroup )
    {
        rRobot->DrawBody( arRender );
    }
}


//-----------------------------------------------------------------------------
bool CSimulation::RunFinished() const
{
    bool BothGroupsDone = GroupFinished( mWallFollowers ) && GroupFinished( mLineFollowers );

    return BothGroupsDone || ( mStepCount >= kMaxSteps );
}


//-----------------------------------------------------------------------------
bool CSimulation::GroupFinished( const CRobotGroup& arGroup ) const
{
    return CountFinished( arGroup ) == static_cast<int>( arGroup.size() );
}


//-----------------------------------------------------------------------------
int CSimulation::CountFinished( const CRobotGroup& arGroup ) const
{
    int Finished = 0;

    for( const std::unique_ptr<CRobot>& rRobot : arGroup )
    {
        if( rRobot->HasFinishedLap() )
        {
            ++Finished;
        }
    }

    return Finished;
}


//-----------------------------------------------------------------------------
void CSimulation::ReportGroup( const CRobotGroup& arGroup ) const
{
    if( !arGroup.empty() )
    {
        int Collisions = 0;

        for( const std::unique_ptr<CRobot>& rRobot : arGroup )
        {
            Collisions += rRobot->GetCollisionCount();
        }

        std::cout << arGroup.front()->Name() << ": "
                  << CountFinished( arGroup ) << " of " << arGroup.size()
                  << " finished the loop, " << Collisions
                  << " wall collisions in total (not counted for this bonus)"
                  << std::endl;
    }
}


//-----------------------------------------------------------------------------
void CSimulation::PrintSummaryOnce()
{
    if( !mSummaryPrinted )
    {
        std::cout << "--- Run summary ---" << std::endl;
        std::cout << "Steps taken: " << mStepCount
                  << " of at most " << kMaxSteps << std::endl;

        ReportGroup( mWallFollowers );
        ReportGroup( mLineFollowers );

        mSummaryPrinted = true;
    }
}
