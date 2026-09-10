//-----------------------------------------------------------------------------
// CRoom.cpp
//
// Implements CRoom. Both members are one line on top of CLoopShape: a
// collision is "the nearest wall is closer than the robot's radius", and
// drawing is the shared segment loop in the wall style.
//-----------------------------------------------------------------------------

#include "CRoom.h"

//-----------------------------------------------------------------------------
bool CRoom::IsColliding( Vec2D aPosition, float aRadius ) const
{
    return DistanceToNearestSegment( aPosition ) < aRadius;
}


//-----------------------------------------------------------------------------
void CRoom::Draw( CRender& arRender ) const
{
    const float WallThickness = 2.0f;

    DrawSegments( arRender, WallThickness, RAYWHITE );
}
