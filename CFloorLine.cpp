//-----------------------------------------------------------------------------
// CFloorLine.cpp
//
// Implements CFloorLine. Like CRoom, both members sit one line on top of
// CLoopShape: a point is "on the line" when the nearest segment is within
// half the line width, and drawing is the shared segment loop in the line
// style.
//-----------------------------------------------------------------------------

#include "CFloorLine.h"

//---The line is 5 units wide for sensing; a point within 2.5 of a segment is
//   on it. Drawn a little wider so it reads clearly under the robot.--------
const float CFloorLine::kHalfWidth     = 2.5f;
const float CFloorLine::kDrawThickness = 5.0f;

//-----------------------------------------------------------------------------
bool CFloorLine::IsLineUnder( Vec2D aPoint ) const
{
    return DistanceToNearestSegment( aPoint ) <= kHalfWidth;
}


//-----------------------------------------------------------------------------
void CFloorLine::Draw( CRender& arRender ) const
{
    DrawSegments( arRender, kDrawThickness, ORANGE );
}
