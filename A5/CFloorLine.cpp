//-----------------------------------------------------------------------------
// CFloorLine.cpp
//
// Both of these are one small step on top of CLoopShape: a point is on the
// line when the nearest segment is within half the line's width, and drawing
// the line is just drawing the shared segment loop in the line's own colour.
//-----------------------------------------------------------------------------

#include "CFloorLine.h"

//---The spec fixes the line at 5 units wide for sensing. It is also drawn at
//   that width, so the picture on screen matches what the sensors see.------
const float CFloorLine::kLineWidth     = 5.0f;
const float CFloorLine::kHalfWidth     = kLineWidth / 2.0f;
const float CFloorLine::kDrawThickness = kLineWidth;

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
