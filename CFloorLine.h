//-----------------------------------------------------------------------------
// CFloorLine.h
//
// The line marked on the floor for the line-following robot to follow: a
// closed loop of straight segments loaded from a loop file. CFloorLine is a
// CLoopShape (the line is a closed loop of segments) and adds only the line-
// specific meaning:
//
//   - "Is the floor at this point part of the line?" (IsLineUnder)
//   - how the line is drawn.
//
// The line is treated as 5 units wide for sensing, so a point counts as "on
// the line" when it is within half that width of a segment.
//-----------------------------------------------------------------------------

#ifndef CFLOORLINE_H
#define CFLOORLINE_H

#include "CLoopShape.h"
#include "CRender.h"   // Vec2D, CRender

//-----------------------------------------------------------------------------
class CFloorLine : public CLoopShape
{
    public:
        //---Sensing---
        // True if aPoint lies on the painted line (within half the line width
        // of a segment).
        bool IsLineUnder( Vec2D aPoint ) const;

        //---Rendering---
        void Draw( CRender& arRender ) const override;

    private:
        static const float kHalfWidth;      // 2.5: the line is 5 units wide
        static const float kDrawThickness;  // rendered stroke width
};

#endif
