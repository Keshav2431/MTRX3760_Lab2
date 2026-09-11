//-----------------------------------------------------------------------------
// CFloorLine.h
//
// A CFloorLine is the line painted on the floor for the line-following robot
// to follow: a closed loop of straight segments, read from a .map file. It is
// a CLoopShape, and adds just two things on top - whether a given point sits
// on the paint, and how the line is drawn.
//
// The spec fixes the line at 5 units wide for sensing purposes, so a point
// counts as "on the line" when it is within half that width of a segment.
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
        // True if aPoint sits on the painted line (within half the line's
        // width of a segment).
        bool IsLineUnder( Vec2D aPoint ) const;

        //---Rendering---
        // Draws the line in its own colour and width.
        void Draw( CRender& arRender ) const override;

    private:
        static const float kLineWidth;      // how wide the painted line is, for sensing
        static const float kHalfWidth;      // kLineWidth / 2: closer than this counts as "on the line"
        static const float kDrawThickness;  // how wide the line is drawn on screen
};

#endif
