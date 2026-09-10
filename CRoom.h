//-----------------------------------------------------------------------------
// CRoom.h
//
// The room the robots drive in: a closed loop of straight walls, loaded from a
// loop file. CRoom is a CLoopShape (a room is a closed loop of segments) and
// adds only the wall-specific meaning on top of the shared geometry:
//
//   - "Am I touching a wall?" (IsColliding)
//   - how a wall is drawn.
//
// Everything else - loading, the start pose, ray casts, point-distance queries
// - comes from CLoopShape.
//-----------------------------------------------------------------------------

#ifndef CROOM_H
#define CROOM_H

#include "CLoopShape.h"
#include "CRender.h"   // Vec2D, CRender

//-----------------------------------------------------------------------------
class CRoom : public CLoopShape
{
    public:
        //---Collision---
        // True if a disc of radius aRadius centred at aPosition overlaps any
        // wall of the room.
        bool IsColliding( Vec2D aPosition, float aRadius ) const;

        //---Rendering---
        void Draw( CRender& arRender ) const override;
};

#endif
