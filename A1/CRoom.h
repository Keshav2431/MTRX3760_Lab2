//-----------------------------------------------------------------------------
// CRoom.h
//
// A room made of straight walls, loaded from a loop file via CLoopReader.
// CRoom turns the loop's vertices into a closed chain of wall segments, and
// answers the two questions a robot needs of its environment:
//
//   - "How far away is the nearest wall in this direction?" (RangeToWall)
//   - "Am I touching a wall?"                                (IsColliding)
//
// It also knows how to draw itself. Nothing outside CRoom needs to know that
// a room is really a sequence of line segments; that is the whole point of
// wrapping it up here rather than leaving CLoopReader's vertex list exposed.
//-----------------------------------------------------------------------------

#ifndef CROOM_H
#define CROOM_H

#include "CRender.h"       // for Vec2D
#include "CLoopReader.h"   // for CPose

#include <string>
#include <vector>

//-----------------------------------------------------------------------------
class CRoom
{
    public:
        //---Loading---
        bool LoadFromFile( const std::string& arFilename );

        //---Access---
        const CPose& GetStartPose() const;

        //---Sensing---
        // Casts a ray from aOrigin in direction aAngle (radians, 0 = +x,
        // increasing clockwise) and returns the distance to the nearest wall
        // it meets, or aMaxRange if no wall is within that range.
        float RangeToWall( Vec2D aOrigin, float aAngle, float aMaxRange ) const;

        //---Collision---
        // True if a disc of radius aRadius centred at aPosition overlaps any
        // wall of the room.
        bool IsColliding( Vec2D aPosition, float aRadius ) const;

        //---Rendering---
        void Draw( CRender& arRender ) const;

    private:
        //---One straight wall, from mStart to mEnd---
        struct CWallSegment
        {
            Vec2D mStart;
            Vec2D mEnd;
        };

        //---Geometry helpers---
        static float DistancePointToSegment( Vec2D aPoint, const CWallSegment& arSegment );
        static float RayIntersectSegment( Vec2D aOrigin, Vec2D aDirection, const CWallSegment& arSegment );

        //---The loop file this room was built from---
        CLoopReader mLoopReader;

        //---The room's walls, one per edge of the loop---
        std::vector<CWallSegment> mWalls;
};

#endif
