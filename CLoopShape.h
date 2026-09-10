//-----------------------------------------------------------------------------
// CLoopShape.h
//
// A closed loop of straight segments, loaded from a .map loop file via
// CLoopReader. It answers the geometric questions any robot asks of a loop
// in this simulator:
//
//   - "How far is the nearest segment in this direction?" (RangeAlongRay)
//   - "How far is the nearest segment from this point?"   (DistanceToNearestSegment)
//
// CRoom (walls) and CFloorLine (a floor line) both derive from it: a room and
// a line are both closed loops of segments, and only the meaning laid on top
// of the geometry differs (a wall you collide with, a line you sense).
//
// The class is abstract: Draw() is pure virtual, so every concrete loop
// chooses its own line thickness and colour via DrawSegments().
//-----------------------------------------------------------------------------

#ifndef CLOOPSHAPE_H
#define CLOOPSHAPE_H

#include "CRender.h"       // Vec2D, Color, CRender
#include "CLoopReader.h"   // CPose, CLoopReader

#include <string>
#include <vector>

//-----------------------------------------------------------------------------
class CLoopShape
{
    public:
        //---Ctor/Dtor---
        CLoopShape() = default;
        virtual ~CLoopShape() = default;

        //---Loading---
        // Reads the loop file and builds the closed segment chain (the last
        // vertex joins back to the first). False on open/parse failure or an
        // empty vertex list.
        bool LoadFromFile( const std::string& arFilename );

        //---Access---
        const CPose& GetStartPose() const;

        //---Geometry---
        // Shortest distance from aPoint to any segment of the loop.
        float DistanceToNearestSegment( Vec2D aPoint ) const;

        // Distance from aOrigin along aAngle (radians, 0 = +x, increasing
        // clockwise) to the nearest segment it meets, capped at aMaxRange.
        float RangeAlongRay( Vec2D aOrigin, float aAngle, float aMaxRange ) const;

        //---Rendering---
        virtual void Draw( CRender& arRender ) const = 0;

    protected:
        // The shared draw loop; each subclass calls this with its own style.
        void DrawSegments( CRender& arRender, float aThickness, Color aColour ) const;

    private:
        //---One straight segment, from mStart to mEnd---
        struct CSegment
        {
            Vec2D mStart;
            Vec2D mEnd;
        };

        //---Geometry helpers (moved from A1 CRoom)---
        static float DistancePointToSegment( Vec2D aPoint, const CSegment& arSegment );
        static float RayIntersectSegment( Vec2D aOrigin, Vec2D aDirection, const CSegment& arSegment );

        //---The loop file this shape was built from---
        CLoopReader mLoopReader;

        //---The shape's segments, one per edge of the loop---
        std::vector<CSegment> mSegments;
};

#endif
