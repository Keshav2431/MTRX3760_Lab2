//-----------------------------------------------------------------------------
// CLoopShape.h
//
// A CLoopShape is a closed loop made of straight segments, read in from a .map
// file. It can answer two questions any robot might ask about a loop: how far
// away is the nearest segment in a given direction, and how far away is the
// nearest segment from a given point.
//
// CRoom (the walls) and CFloorLine (the floor line) are both loops of
// segments, so they both inherit from CLoopShape. The only difference is what
// each one means: a wall the robot bumps into, or a line the robot senses.
// Draw() is left for each of them to fill in, so each can pick its own colour
// and thickness.
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
        CLoopShape();
        virtual ~CLoopShape();

        //---Loading---
        // Reads the loop file and builds the closed chain of segments (the
        // last vertex joins back up to the first). Returns false if the file
        // can't be opened or read, or if it has no vertices in it.
        bool LoadFromFile( const std::string& arFilename );

        //---Access---
        const CPose& GetStartPose() const;   // the start pose read from the file

        //---Geometry---
        // The shortest distance from aPoint to any segment of the loop.
        float DistanceToNearestSegment( Vec2D aPoint ) const;

        // The distance from aOrigin, travelling along aAngle (in radians, 0
        // pointing along +x, clockwise is positive), to the nearest segment
        // it reaches - capped at aMaxRange if nothing is found sooner.
        float RangeAlongRay( Vec2D aOrigin, float aAngle, float aMaxRange ) const;

        //---Rendering---
        // Draws the loop. Each kind of loop decides its own colour and width.
        virtual void Draw( CRender& arRender ) const = 0;

    protected:
        // Draws every segment at one colour and thickness. A subclass calls
        // this from its own Draw().
        void DrawSegments( CRender& arRender, float aThickness, Color aColour ) const;

    private:
        //---One straight segment of the loop, from mStart to mEnd---
        struct CSegment
        {
            Vec2D mStart;
            Vec2D mEnd;
        };

        //---Turns the loop's vertices into the closed chain kept in mSegments---
        void BuildSegments( const std::vector<Vec2D>& arVertices );

        //---Small geometry helpers used by the two public functions above---
        static float DistancePointToSegment( Vec2D aPoint, const CSegment& arSegment );
        static float RayIntersectSegment( Vec2D aOrigin, Vec2D aDirection, const CSegment& arSegment );

        CLoopReader mLoopReader;          // the file this shape was read from
        std::vector<CSegment> mSegments;  // one segment for each edge of the loop
};

#endif
