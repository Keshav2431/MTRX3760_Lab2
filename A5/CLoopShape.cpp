//-----------------------------------------------------------------------------
// CLoopShape.cpp
//
// LoadFromFile turns the vertices from the file into a closed chain of
// segments. DistanceToNearestSegment and RangeAlongRay both check every
// segment in turn and keep the best answer, using the two small geometry
// helpers at the bottom of the file to do the actual maths. All of this used
// to be part of CRoom in A1; it moved here so CFloorLine could use it too.
//-----------------------------------------------------------------------------

#include "CLoopShape.h"

#include <algorithm>
#include <cmath>
#include <iostream>

//-----------------------------------------------------------------------------
// There is nothing to set up yet: the file reader and the segment list start
// out empty, and LoadFromFile fills them in later. Written here rather than
// in the header, as usual.
//-----------------------------------------------------------------------------
CLoopShape::CLoopShape()
{
}


//-----------------------------------------------------------------------------
CLoopShape::~CLoopShape()
{
}


//-----------------------------------------------------------------------------
// Reads the file, checks it actually described a shape, and if so turns its
// vertices into segments. Fails and leaves the old shape (if any) alone
// otherwise.
bool CLoopShape::LoadFromFile( const std::string& arFilename )
{
    bool Success = mLoopReader.ReadFile( arFilename );

    if( Success && mLoopReader.GetVertices().empty() )
    {
        std::cout << "CLoopShape: '" << arFilename << "' contains no vertices" << std::endl;
        Success = false;
    }

    if( Success )
    {
        BuildSegments( mLoopReader.GetVertices() );
    }

    return Success;
}


//-----------------------------------------------------------------------------
void CLoopShape::BuildSegments( const std::vector<Vec2D>& arVertices )
{
    mSegments.clear();

    // Close the loop: the last vertex joins back up to the first one.
    Vec2D Previous = arVertices.back();
    for( const Vec2D& Vertex : arVertices )
    {
        mSegments.push_back( CSegment{ Previous, Vertex } );
        Previous = Vertex;
    }
}


//-----------------------------------------------------------------------------
const CPose& CLoopShape::GetStartPose() const
{
    return mLoopReader.GetStartPose();
}


//-----------------------------------------------------------------------------
// Checks every segment in turn and keeps whichever one comes out closest.
// HaveNearest just means "is this the first segment we've checked", so the
// very first distance found always gets kept as the starting best guess.
float CLoopShape::DistanceToNearestSegment( Vec2D aPoint ) const
{
    bool HaveNearest = false;
    float NearestDistance = 0.0f;

    for( const CSegment& Segment : mSegments )
    {
        float Distance = DistancePointToSegment( aPoint, Segment );
        if( !HaveNearest || Distance < NearestDistance )
        {
            NearestDistance = Distance;
            HaveNearest = true;
        }
    }

    return NearestDistance;
}


//-----------------------------------------------------------------------------
// Checks every segment for where the ray crosses it, and keeps the closest
// crossing found. Starting NearestDistance at aMaxRange means a ray that hits
// nothing at all just reports aMaxRange, with no separate "nothing found" case
// needed.
float CLoopShape::RangeAlongRay( Vec2D aOrigin, float aAngle, float aMaxRange ) const
{
    Vec2D Direction{ std::cos( aAngle ), std::sin( aAngle ) };

    float NearestDistance = aMaxRange;

    for( const CSegment& Segment : mSegments )
    {
        float Distance = RayIntersectSegment( aOrigin, Direction, Segment );
        if( Distance >= 0.0f && Distance < NearestDistance )
        {
            NearestDistance = Distance;
        }
    }

    return NearestDistance;
}


//-----------------------------------------------------------------------------
void CLoopShape::DrawSegments( CRender& arRender, float aThickness, Color aColour ) const
{
    for( const CSegment& Segment : mSegments )
    {
        arRender.DrawLine( Segment.mStart, Segment.mEnd, aThickness, aColour );
    }
}


//-----------------------------------------------------------------------------
// The shortest distance from aPoint to the segment. The point is allowed to
// project anywhere along the segment's length, but that projection is clamped
// to the two ends, so this measures against the real segment and not the
// infinite line running through it.
//-----------------------------------------------------------------------------
float CLoopShape::DistancePointToSegment( Vec2D aPoint, const CSegment& arSegment )
{
    const float Epsilon = 1.0e-6f;

    Vec2D SegmentVector{ arSegment.mEnd.x - arSegment.mStart.x, arSegment.mEnd.y - arSegment.mStart.y };
    Vec2D StartToPoint{ aPoint.x - arSegment.mStart.x, aPoint.y - arSegment.mStart.y };

    float SegmentLengthSquared = SegmentVector.x * SegmentVector.x + SegmentVector.y * SegmentVector.y;

    // How far along the segment (0 at the start, 1 at the end) the point
    // lands, clamped so the closest point stays on the segment itself.
    float Projection = 0.0f;
    if( SegmentLengthSquared > Epsilon )
    {
        Projection = ( StartToPoint.x * SegmentVector.x + StartToPoint.y * SegmentVector.y ) / SegmentLengthSquared;
        Projection = std::max( 0.0f, std::min( 1.0f, Projection ) );
    }

    Vec2D ClosestPoint{
        arSegment.mStart.x + Projection * SegmentVector.x,
        arSegment.mStart.y + Projection * SegmentVector.y
    };

    Vec2D Delta{ aPoint.x - ClosestPoint.x, aPoint.y - ClosestPoint.y };

    return std::sqrt( Delta.x * Delta.x + Delta.y * Delta.y );
}


//-----------------------------------------------------------------------------
// Where the ray (aOrigin plus t times aDirection, for t >= 0) crosses the
// segment. Returns -1 if it never does: the ray and segment run parallel, the
// crossing point falls behind the ray's start, or it falls outside the
// segment's two ends.
//-----------------------------------------------------------------------------
float CLoopShape::RayIntersectSegment( Vec2D aOrigin, Vec2D aDirection, const CSegment& arSegment )
{
    const float Epsilon = 1.0e-6f;
    const float NoIntersection = -1.0f;   // returned when the ray misses the segment

    Vec2D SegmentVector{ arSegment.mEnd.x - arSegment.mStart.x, arSegment.mEnd.y - arSegment.mStart.y };
    Vec2D OriginToStart{ arSegment.mStart.x - aOrigin.x, arSegment.mStart.y - aOrigin.y };

    // The 2D "cross product" of aDirection and SegmentVector.
    float Denominator = aDirection.x * SegmentVector.y - aDirection.y * SegmentVector.x;

    float Result = NoIntersection;

    if( std::fabs( Denominator ) > Epsilon )
    {
        float RayDistance = ( OriginToStart.x * SegmentVector.y - OriginToStart.y * SegmentVector.x ) / Denominator;
        float SegmentFraction = ( OriginToStart.x * aDirection.y - OriginToStart.y * aDirection.x ) / Denominator;

        if( RayDistance >= 0.0f && SegmentFraction >= 0.0f && SegmentFraction <= 1.0f )
        {
            Result = RayDistance;
        }
    }

    return Result;
}
