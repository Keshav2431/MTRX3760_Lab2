//-----------------------------------------------------------------------------
// CLoopShape.cpp
//
// Implements CLoopShape. LoadFromFile turns the loop's vertices into a closed
// chain of CSegments (last vertex joins back to the first). DistanceToNearest-
// Segment and RangeAlongRay are both "test every segment, keep the best
// result" loops over two small geometry helpers. All of this was CRoom in A1;
// it moved here so CFloorLine can reuse it.
//-----------------------------------------------------------------------------

#include "CLoopShape.h"

#include <algorithm>
#include <cmath>
#include <iostream>

//-----------------------------------------------------------------------------
bool CLoopShape::LoadFromFile( const std::string& arFilename )
{
    bool Success = mLoopReader.ReadFile( arFilename );

    if( Success )
    {
        mSegments.clear();

        const std::vector<Vec2D>& Vertices = mLoopReader.GetVertices();
        if( Vertices.empty() )
        {
            std::cout << "CLoopShape: '" << arFilename << "' contains no vertices" << std::endl;
            Success = false;
        }
        else
        {
            // Close the loop: the last vertex joins back to the first.
            Vec2D Previous = Vertices.back();
            for( const Vec2D& Vertex : Vertices )
            {
                mSegments.push_back( CSegment{ Previous, Vertex } );
                Previous = Vertex;
            }
        }
    }

    return Success;
}


//-----------------------------------------------------------------------------
const CPose& CLoopShape::GetStartPose() const
{
    return mLoopReader.GetStartPose();
}


//-----------------------------------------------------------------------------
float CLoopShape::DistanceToNearestSegment( Vec2D aPoint ) const
{
    float NearestDistance = -1.0f;

    for( const CSegment& Segment : mSegments )
    {
        float Distance = DistancePointToSegment( aPoint, Segment );
        if( NearestDistance < 0.0f || Distance < NearestDistance )
        {
            NearestDistance = Distance;
        }
    }

    return NearestDistance;
}


//-----------------------------------------------------------------------------
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
// Shortest distance from aPoint to the nearest point on the segment (clamping
// the projection to the segment's ends, so it is a true segment, not an
// infinite line).
//-----------------------------------------------------------------------------
float CLoopShape::DistancePointToSegment( Vec2D aPoint, const CSegment& arSegment )
{
    const float Epsilon = 1.0e-6f;

    Vec2D SegmentVector{ arSegment.mEnd.x - arSegment.mStart.x, arSegment.mEnd.y - arSegment.mStart.y };
    Vec2D StartToPoint{ aPoint.x - arSegment.mStart.x, aPoint.y - arSegment.mStart.y };

    float SegmentLengthSquared = SegmentVector.x * SegmentVector.x + SegmentVector.y * SegmentVector.y;

    float T = 0.0f;
    if( SegmentLengthSquared > Epsilon )
    {
        T = ( StartToPoint.x * SegmentVector.x + StartToPoint.y * SegmentVector.y ) / SegmentLengthSquared;
        T = std::max( 0.0f, std::min( 1.0f, T ) );
    }

    Vec2D ClosestPoint{
        arSegment.mStart.x + T * SegmentVector.x,
        arSegment.mStart.y + T * SegmentVector.y
    };

    Vec2D Delta{ aPoint.x - ClosestPoint.x, aPoint.y - ClosestPoint.y };

    return std::sqrt( Delta.x * Delta.x + Delta.y * Delta.y );
}


//-----------------------------------------------------------------------------
// Distance along the ray (aOrigin + t * aDirection, t >= 0) to where it meets
// the segment, or -1 if the ray misses it (parallel, or the meeting point
// falls before the ray's start or outside the segment).
//-----------------------------------------------------------------------------
float CLoopShape::RayIntersectSegment( Vec2D aOrigin, Vec2D aDirection, const CSegment& arSegment )
{
    const float Epsilon = 1.0e-6f;

    Vec2D SegmentVector{ arSegment.mEnd.x - arSegment.mStart.x, arSegment.mEnd.y - arSegment.mStart.y };
    Vec2D OriginToStart{ arSegment.mStart.x - aOrigin.x, arSegment.mStart.y - aOrigin.y };

    // 2D "cross product": aDirection x SegmentVector.
    float Denominator = aDirection.x * SegmentVector.y - aDirection.y * SegmentVector.x;

    float Result = -1.0f;

    if( std::fabs( Denominator ) > Epsilon )
    {
        float T = ( OriginToStart.x * SegmentVector.y - OriginToStart.y * SegmentVector.x ) / Denominator;
        float U = ( OriginToStart.x * aDirection.y - OriginToStart.y * aDirection.x ) / Denominator;

        if( T >= 0.0f && U >= 0.0f && U <= 1.0f )
        {
            Result = T;
        }
    }

    return Result;
}
