//-----------------------------------------------------------------------------
// CRoom.cpp
//
// Implements CRoom. LoadFromFile turns the loop's vertices into a closed
// chain of CWallSegments (last vertex joins back to the first, matching the
// loop file format described in CLoopReader.h). RangeToWall and IsColliding
// are both simple "test every wall segment, keep the best result" loops built
// on top of two small geometry helpers.
//-----------------------------------------------------------------------------

#include "CRoom.h"

#include <algorithm>
#include <cmath>
#include <iostream>

//-----------------------------------------------------------------------------
bool CRoom::LoadFromFile( const std::string& arFilename )
{
    bool Success = mLoopReader.ReadFile( arFilename );

    if( Success )
    {
        mWalls.clear();

        const std::vector<Vec2D>& Vertices = mLoopReader.GetVertices();
        if( Vertices.empty() )
        {
            std::cout << "CRoom: '" << arFilename << "' contains no vertices" << std::endl;
            Success = false;
        }
        else
        {
            // Close the loop: the last vertex joins back to the first.
            Vec2D Previous = Vertices.back();
            for( const Vec2D& Vertex : Vertices )
            {
                mWalls.push_back( CWallSegment{ Previous, Vertex } );
                Previous = Vertex;
            }
        }
    }

    return Success;
}


//-----------------------------------------------------------------------------
const CPose& CRoom::GetStartPose() const
{
    return mLoopReader.GetStartPose();
}


//-----------------------------------------------------------------------------
float CRoom::RangeToWall( Vec2D aOrigin, float aAngle, float aMaxRange ) const
{
    Vec2D Direction{ std::cos( aAngle ), std::sin( aAngle ) };

    float NearestDistance = aMaxRange;

    for( const CWallSegment& Wall : mWalls )
    {
        float Distance = RayIntersectSegment( aOrigin, Direction, Wall );
        if( Distance >= 0.0f && Distance < NearestDistance )
        {
            NearestDistance = Distance;
        }
    }

    return NearestDistance;
}


//-----------------------------------------------------------------------------
bool CRoom::IsColliding( Vec2D aPosition, float aRadius ) const
{
    bool Colliding = false;

    for( const CWallSegment& Wall : mWalls )
    {
        if( DistancePointToSegment( aPosition, Wall ) < aRadius )
        {
            Colliding = true;
            break;
        }
    }

    return Colliding;
}


//-----------------------------------------------------------------------------
void CRoom::Draw( CRender& arRender ) const
{
    const float WallThickness = 2.0f;

    for( const CWallSegment& Wall : mWalls )
    {
        arRender.DrawLine( Wall.mStart, Wall.mEnd, WallThickness, RAYWHITE );
    }
}


//-----------------------------------------------------------------------------
// Shortest distance from aPoint to the nearest point on the segment (clamping
// the projection to the segment's ends, so it is a true segment, not an
// infinite line).
//-----------------------------------------------------------------------------
float CRoom::DistancePointToSegment( Vec2D aPoint, const CWallSegment& arSegment )
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
float CRoom::RayIntersectSegment( Vec2D aOrigin, Vec2D aDirection, const CWallSegment& arSegment )
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
