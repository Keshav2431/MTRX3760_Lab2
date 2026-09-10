//-----------------------------------------------------------------------------
// CRobot.cpp
//
// The shared robot mechanics, lifted almost verbatim from A1's
// CWallFollowerRobot: the Update() sequence, the collision handling (reject
// the move but keep the heading change; count once on entry), the trail, and
// the drawing. Only the sensing and steering are left to the subclass.
//-----------------------------------------------------------------------------

#include "CRobot.h"
#include "CRoom.h"

#include <cmath>
#include <cstddef>
#include <iostream>

//-----------------------------------------------------------------------------
CRobot::CRobot( const CPose& arStartPose, const CRoom& arRoom, Color aBodyColour )
    :
        mRadius( 15.0f ),
        mAxleWidth( 26.0f ),
        mBaseSpeed( 40.0f ),
        mTimeStep( 0.05f ),
        mPose( arStartPose ),
        mDriveTrain( mAxleWidth ),
        mBodyColour( aBodyColour ),
        mUpdateCount( 0 ),
        mCollisionCount( 0 ),
        mWasColliding( false ),
        mrRoom( arRoom )
{
    mTrail.push_back( mPose.mPosition );
}


//-----------------------------------------------------------------------------
void CRobot::Update()
{
    Sense();
    SteerFromSensors();

    CPose TentativePose = mDriveTrain.Advance( mPose, mTimeStep );

    if( HasCollided( TentativePose ) )
    {
        // Reject the position change but keep the new heading, so steering
        // can still turn the robot clear of the wall on a later update.
        mPose.mHeading = TentativePose.mHeading;

        if( !mWasColliding )
        {
            ++mCollisionCount;
            std::cout << Name() << " collision #" << mCollisionCount
                       << " at update " << mUpdateCount << std::endl;
        }
        mWasColliding = true;
    }
    else
    {
        mPose = TentativePose;
        mWasColliding = false;
    }

    mTrail.push_back( mPose.mPosition );
    ++mUpdateCount;
}


//-----------------------------------------------------------------------------
void CRobot::Draw( CRender& arRender ) const
{
    const float TrailThickness = 1.5f;
    const float HeadingLineThickness = 2.0f;

    // Trail, drawn as a chain of segments between consecutive stored points.
    for( std::size_t i = 1; i < mTrail.size(); ++i )
    {
        arRender.DrawLine( mTrail[i - 1], mTrail[i], TrailThickness, GRAY );
    }

    // Body.
    arRender.DrawCircle( mPose.mPosition, static_cast<int>( mRadius ), mBodyColour );

    // Heading indicator: a line from the centre to the edge, facing forward.
    Vec2D HeadingEnd
    {
        mPose.mPosition.x + mRadius * std::cos( mPose.mHeading ),
        mPose.mPosition.y + mRadius * std::sin( mPose.mHeading )
    };
    arRender.DrawLine( mPose.mPosition, HeadingEnd, HeadingLineThickness, BLACK );
}


//-----------------------------------------------------------------------------
int CRobot::GetUpdateCount() const
{
    return mUpdateCount;
}


//-----------------------------------------------------------------------------
int CRobot::GetCollisionCount() const
{
    return mCollisionCount;
}


//-----------------------------------------------------------------------------
const CPose& CRobot::Pose() const
{
    return mPose;
}


//-----------------------------------------------------------------------------
const CRoom& CRobot::Room() const
{
    return mrRoom;
}


//-----------------------------------------------------------------------------
void CRobot::SetWheelSpeeds( float aLeft, float aRight )
{
    mDriveTrain.SetWheelSpeeds( aLeft, aRight );
}


//-----------------------------------------------------------------------------
float CRobot::BaseSpeed() const
{
    return mBaseSpeed;
}


//-----------------------------------------------------------------------------
bool CRobot::HasCollided( const CPose& arTentativePose ) const
{
    return mrRoom.IsColliding( arTentativePose.mPosition, mRadius );
}
