//-----------------------------------------------------------------------------
// CWallFollowerRobot.cpp
//
// See CWallFollowerRobot.h for the overall design. The steering law used
// here is a simple two-mode proportional controller:
//
//   - Normally, it holds the side sensor at mTargetWallDistance: too far
//     from the wall and it steers toward it, too close and it steers away.
//   - When the forward-right sensor sees a wall closing in (a corner, or the
//     end of the current wall), that overrides the side control and steers
//     the robot left, around the corner.
//
// Collisions are handled by rejecting the position part of a move that would
// end inside a wall (keeping the heading change, so steering can still turn
// the robot clear), and are only counted and reported on the frame the
// robot first touches the wall, not on every frame it remains in contact.
//-----------------------------------------------------------------------------

#include "CWallFollowerRobot.h"
#include "CRoom.h"

#include <algorithm>
#include <cmath>
#include <iostream>

//---Fixed by the assignment spec: two sensors, aimed 90 and 45 degrees right---
static const float kSideSensorAngle    = float( M_PI ) / 2.0f;   // 90 degrees
static const float kForwardSensorAngle = float( M_PI ) / 4.0f;   // 45 degrees

//---Steering error is clamped to this magnitude before scaling by a gain, so
//   a lost sensor reading (returning mMaxSensorRange) cannot demand a wildly
//   aggressive turn---
static const float kMaxSteeringError = 100.0f;


//-----------------------------------------------------------------------------
CWallFollowerRobot::CWallFollowerRobot( const CPose& arStartPose, const CRoom& arRoom )
    :
        mRadius( 15.0f ),
        mAxleWidth( 26.0f ),
        mBaseSpeed( 40.0f ),
        mTimeStep( 0.05f ),
        mMaxSensorRange( 300.0f ),
        mTargetWallDistance( 60.0f ),
        mSteeringGain( 0.9f ),
        mCornerThreshold( 70.0f ),
        mCornerGain( 1.4f ),
        mPose( arStartPose ),
        mDriveTrain( mAxleWidth ),
        mSideSensor( kSideSensorAngle, mMaxSensorRange ),
        mForwardSensor( kForwardSensorAngle, mMaxSensorRange ),
        mSideReading( mMaxSensorRange ),
        mForwardReading( mMaxSensorRange ),
        mUpdateCount( 0 ),
        mCollisionCount( 0 ),
        mWasColliding( false ),
        mrRoom( arRoom )
{
    mTrail.push_back( mPose.mPosition );
}


//-----------------------------------------------------------------------------
void CWallFollowerRobot::Update()
{
    mSideReading    = mSideSensor.Sense( mPose, mrRoom );
    mForwardReading = mForwardSensor.Sense( mPose, mrRoom );

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
            std::cout << "Collision #" << mCollisionCount
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
void CWallFollowerRobot::Draw( CRender& arRender ) const
{
    const float TrailThickness = 1.5f;
    const float HeadingLineThickness = 2.0f;

    // Trail, drawn as a chain of segments between consecutive stored points.
    for( std::size_t i = 1; i < mTrail.size(); ++i )
    {
        arRender.DrawLine( mTrail[i - 1], mTrail[i], TrailThickness, GRAY );
    }

    // Body.
    arRender.DrawCircle( mPose.mPosition, static_cast<int>( mRadius ), SKYBLUE );

    // Heading indicator: a line from the centre to the edge, facing forward.
    Vec2D HeadingEnd
    {
        mPose.mPosition.x + mRadius * std::cos( mPose.mHeading ),
        mPose.mPosition.y + mRadius * std::sin( mPose.mHeading )
    };
    arRender.DrawLine( mPose.mPosition, HeadingEnd, HeadingLineThickness, BLACK );
}


//-----------------------------------------------------------------------------
int CWallFollowerRobot::GetUpdateCount() const
{
    return mUpdateCount;
}


//-----------------------------------------------------------------------------
int CWallFollowerRobot::GetCollisionCount() const
{
    return mCollisionCount;
}


//-----------------------------------------------------------------------------
void CWallFollowerRobot::SteerFromSensors()
{
    float LeftSpeed = mBaseSpeed;
    float RightSpeed = mBaseSpeed;

    if( mForwardReading < mCornerThreshold )
    {
        // A wall is closing in ahead-right: turn left to round the corner,
        // overriding the side-distance control below. The closer the wall,
        // the harder the turn.
        float CornerError = mCornerThreshold - mForwardReading;
        CornerError = std::min( CornerError, kMaxSteeringError );

        LeftSpeed  -= mCornerGain * CornerError;
        RightSpeed += mCornerGain * CornerError;
    }
    else
    {
        // Hold the side sensor at the target distance: too far away and we
        // steer toward the wall, too close and we steer away from it.
        float SideError = mSideReading - mTargetWallDistance;
        SideError = std::max( -kMaxSteeringError, std::min( kMaxSteeringError, SideError ) );

        LeftSpeed  += mSteeringGain * SideError;
        RightSpeed -= mSteeringGain * SideError;
    }

    mDriveTrain.SetWheelSpeeds( LeftSpeed, RightSpeed );
}


//-----------------------------------------------------------------------------
bool CWallFollowerRobot::HasCollided( const CPose& arTentativePose ) const
{
    return mrRoom.IsColliding( arTentativePose.mPosition, mRadius );
}
