//-----------------------------------------------------------------------------
// CWallFollowerRobot.cpp
//
// The wall-specific half of the robot. The steering law is unchanged from A1:
// a two-mode proportional controller on the two wheel speeds around a base
// speed.
//
//   - Corner mode: if the 45-degree forward-right sensor reads below
//     mCornerThreshold, a wall is closing in ahead; steer left, harder the
//     closer it is.
//   - Wall-hold mode: otherwise hold the 90-degree side sensor at
//     mTargetWallDistance; too far steers toward the wall, too close away.
//
// The steering error is clamped so a lost reading (max range) cannot demand a
// violent turn.
//-----------------------------------------------------------------------------

#include "CWallFollowerRobot.h"
#include "CRoom.h"

#include <algorithm>
#include <cmath>

//---Fixed by the assignment spec: two sensors, aimed 90 and 45 degrees right---
static const float kPi = 3.14159265358979323846f;
static const float kSideSensorAngle    = kPi / 2.0f;   // 90 degrees
static const float kForwardSensorAngle = kPi / 4.0f;   // 45 degrees

//---Steering error is clamped to this magnitude before scaling by a gain---
static const float kMaxSteeringError = 100.0f;


//-----------------------------------------------------------------------------
CWallFollowerRobot::CWallFollowerRobot( const CPose& arStartPose, const CRoom& arRoom )
    :
        CRobot( arStartPose, arRoom, SKYBLUE ),
        mMaxSensorRange( 300.0f ),
        mTargetWallDistance( 60.0f ),
        mSteeringGain( 0.9f ),
        mCornerThreshold( 70.0f ),
        mCornerGain( 1.4f ),
        mSideSensor( kSideSensorAngle, mMaxSensorRange ),
        mForwardSensor( kForwardSensorAngle, mMaxSensorRange ),
        mSideReading( mMaxSensorRange ),
        mForwardReading( mMaxSensorRange )
{
}


//-----------------------------------------------------------------------------
std::string CWallFollowerRobot::Name() const
{
    return "Wall follower";
}


//-----------------------------------------------------------------------------
void CWallFollowerRobot::Sense()
{
    mSideReading    = mSideSensor.Sense( Pose(), Room() );
    mForwardReading = mForwardSensor.Sense( Pose(), Room() );
}


//-----------------------------------------------------------------------------
void CWallFollowerRobot::SteerFromSensors()
{
    float LeftSpeed = BaseSpeed();
    float RightSpeed = BaseSpeed();

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

    SetWheelSpeeds( LeftSpeed, RightSpeed );
}
