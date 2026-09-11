//-----------------------------------------------------------------------------
// CLineFollowerRobot.cpp
//
// The line-following half of the robot.
//
// Both sensors sit ahead of the robot's centre. mInnerSensor sits right on the
// centre line, so it sees paint whenever the robot is tracking well.
// mOuterSensor sits a little to the right of that, so it normally sees no
// paint at all - it only sees paint once the line has drifted that far over.
//
// SteerFromSensors looks at what those two sensors last saw and picks one of
// four moves: drive straight, turn right a little, turn right harder, or - if
// neither sensor sees any paint at all - turn left to go looking for the line
// again.
//
// Every turn happens at mTurnSpeed, well below the normal driving speed, so a
// sharp corner is taken almost like a pivot on the spot. That keeps the robot
// from ever running wide of the 5-unit-wide line. SimpleLine.map mostly turns
// right, with one left-hand corner, so whenever the line is lost, it is
// always somewhere to the left - which is why the "line lost" case always
// turns left to look for it again.
//
// None of this steering changes for the noise bonus. The "line lost" turn is
// what carries a robot through the noise: if a random nudge pushes it clear of
// the line, this same turn circles it back onto the line, and from there it
// tracks as normal. The wheel slip added each step is much smaller than one
// of these turns, so it just shows up as a little wobble along the line
// rather than as a genuinely lost line.
//-----------------------------------------------------------------------------

#include "CLineFollowerRobot.h"

const Color CLineFollowerRobot::kFamilyColour = LIME;


//-----------------------------------------------------------------------------
CLineFollowerRobot::CLineFollowerRobot( const CPose& arStartPose, const CRoom& arRoom, const CFloorLine& arLine,
                                        CNoiseSource& arNoise, float aShadeFraction )
    :
        CRobot( arStartPose, arRoom, arNoise,
                CRender::Lighten( kFamilyColour, aShadeFraction ) ),
        mSensorForwardOffset( 20.0f ),
        mSensorLateralOffset( 7.0f ),
        mTurnSpeed( 50.0f ),
        mSteerDelta( 40.0f ),
        mHardSteerDelta( 60.0f ),
        mInnerSensor( mSensorForwardOffset, 0.0f ),
        mOuterSensor( mSensorForwardOffset, mSensorLateralOffset ),
        mInnerOnLine( false ),
        mOuterOnLine( false ),
        mrLine( arLine )
{
}


//-----------------------------------------------------------------------------
std::string CLineFollowerRobot::Name() const
{
    return "Line follower";
}


//-----------------------------------------------------------------------------
void CLineFollowerRobot::Sense()
{
    mInnerOnLine = mInnerSensor.Sense( Pose(), mrLine );
    mOuterOnLine = mOuterSensor.Sense( Pose(), mrLine );
}


//-----------------------------------------------------------------------------
void CLineFollowerRobot::SteerFromSensors()
{
    const float Base = BaseSpeed();

    float LeftSpeed = Base;
    float RightSpeed = Base;

    if( mInnerOnLine && !mOuterOnLine )
    {
        // Right on the line: drive straight at the normal speed.
        LeftSpeed = Base;
        RightSpeed = Base;
    }
    else if( mInnerOnLine && mOuterOnLine )
    {
        // The line has drifted right, into the outer sensor: slow the right
        // wheel so the faster left wheel swings the robot back to the right,
        // and slow both wheels down to turning speed.
        LeftSpeed = mTurnSpeed;
        RightSpeed = mTurnSpeed - mSteerDelta;
    }
    else if( mOuterOnLine )
    {
        // Only the outer sensor still sees the line: the same right turn as
        // above, but harder, before the line disappears completely.
        LeftSpeed = mTurnSpeed;
        RightSpeed = mTurnSpeed - mHardSteerDelta;
    }
    else
    {
        // Neither sensor sees any paint: the line must now be to the left.
        // Slow the left wheel so the faster right wheel swings the robot
        // back around to the left.
        LeftSpeed = mTurnSpeed - mSteerDelta;
        RightSpeed = mTurnSpeed;
    }

    SetWheelSpeeds( LeftSpeed, RightSpeed );
}
