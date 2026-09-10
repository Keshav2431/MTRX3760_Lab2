//-----------------------------------------------------------------------------
// CLineFollowerRobot.cpp
//
// The line-specific half of the robot.
//
// SENSOR LAYOUT (spec: "one over the line and one to the side of the line")
//   mInnerSensor - mounted ahead of centre, no lateral offset: over the line
//                  while the robot is tracking it correctly.
//   mOuterSensor - same forward offset, shifted onto the interior side of the
//                  loop (the robot's right): clear of the paint while tracking.
//
// STEERING is bang-bang on the two wheel speeds. The four sensor combinations
// map straight onto {straight, right, left}:
//
//   inner && !outer  -> centred                    -> straight at base speed
//   inner &&  outer  -> line has reached the outer  -> turn right
//                       sensor: a right-hand bend
//   !inner &&  outer  -> only the outer sensor is    -> turn right, harder
//                       still on the line
//   !inner && !outer  -> no paint at all: overshot   -> turn left
//                       a bend, or the concave corner
//
// Every correction is taken at mTurnSpeed, which is well below base speed, so a
// sharp vertex is rounded as a near-pivot: the robot barely translates while it
// swings onto the next edge, so it never leaves the 5-unit line and never gets
// far enough away to re-acquire the line facing backwards.
//
// SimpleLine.map runs clockwise with the loop interior on the robot's right, so
// five corners bend right and one (the concave corner at vertex 2) bends left;
// whenever the line is lost it is now to the left, so the recovery turns left.
//-----------------------------------------------------------------------------

#include "CLineFollowerRobot.h"
#include "CRoom.h"
#include "CFloorLine.h"

//-----------------------------------------------------------------------------
CLineFollowerRobot::CLineFollowerRobot( const CPose& arStartPose, const CRoom& arRoom, const CFloorLine& arLine )
    :
        CRobot( arStartPose, arRoom, LIME ),
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
        // Centred on the line: drive straight at base speed.
        LeftSpeed = Base;
        RightSpeed = Base;
    }
    else if( mInnerOnLine && mOuterOnLine )
    {
        // Right-hand bend: slow the right wheel so the faster left wheel
        // swings the robot right, and drop the pair to turn speed.
        LeftSpeed = mTurnSpeed;
        RightSpeed = mTurnSpeed - mSteerDelta;
    }
    else if( mOuterOnLine )
    {
        // Only the outer sensor is still on the line: the same right turn,
        // harder, before the line is lost entirely.
        LeftSpeed = mTurnSpeed;
        RightSpeed = mTurnSpeed - mHardSteerDelta;
    }
    else
    {
        // No paint under either sensor: the line is now to the left. Slow the
        // left wheel so the faster right wheel swings the robot left.
        LeftSpeed = mTurnSpeed - mSteerDelta;
        RightSpeed = mTurnSpeed;
    }

    SetWheelSpeeds( LeftSpeed, RightSpeed );
}
