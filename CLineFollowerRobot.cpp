//-----------------------------------------------------------------------------
// CLineFollowerRobot.cpp
//
// The line-specific half of the robot.
//
// SENSOR LAYOUT (spec: "one over the line and one to the side of the line")
//   mOnLineSensor  - mounted ahead of centre, no lateral offset: over the line
//   mOffLineSensor - same forward offset, shifted to the robot's right: beside
//                    the line while the robot is tracking it correctly
//
// STEERING (bang-bang around BaseSpeed(), keeping the line just to the right):
//   on  && !off  -> centred          -> drive straight
//   on  &&  off  -> line curving right -> steer right by mSteerDelta
//   !on &&  off  -> drifted left, line now only under the side sensor
//                                     -> steer right, harder
//   !on && !off  -> line lost (overshot, or a sharp left corner)
//                                     -> steer left to bring it back
//
// >>> SteerFromSensors() below is a stub: it drives straight. Fill in the law
// >>> above and tune mSensorLateralOffset / mSteerDelta and the lost-line
// >>> recovery against SimpleLine.map. The concave corner is the hard case.
// >>> Fallback if the prescribed layout tunes poorly: a symmetric straddle
// >>> pair (lateral -d and +d) and a left/right turn when exactly one sensor
// >>> sees the line.
//-----------------------------------------------------------------------------

#include "CLineFollowerRobot.h"
#include "CRoom.h"
#include "CFloorLine.h"

//-----------------------------------------------------------------------------
CLineFollowerRobot::CLineFollowerRobot( const CPose& arStartPose, const CRoom& arRoom, const CFloorLine& arLine )
    :
        CRobot( arStartPose, arRoom, LIME ),
        mSensorForwardOffset( 15.0f ),
        mSensorLateralOffset( 6.0f ),
        mSteerDelta( 20.0f ),
        mOnLineSensor( mSensorForwardOffset, 0.0f ),
        mOffLineSensor( mSensorForwardOffset, mSensorLateralOffset ),
        mOnLine( false ),
        mOffLine( false ),
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
    mOnLine  = mOnLineSensor.Sense( Pose(), mrLine );
    mOffLine = mOffLineSensor.Sense( Pose(), mrLine );
}


//-----------------------------------------------------------------------------
void CLineFollowerRobot::SteerFromSensors()
{
    // STUB - see the file header. Replace with the four-case bang-bang law.
    float LeftSpeed = BaseSpeed();
    float RightSpeed = BaseSpeed();

    SetWheelSpeeds( LeftSpeed, RightSpeed );
}
