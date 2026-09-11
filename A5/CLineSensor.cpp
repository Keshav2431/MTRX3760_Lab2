//-----------------------------------------------------------------------------
// CLineSensor.cpp
//
// Turns the sensor's mount position - which is measured relative to the robot
// - into a point in the world, using the robot's current heading, then asks
// the line whether that point is on the paint.
//-----------------------------------------------------------------------------

#include "CLineSensor.h"
#include "CFloorLine.h"

#include <cmath>

//-----------------------------------------------------------------------------
CLineSensor::CLineSensor( float aForwardOffset, float aLateralOffset )
    :
        mForwardOffset( aForwardOffset ),
        mLateralOffset( aLateralOffset )
{
}


//-----------------------------------------------------------------------------
bool CLineSensor::Sense( const CPose& arRobotPose, const CFloorLine& arLine ) const
{
    float CosHeading = std::cos( arRobotPose.mHeading );
    float SinHeading = std::sin( arRobotPose.mHeading );

    // Straight ahead is (cos, sin) of the heading. The robot's right is that
    // direction turned 90 degrees clockwise, which is (-sin, cos).
    Vec2D SamplePoint
    {
        arRobotPose.mPosition.x + mForwardOffset * CosHeading - mLateralOffset * SinHeading,
        arRobotPose.mPosition.y + mForwardOffset * SinHeading + mLateralOffset * CosHeading
    };

    return arLine.IsLineUnder( SamplePoint );
}
