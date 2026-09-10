//-----------------------------------------------------------------------------
// CLineSensor.cpp
//
// Rotates the sensor's robot-frame mount offset into world space by the
// robot's heading, then asks the line whether that world point is painted.
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

    // Forward is (cos, sin) along the heading; the robot's right is that
    // turned 90 degrees clockwise, i.e. (-sin, cos).
    Vec2D SamplePoint
    {
        arRobotPose.mPosition.x + mForwardOffset * CosHeading - mLateralOffset * SinHeading,
        arRobotPose.mPosition.y + mForwardOffset * SinHeading + mLateralOffset * CosHeading
    };

    return arLine.IsLineUnder( SamplePoint );
}
