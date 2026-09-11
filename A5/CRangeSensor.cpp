//-----------------------------------------------------------------------------
// CRangeSensor.cpp
//-----------------------------------------------------------------------------

#include "CRangeSensor.h"
#include "CRoom.h"

//-----------------------------------------------------------------------------
CRangeSensor::CRangeSensor( float aMountAngle, float aMaxRange )
    :
        mMountAngle( aMountAngle ),
        mMaxRange( aMaxRange )
{
}


//-----------------------------------------------------------------------------
float CRangeSensor::Sense( const CPose& arRobotPose, const CRoom& arRoom ) const
{
    float WorldAngle = arRobotPose.mHeading + mMountAngle;

    // RangeToWall was renamed RangeAlongRay when it moved to CLoopShape.
    return arRoom.RangeAlongRay( arRobotPose.mPosition, WorldAngle, mMaxRange );
}
