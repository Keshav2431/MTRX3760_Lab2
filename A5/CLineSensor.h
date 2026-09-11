//-----------------------------------------------------------------------------
// CLineSensor.h
//
// A CLineSensor is a single downward-facing sensor, fixed to a robot at a set
// spot relative to its centre. Given the robot's pose and the floor line, it
// reports whether the floor right underneath it is line or not.
//
// It doesn't share anything with CRangeSensor on purpose. A CRangeSensor is
// mounted at an angle and measures a distance; this one is mounted at a
// position and just says yes or no. They only have the word "sensor" in
// common.
//-----------------------------------------------------------------------------

#ifndef CLINESENSOR_H
#define CLINESENSOR_H

#include "CLoopReader.h"   // CPose

class CFloorLine;   // forward declaration: only used here by const reference

//-----------------------------------------------------------------------------
class CLineSensor
{
    public:
        //---Ctor---
        // Where the sensor is mounted, relative to the robot:
        //   aForwardOffset - how far ahead, along the way the robot faces
        //   aLateralOffset - how far to the robot's right (90 degrees off that)
        CLineSensor( float aForwardOffset, float aLateralOffset );

        //---Sensing---
        // True if the floor under the sensor is line, given the robot's pose.
        bool Sense( const CPose& arRobotPose, const CFloorLine& arLine ) const;

    private:
        const float mForwardOffset;   // how far ahead of the robot the sensor sits
        const float mLateralOffset;   // how far to the robot's right the sensor sits
};

#endif
