//-----------------------------------------------------------------------------
// CLineSensor.h
//
// A single downward-looking line sensor, rigidly mounted on a robot at a fixed
// offset from the robot's centre. Given the robot's pose and the floor line,
// it reports whether the floor directly beneath it is line or not.
//
// Deliberately not sharing a base with CRangeSensor: that one is mounted by
// angle and reports a distance; this one is mounted by position and reports a
// bool. They have nothing to share but a name.
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
        // The mount point, in the robot frame:
        //   aForwardOffset - along the robot's heading
        //   aLateralOffset - to the robot's right (heading + 90 degrees)
        CLineSensor( float aForwardOffset, float aLateralOffset );

        //---Sensing---
        bool Sense( const CPose& arRobotPose, const CFloorLine& arLine ) const;

    private:
        const float mForwardOffset;
        const float mLateralOffset;
};

#endif
