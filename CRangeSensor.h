//-----------------------------------------------------------------------------
// CRangeSensor.h
//
// A single range sensor, rigidly mounted on a robot at some fixed angle
// relative to the robot's heading. Given the robot's current pose and the
// room it is in, it reports the distance to the nearest wall along its beam.
//
// A CRangeSensor knows nothing about which robot it is mounted on, or how
// many other sensors that robot carries; it is handed a pose each time it is
// asked to sense.
//-----------------------------------------------------------------------------

#ifndef CRANGESENSOR_H
#define CRANGESENSOR_H

#include "CLoopReader.h"   // for CPose

class CRoom;   // forward declaration: only used here by const reference

//-----------------------------------------------------------------------------
class CRangeSensor
{
    public:
        //---Ctor---
        // aMountAngle: the sensor's direction relative to the robot's heading,
        // in radians (0 = straight ahead, positive = clockwise, matching the
        // heading convention used throughout the loop files and CPose).
        // aMaxRange: the furthest distance the sensor can report.
        CRangeSensor( float aMountAngle, float aMaxRange );

        //---Sensing---
        float Sense( const CPose& arRobotPose, const CRoom& arRoom ) const;

    private:
        const float mMountAngle;
        const float mMaxRange;
};

#endif
