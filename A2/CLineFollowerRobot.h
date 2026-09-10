//-----------------------------------------------------------------------------
// CLineFollowerRobot.h
//
// A CRobot that follows the floor line. It carries two line sensors - one
// mounted over the line, one offset to the robot's right, beside it - and
// supplies the two CRobot hooks:
//
//   Sense()            - read both line sensors against the floor line
//   SteerFromSensors() - a bang-bang controller that keeps the line just to
//                        the robot's right (see the .cpp)
//
// It still collides against the room walls like the wall follower does (and
// should report ~0 collisions, since its line never touches a wall).
//-----------------------------------------------------------------------------

#ifndef CLINEFOLLOWERROBOT_H
#define CLINEFOLLOWERROBOT_H

#include "CRobot.h"
#include "CLineSensor.h"
#include "CLoopReader.h"   // CPose

#include <string>

class CRoom;        // forward declaration: only used here by const reference
class CFloorLine;   // forward declaration: only used here by const reference

//-----------------------------------------------------------------------------
class CLineFollowerRobot : public CRobot
{
    public:
        //---Ctor---
        // arRoom: for collision checks (shared with the wall follower).
        // arLine: the floor line this robot follows. Not owned.
        CLineFollowerRobot( const CPose& arStartPose, const CRoom& arRoom, const CFloorLine& arLine );

        //---Reporting---
        std::string Name() const override;

    protected:
        //---CRobot hooks---
        void Sense() override;
        void SteerFromSensors() override;

    private:
        //---Sensor mounting and steering tuning (tuned against SimpleLine.map)---
        const float mSensorForwardOffset;   // both sensors sit this far ahead of centre
        const float mSensorLateralOffset;   // the "beside" sensor's offset to the right
        const float mTurnSpeed;             // forward speed while correcting; below base speed so a
                                            //   corner becomes a near-pivot and the line is never lost
        const float mSteerDelta;            // wheel-speed swing for a normal correction
        const float mHardSteerDelta;        // larger swing when the line has slipped to the outer sensor

        //---Sensors: one over the line, one beside it to the right---
        CLineSensor mInnerSensor;
        CLineSensor mOuterSensor;

        //---Most recent readings, refreshed by Sense()---
        bool mInnerOnLine;
        bool mOuterOnLine;

        //---The line this robot follows; not owned---
        const CFloorLine& mrLine;
};

#endif
