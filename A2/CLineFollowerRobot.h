//-----------------------------------------------------------------------------
// CLineFollowerRobot.h
//
// A CLineFollowerRobot is a CRobot that follows the floor line. It carries two
// line sensors - one sitting over the line, one just beside it to the robot's
// right - and writes the two functions CRobot needs: Sense() checks both
// sensors, and SteerFromSensors() runs a simple on/off steering rule that
// keeps the line just to the robot's right (see the .cpp for how).
//
// It still checks for wall collisions, the same as the wall follower, but
// should end up with close to zero of them, since its line never touches a
// wall.
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
        // arRoom: shared with the wall follower, just for collision checks.
        // arLine: the floor line this robot follows. Not owned.
        CLineFollowerRobot( const CPose& arStartPose, const CRoom& arRoom, const CFloorLine& arLine );

        //---Reporting---
        std::string Name() const override;   // "Line follower"

    protected:
        //---The two functions CRobot needs a specific robot to write---
        void Sense() override;              // checks both line sensors
        void SteerFromSensors() override;   // decides the wheel speeds from what they found

    private:
        //---Sensor placement and steering settings, tuned for SimpleLine.map---
        const float mSensorForwardOffset;   // how far ahead of the robot both sensors sit
        const float mSensorLateralOffset;   // how far the "side" sensor sits to the right
        const float mTurnSpeed;             // forward speed while correcting; slower than base speed,
                                            //   so corners are turned almost on the spot
        const float mSteerDelta;            // how much the wheel speeds differ for a normal turn
        const float mHardSteerDelta;        // how much they differ when the line is almost lost

        //---Sensors: one over the line, one beside it to the right---
        CLineSensor mInnerSensor;
        CLineSensor mOuterSensor;

        //---What the sensors last saw, updated every time Sense() runs---
        bool mInnerOnLine;
        bool mOuterOnLine;

        //---The line this robot follows; not owned---
        const CFloorLine& mrLine;
};

#endif
