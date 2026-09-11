//-----------------------------------------------------------------------------
// CWallFollowerRobot.h
//
// A CRobot that follows the wall on its right-hand side. It carries the two
// range sensors the spec fixes (one aimed 90 degrees right, one 45 degrees
// forward-right) and supplies the two CRobot hooks:
//
//   Sense()            - read both range sensors against the room
//   SteerFromSensors() - a two-mode proportional controller (see the .cpp)
//
// All the machinery it used to have in A1 - pose, drive train, trail,
// collision handling, counters, Update(), Draw() - now lives in CRobot.
//
// For this noise bonus, the constructor also takes the shared noise source
// (passed straight on to the drive train) and a shade fraction from 0 to 1,
// which lightens the body colour so the twenty wall followers can be told
// apart on screen.
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWERROBOT_H
#define CWALLFOLLOWERROBOT_H

#include "CRobot.h"
#include "CRangeSensor.h"
#include "CLoopReader.h"   // CPose

#include <string>

class CRoom;          // forward declaration: only used here by const reference
class CNoiseSource;   // forward declaration: only used here by reference

//-----------------------------------------------------------------------------
class CWallFollowerRobot : public CRobot
{
    public:
        //---Ctor---
        CWallFollowerRobot( const CPose& arStartPose, const CRoom& arRoom,
                            CNoiseSource& arNoise, float aShadeFraction );

        //---Reporting---
        std::string Name() const override;

    protected:
        //---CRobot hooks---
        void Sense() override;
        void SteerFromSensors() override;

    private:
        //---The colour every wall follower is a lighter shade of---
        static const Color kFamilyColour;

        //---Tuning, fixed for the life of the robot---
        const float mMaxSensorRange;       // sensors report at most this far
        const float mTargetWallDistance;   // desired distance from the side wall
        const float mSteeringGain;         // proportional gain, side-distance control
        const float mCornerThreshold;      // forward reading below this means "corner ahead"
        const float mCornerGain;           // proportional gain, corner-avoidance control

        //---Sensors: 90 degrees right, and 45 degrees forward-right---
        CRangeSensor mSideSensor;
        CRangeSensor mForwardSensor;

        //---Most recent readings, refreshed by Sense()---
        float mSideReading;
        float mForwardReading;
};

#endif
