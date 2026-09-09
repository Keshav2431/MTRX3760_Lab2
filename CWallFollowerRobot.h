//-----------------------------------------------------------------------------
// CWallFollowerRobot.h
//
// A simulated disc-shaped robot that drives around a room, following the
// wall on its right-hand side. It carries two range sensors (one aimed
// directly right, one aimed forward-and-right) and a two-wheel differential
// drive, and steers itself using only what those sensors report.
//
// The robot owns and drives itself: Update() reads its sensors, decides on
// wheel speeds, advances its own pose by one fixed simulated time step, and
// checks itself for collisions. Nothing outside the class needs to know how
// any of that is done.
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWERROBOT_H
#define CWALLFOLLOWERROBOT_H

#include "CRender.h"
#include "CLoopReader.h"   // for CPose
#include "CRangeSensor.h"
#include "CDriveTrain.h"

#include <vector>

class CRoom;   // forward declaration: only used here by const reference

//-----------------------------------------------------------------------------
class CWallFollowerRobot
{
    public:
        //---Ctor---
        // arStartPose: where the robot begins.
        // arRoom: the room this robot senses and collides against. The robot
        // does not own the room, and keeps only a reference to it.
        CWallFollowerRobot( const CPose& arStartPose, const CRoom& arRoom );

        //---Simulation---
        // Advances the robot by one fixed simulated time step: sense, steer,
        // move, and check for collisions.
        void Update();

        //---Rendering---
        void Draw( CRender& arRender ) const;

        //---Reporting---
        int GetUpdateCount() const;
        int GetCollisionCount() const;

    private:
        //---Internal behaviour, broken out of Update() for readability---
        void SteerFromSensors();
        bool HasCollided( const CPose& arTentativePose ) const;

        //---Robot geometry and tuning, fixed for the life of the robot---
        const float mRadius;               // body radius; fixed by the spec at 15
        const float mAxleWidth;            // distance between the two wheels
        const float mBaseSpeed;            // wheel speed when driving straight
        const float mTimeStep;             // simulated seconds advanced per Update()
        const float mMaxSensorRange;       // sensors report at most this far
        const float mTargetWallDistance;   // desired distance from the side wall
        const float mSteeringGain;         // proportional gain, side-distance control
        const float mCornerThreshold;      // forward reading below this means "corner ahead"
        const float mCornerGain;           // proportional gain, corner-avoidance control

        //---State---
        CPose mPose;
        CDriveTrain mDriveTrain;
        CRangeSensor mSideSensor;      // aimed 90 degrees right: directly to the side
        CRangeSensor mForwardSensor;   // aimed 45 degrees right: forward-and-right

        float mSideReading;
        float mForwardReading;

        std::vector<Vec2D> mTrail;

        int mUpdateCount;
        int mCollisionCount;
        bool mWasColliding;   // so a collision is counted once on entry, not every
                              // frame the robot remains pressed against the wall

        const CRoom& mrRoom;   // the room this robot senses/collides against; not owned
};

#endif
