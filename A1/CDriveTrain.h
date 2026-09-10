//-----------------------------------------------------------------------------
// CDriveTrain.h
//
// A two-wheeled differential drive train. It knows nothing about sensing or
// control strategy: it is simply told a speed for each wheel, and can then
// advance a given pose by one fixed time step according to those speeds.
//
// Coordinates follow the convention used throughout this project: x right,
// y down, heading in radians measured clockwise from +x. With that
// convention, driving the left wheel faster than the right turns the robot
// anticlockwise (heading decreases); driving the right wheel faster turns it
// clockwise (heading increases).
//-----------------------------------------------------------------------------

#ifndef CDRIVETRAIN_H
#define CDRIVETRAIN_H

#include "CLoopReader.h"   // for CPose

//-----------------------------------------------------------------------------
class CDriveTrain
{
    public:
        //---Ctor---
        explicit CDriveTrain( float aAxleWidth );

        //---Control---
        void SetWheelSpeeds( float aLeftSpeed, float aRightSpeed );

        //---Simulation---
        // Returns the pose that results from starting at arPose and driving
        // at the current wheel speeds for aTimeStep seconds of simulated
        // time. Does not modify arPose or any state of its own.
        CPose Advance( const CPose& arPose, float aTimeStep ) const;

    private:
        const float mAxleWidth;

        float mLeftSpeed;
        float mRightSpeed;
};

#endif
