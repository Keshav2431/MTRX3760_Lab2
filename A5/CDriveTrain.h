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
//
// For this noise bonus, it also holds the shared CNoiseSource, and Advance()
// uses it to add a small random slip to each wheel's travel. This is the only
// place in the whole program where the wheel noise comes in.
//-----------------------------------------------------------------------------

#ifndef CDRIVETRAIN_H
#define CDRIVETRAIN_H

#include "CLoopReader.h"   // for CPose

class CNoiseSource;   // shared with every drive train, so just a reference here

//-----------------------------------------------------------------------------
class CDriveTrain
{
    public:
        //---Ctor---
        CDriveTrain( float aAxleWidth, CNoiseSource& arNoise );

        //---Control---
        void SetWheelSpeeds( float aLeftSpeed, float aRightSpeed );

        //---Simulation---
        // Returns the pose reached by starting at arPose and driving at the
        // current wheel speeds for aTimeStep seconds, with a small random
        // slip added to each wheel along the way. Does not change arPose.
        CPose Advance( const CPose& arPose, float aTimeStep );

    private:
        const float mAxleWidth;

        float mLeftSpeed;
        float mRightSpeed;

        CNoiseSource& mrNoise;   // shared with every drive train; not owned
};

#endif
