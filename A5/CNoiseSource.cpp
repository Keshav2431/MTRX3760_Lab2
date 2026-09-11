//-----------------------------------------------------------------------------
// CNoiseSource.cpp
//
// Every noise function here is just one call to RandomUnit(), scaled by
// whichever of the three sizes below applies. Those three sizes are really
// the only decisions being made in this file.
//-----------------------------------------------------------------------------

#include "CNoiseSource.h"

#include <cstdlib>
#include <iostream>

//---A robot has radius 15, and its wheels move about 2 units per step at
//   normal driving speed. The three sizes below were picked against those
//   two numbers:
//
//   - 4 units of start shift is roughly a quarter of a robot's radius. That's
//     enough to spread twenty robots out that would otherwise all start on
//     the same spot, but not so much that a line follower starts off the line.
//   - 0.1 radians is a bit under 6 degrees of starting heading error.
//   - 0.3 units of wheel slip is about 15% of one step. Each wheel slips on
//     its own, so the heading wanders a little every step - but only a
//     little, far less than a correction can undo, so a robot always finds
//     its way back and no two robots end up tracing the same path.----------
const float CNoiseSource::kStartMoveNoise = 4.0f;
const float CNoiseSource::kStartTurnNoise = 0.10f;
const float CNoiseSource::kWheelStepNoise = 0.30f;


//-----------------------------------------------------------------------------
// Seeds rand() so this run draws whichever sequence of numbers aSeed picks
// out. There's nothing else to set up.
//-----------------------------------------------------------------------------
CNoiseSource::CNoiseSource( unsigned int aSeed )
    :
        mSeed( aSeed )
{
    std::srand( aSeed );
}


//-----------------------------------------------------------------------------
CPose CNoiseSource::ScatterStartPose( const CPose& aIdealPose )
{
    CPose Result = aIdealPose;

    // x, y and heading each get their own random number, so this really does
    // scatter the robots instead of shoving them all the same way.
    Result.mPosition.x += kStartMoveNoise * RandomUnit();
    Result.mPosition.y += kStartMoveNoise * RandomUnit();
    Result.mHeading    += kStartTurnNoise * RandomUnit();

    return Result;
}


//-----------------------------------------------------------------------------
float CNoiseSource::AddWheelSlip( float aIdealStep )
{
    return aIdealStep + kWheelStepNoise * RandomUnit();
}


//-----------------------------------------------------------------------------
void CNoiseSource::PrintNoiseLevels() const
{
    const float RadiansToDegrees = 57.2957795f;

    std::cout << "Noise: seed " << mSeed
              << ", start move +/-" << kStartMoveNoise << " units"
              << ", start turn +/-" << kStartTurnNoise * RadiansToDegrees << " degrees"
              << ", wheel slip +/-" << kWheelStepNoise << " units per step"
              << std::endl;
}


//-----------------------------------------------------------------------------
// rand() gives back a whole number from 0 to RAND_MAX. Dividing that range
// down to 0..1, doubling it, and subtracting 1 turns it into a number
// spread evenly between -1 and 1.
//-----------------------------------------------------------------------------
float CNoiseSource::RandomUnit()
{
    float ZeroToOne = static_cast<float>( std::rand() ) / static_cast<float>( RAND_MAX );

    return 2.0f * ZeroToOne - 1.0f;
}
