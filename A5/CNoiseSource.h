//-----------------------------------------------------------------------------
// CNoiseSource.h
//
// A CNoiseSource is the one thing in the whole program that generates random
// numbers. It knows how big the two kinds of noise this bonus needs should be,
// and hands them out through two functions: one shifts a robot's start pose by
// a small random amount, and the other adds a small random slip to how far a
// wheel actually turns in a step.
//
// It's built on rand() and srand() from <cstdlib>. rand() shares one stream
// across the whole program rather than each object getting its own, but that's
// fine here because the program only ever makes one CNoiseSource.
//
// One CNoiseSource is made in CSimulation and shared by every robot, so the
// whole run comes from a single seeded sequence and can be run again exactly.
//-----------------------------------------------------------------------------

#ifndef CNOISESOURCE_H
#define CNOISESOURCE_H

#include "CLoopReader.h"   // CPose

//-----------------------------------------------------------------------------
class CNoiseSource
{
    public:
        //---Ctor---
        // aSeed picks the random sequence, via srand(). The same seed always
        // gives the same run, which is how the report screenshot can be
        // reproduced later.
        explicit CNoiseSource( unsigned int aSeed );

        //---Noise---
        // Returns aIdealPose nudged by a small random amount in x, y and
        // heading, so each robot starts from a slightly different place.
        CPose ScatterStartPose( const CPose& aIdealPose );

        // Returns aIdealStep - the distance a wheel was told to move this step
        // - with a small random slip added on, so the two wheels never move by
        // exactly the amount they were asked to.
        float AddWheelSlip( float aIdealStep );

        //---Reporting---
        // Prints the seed and the three noise amounts, so a screenshot can be
        // matched back to the run that produced it.
        void PrintNoiseLevels() const;

    private:
        //---A random number between -1 and 1, spread evenly, from rand()---
        float RandomUnit();

        //---How big each kind of noise is. Chosen against a robot of radius 15
        //   whose wheels move about 2 units per step at normal driving speed.---
        static const float kStartMoveNoise;   // how far the start point can shift, in units
        static const float kStartTurnNoise;   // how far the start heading can shift, in radians
        static const float kWheelStepNoise;   // how much a wheel's step can vary, in units

        //---Kept only so PrintNoiseLevels can say which run this was---
        const unsigned int mSeed;
};

#endif
