//-----------------------------------------------------------------------------
// CRobot.h
//
// A CRobot is the common part of every robot in the simulation. It keeps
// track of where the robot is, drives its wheels, remembers its trail, and
// counts how many steps it has taken and how many times it has hit a wall.
//
// It does not know how to sense the world or how to steer. Each specific kind
// of robot - the wall follower, the line follower - fills in Sense() and
// SteerFromSensors() to do that its own way. Everything else about running
// the robot happens here, once, for both kinds.
//
// For this noise bonus, a robot is also given the shared CNoiseSource, so its
// drive train can add a little slip to each wheel. That's the only thing that
// changes: it still steers purely on what its own sensors tell it and just has
// to cope with the slip. A robot also stops moving once it has driven a full
// lap, so a finished robot doesn't keep drawing over its own picture, and wall
// touches are still counted but no longer printed one by one, since the bonus
// doesn't score them and forty robots hitting walls would flood the console.
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CRender.h"       // CRender, Color, Vec2D
#include "CLoopReader.h"   // CPose
#include "CDriveTrain.h"

#include <string>
#include <vector>

class CRoom;          // forward declaration: held by const reference, not owned
class CNoiseSource;   // forward declaration: shared, held by reference, not owned

//-----------------------------------------------------------------------------
class CRobot
{
    public:
        //---Ctor/Dtor---
        // arStartPose: where the robot begins. This has already had the random
        //   start scatter added by the caller, so the robot itself never has
        //   to think about noise.
        // arRoom:  the walls this robot checks for collisions (and, for the
        //   wall follower, senses). Not owned, just a reference.
        // arNoise: the shared noise source its wheels slip from. Not owned.
        // aBodyColour: the colour the robot's body and trail are drawn in.
        CRobot( const CPose& arStartPose, const CRoom& arRoom,
                CNoiseSource& arNoise, Color aBodyColour );
        virtual ~CRobot();

        //---Simulation---
        // Moves the robot forward by one fixed time step. Does nothing once
        // the robot has finished its lap.
        void Update();

        //---Rendering---
        // Split into two calls so a whole group of robots can have every
        // trail drawn first and every body drawn after, and no robot's body
        // ends up hidden under a trail drawn later.
        void DrawTrail( CRender& arRender ) const;
        void DrawBody( CRender& arRender ) const;

        //---Reporting---
        int GetUpdateCount() const;             // how many steps the robot has taken
        int GetCollisionCount() const;          // how many times it has hit a wall
        bool HasFinishedLap() const;            // whether it has made it all the way around
        virtual std::string Name() const = 0;   // the robot's name, e.g. "Wall follower"

    protected:
        //---Every specific robot has to write these two functions itself---
        virtual void Sense() = 0;              // checks the robot's own sensors
        virtual void SteerFromSensors() = 0;   // works out the wheel speeds from what the sensors found

        //---The only parts a specific robot is allowed to touch. Everything
        //   else stays private, so only CRobot itself can change it.---
        const CPose& Pose() const;             // where the robot currently is
        const CRoom& Room() const;             // the room its sensors check against
        void SetWheelSpeeds( float aLeft, float aRight );   // the only way to change the wheel speeds
        float BaseSpeed() const;               // the wheel speed used when driving straight

    private:
        //---True if the robot would be touching a wall at arTentativePose---
        bool HasCollided( const CPose& arTentativePose ) const;

        //---Marks the lap as finished once the robot has driven well clear of
        //   its start point and then made its way back close to it again---
        void CheckLap();

        //---Adds the robot's current position to its trail, but only once it
        //   has moved a little way from the last point that was kept---
        void ExtendTrail();

        //---How far the robot has to move before another trail point is kept.
        //   Keeping every single step would mean storing and redrawing forty
        //   robots' worth of almost identical points, every single frame.---
        static const float kTrailPointSpacing;

        //---How the lap is judged, measured as distance from the start point---
        static const float kLapLeaveDistance;    // the robot must get at least this far away first
        static const float kLapReturnDistance;   // then come back to within this distance

        //---Fixed measurements and settings, the same for the robot's whole life---
        const float mRadius;      // the robot's radius, fixed at 15 by the spec
        const float mAxleWidth;   // the distance between the two wheels
        const float mBaseSpeed;   // the wheel speed used when driving straight
        const float mTimeStep;    // how many simulated seconds pass in one Update()

        //---What changes as the robot runs---
        CPose mPose;
        CDriveTrain mDriveTrain;
        Color mBodyColour;

        Vec2D mStartPosition;   // where the robot began, used to judge the lap
        bool mHasLeftStart;     // true once the robot has driven far enough away
        bool mLapDone;          // true once it has come back again after that

        std::vector<Vec2D> mTrail;   // one point per step; the ctor adds the very first one

        int mUpdateCount;
        int mCollisionCount;
        bool mWasColliding;   // so a collision is only counted once when it starts,
                              // not on every step the robot stays against the wall

        const CRoom& mrRoom;   // the room this robot checks against; not owned
};

#endif
