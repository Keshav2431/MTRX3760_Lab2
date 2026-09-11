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
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CRender.h"       // CRender, Color, Vec2D
#include "CLoopReader.h"   // CPose
#include "CDriveTrain.h"

#include <string>
#include <vector>

class CRoom;   // forward declaration: held by const reference, not owned

//-----------------------------------------------------------------------------
class CRobot
{
    public:
        //---Ctor/Dtor---
        // arStartPose:  where the robot begins.
        // arRoom:  the room this robot checks for walls (and, for the wall
        //   follower, senses against). Not owned, just a reference.
        // aBodyColour:  the colour the robot's body and trail are drawn in.
        CRobot( const CPose& arStartPose, const CRoom& arRoom, Color aBodyColour );
        virtual ~CRobot();

        //---Simulation---
        void Update();   // moves the robot forward by one fixed time step

        //---Rendering---
        void Draw( CRender& arRender ) const;   // draws the trail, the body, and which way it's facing

        //---Reporting---
        int GetUpdateCount() const;             // how many steps the robot has taken
        int GetCollisionCount() const;          // how many times it has hit a wall
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

        //---Fixed measurements and settings, the same for the robot's whole life---
        const float mRadius;      // the robot's radius, fixed at 15 by the spec
        const float mAxleWidth;   // the distance between the two wheels
        const float mBaseSpeed;   // the wheel speed used when driving straight
        const float mTimeStep;    // how many simulated seconds pass in one Update()

        //---What changes as the robot runs---
        CPose mPose;
        CDriveTrain mDriveTrain;
        Color mBodyColour;

        std::vector<Vec2D> mTrail;   // one point per step; the ctor adds the very first one

        int mUpdateCount;
        int mCollisionCount;
        bool mWasColliding;   // so a collision is only counted once when it starts,
                              // not on every step the robot stays against the wall

        const CRoom& mrRoom;   // the room this robot checks against; not owned
};

#endif
