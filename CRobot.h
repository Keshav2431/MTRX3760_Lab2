//-----------------------------------------------------------------------------
// CRobot.h
//
// The common half of every robot in the simulator: it owns the pose, the
// drive train, the trail, the update and collision counts, and it runs the
// fixed-timestep Update() loop and the drawing. What it does NOT know is how
// this particular robot senses the world or how it steers - those are the two
// pure-virtual hooks a subclass fills in.
//
//   Update()  ->  Sense()            (subclass reads its own sensors)
//             ->  SteerFromSensors() (subclass sets wheel speeds)
//             ->  advance one timestep, reject the move on collision
//             ->  extend the trail, bump the counters
//
// CWallFollowerRobot and CLineFollowerRobot differ only in those two hooks and
// in which sensors they carry.
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
        // arStartPose: where the robot begins.
        // arRoom: the room this robot collides against (and, for the wall
        //   follower, senses against). Not owned; only a reference is kept.
        // aBodyColour: the colour the body disc is drawn in.
        CRobot( const CPose& arStartPose, const CRoom& arRoom, Color aBodyColour );
        virtual ~CRobot() = default;

        //---Simulation---
        // Advances the robot by one fixed simulated time step.
        void Update();

        //---Rendering---
        void Draw( CRender& arRender ) const;

        //---Reporting---
        int GetUpdateCount() const;
        int GetCollisionCount() const;
        virtual std::string Name() const = 0;

    protected:
        //---Hooks every concrete robot must supply---
        virtual void Sense() = 0;              // read this robot's own sensors
        virtual void SteerFromSensors() = 0;   // set wheel speeds from those readings

        //---The only state a subclass needs to reach. Kept protected (not
        //   public, and the rest of the state private) so that only the robot
        //   subclasses can touch it.---
        const CPose& Pose() const;             // sensors need the current pose
        const CRoom& Room() const;             // range sensors sense against it
        void SetWheelSpeeds( float aLeft, float aRight );   // the only drive path
        float BaseSpeed() const;               // straight-line wheel speed

    private:
        //---Collision test---
        bool HasCollided( const CPose& arTentativePose ) const;

        //---Geometry and tuning, fixed for the life of the robot---
        const float mRadius;      // body radius; fixed by the spec at 15
        const float mAxleWidth;   // distance between the two wheels
        const float mBaseSpeed;   // wheel speed when driving straight
        const float mTimeStep;    // simulated seconds advanced per Update()

        //---State---
        CPose mPose;
        CDriveTrain mDriveTrain;
        Color mBodyColour;

        std::vector<Vec2D> mTrail;

        int mUpdateCount;
        int mCollisionCount;
        bool mWasColliding;   // so a collision is counted once on entry, not
                              // every frame the robot stays against the wall

        const CRoom& mrRoom;   // sensed/collided against; not owned
};

#endif
