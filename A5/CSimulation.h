//-----------------------------------------------------------------------------
// CSimulation.h
//
// A CSimulation owns the whole simulated world: the room, the floor line, the
// noise source, and both groups of robots. It runs the fixed-timestep loop
// that updates and draws them. main() just needs to build one of these, load
// the maps, and call Run().
//
// This differs from the plain A2 simulation in three ways, all handled here:
// there are twenty wall followers and twenty line followers instead of one of
// each, held as two groups; every robot's start pose is the map's start pose
// with a small random scatter added, so no two robots begin in exactly the
// same spot; and the run stops itself once every robot has finished its lap
// (or a step limit is hit), then holds that finished picture and prints the
// summary, so the report screenshot and console output can be captured at the
// same moment.
//
// The noise source is owned here and shared with every robot, so one seed
// decides the whole run.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CRoom.h"
#include "CFloorLine.h"
#include "CNoiseSource.h"
#include "CRobot.h"

#include <memory>
#include <string>
#include <vector>

class CRender;   // forward declaration: Run() takes it by reference only

//-----------------------------------------------------------------------------
class CSimulation
{
    public:
        //---Ctor---
        CSimulation();

        //---Setup---
        // Loads both maps and, if they both load, builds both groups of
        // robots. Returns false, and prints a message, if either map fails.
        bool LoadMaps( const std::string& arWallsMap, const std::string& arLineMap );

        //---Run---
        // Updates and draws every robot until the run is finished, then keeps
        // showing the finished picture until the window is closed.
        void Run( CRender& arRender );

    private:
        //---A group of robots of the same type. The simulation keeps two.---
        typedef std::vector<std::unique_ptr<CRobot>> CRobotGroup;

        //---Setup---
        void BuildRobots();                        // fills both groups of robots
        float ShadeForRobot( int aIndex ) const;   // how much to lighten robot number aIndex, 0 to 1

        //---One frame of the run---
        void StepGroup( CRobotGroup& arGroup );                                 // moves every robot in the group forward one step
        void DrawTrails( const CRobotGroup& arGroup, CRender& arRender ) const; // draws every trail in the group
        void DrawBodies( const CRobotGroup& arGroup, CRender& arRender ) const; // draws every body in the group
        void DrawFrame( CRender& arRender ) const;                              // draws the whole picture, one frame

        //---Working out when the run is finished, and reporting the result---
        bool RunFinished() const;                                // true once both groups are done, or time runs out
        bool GroupFinished( const CRobotGroup& arGroup ) const;  // true once every robot in the group has finished
        int CountFinished( const CRobotGroup& arGroup ) const;   // how many robots in the group have finished
        void ReportGroup( const CRobotGroup& arGroup ) const;    // prints one summary line for the group
        void PrintSummaryOnce();                                 // prints the whole summary, but only the first time

        //---Fixed for every run---
        static const int kRobotsPerType;      // twenty of each robot type, as the brief asks for
        static const int kMaxSteps;           // gives up on a robot that never makes it back
        static const unsigned int kNoiseSeed; // fixed, so a run can be repeated exactly
        static const float kMaxLighten;       // how much lighter the last robot's colour can get

        CRoom mRoom;
        CFloorLine mLine;

        CNoiseSource mNoise;   // shared with every robot; owned here

        CRobotGroup mWallFollowers;
        CRobotGroup mLineFollowers;

        int mStepCount;
        bool mSummaryPrinted;
};

#endif
