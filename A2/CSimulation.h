//-----------------------------------------------------------------------------
// CSimulation.h
//
// A CSimulation owns the whole simulated world: the room, the floor line, and
// both robots. It runs the fixed-timestep loop that updates and draws them.
// main() just needs to build one of these, load the maps, and call Run().
//
// The robots are kept as CRobot pointers because they can only be built once
// the maps have loaded (each one needs a start pose from its map), and having
// them in one list means Update(), Draw() and the summary can all just walk
// over both robots the same way.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CRoom.h"
#include "CFloorLine.h"
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
        // Loads both maps and, if they both load, builds the two robots.
        // Returns false, and prints a message, if either map fails to load.
        bool LoadMaps( const std::string& arWallsMap, const std::string& arLineMap );

        //---Run---
        // Updates and draws every robot until the window is closed, then
        // prints a short summary for each robot to the console.
        void Run( CRender& arRender );

    private:
        //---One frame of the run, broken out of Run() to keep it short---
        void UpdateRobots();                         // moves every robot forward one step
        void DrawFrame( CRender& arRender ) const;   // draws the room, the line, then the robots
        void PrintSummary() const;                   // prints one line per robot

        CRoom mRoom;
        CFloorLine mLine;
        std::vector<std::unique_ptr<CRobot>> mRobots;
};

#endif
