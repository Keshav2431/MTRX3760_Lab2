//-----------------------------------------------------------------------------
// CSimulation.h
//
// Owns the whole simulated world - the room, the floor line, and both robots -
// and runs the fixed-timestep sense/draw loop over them. main() just builds
// one of these, loads the maps, and calls Run().
//
// Robots are held as CRobot pointers so they can be built only after their
// maps have loaded (each needs a start pose), and so Update(), Draw() and the
// summary can walk them uniformly.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CRoom.h"
#include "CFloorLine.h"
#include "CRobot.h"
#include "CRender.h"

#include <memory>
#include <string>
#include <vector>

//-----------------------------------------------------------------------------
class CSimulation
{
    public:
        //---Ctor---
        CSimulation();

        //---Setup---
        // Loads both maps and, on success, builds the two robots. False (with
        // a console message) if either map fails to load.
        bool LoadMaps( const std::string& arWallsMap, const std::string& arLineMap );

        //---Run---
        // Steps and draws every robot until the window closes, then prints a
        // per-robot summary to the console.
        void Run( CRender& arRender );

    private:
        CRoom mRoom;
        CFloorLine mLine;
        std::vector<std::unique_ptr<CRobot>> mRobots;
};

#endif
