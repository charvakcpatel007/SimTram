/****************************************************************************/
/// @file    MSGlobals.h
/// @author  Daniel Krajzewicz
/// @author  Christian Roessel
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    late summer 2003
/// @version $Id: MSGlobals.h 20550 2016-04-26 10:57:45Z namdre $
///
// Some static variables for faster access
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2003-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSGlobals_h
#define MSGlobals_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <map>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MELoop;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSGlobals
 * This class holds some static variables, filled mostly with values coming
 *  from the command line or the simulation configuration file.
 * They are stored herein to allow a faster access than from the options
 *  container.
 */
class MSGlobals {
public:
    /// Information whether empty edges shall be written on dump
    static bool gOmitEmptyEdgesOnDump;

    /** Information how long the simulation shall wait until it recognizes
        a vehicle as a grid lock participant */
    static SUMOTime gTimeToGridlock;

    /** The time to detect grid locks on highways */
    static SUMOTime gTimeToGridlockHighways;

    /// Information whether the simulation regards internal lanes
    static bool gUsingInternalLanes;

    /// Information whether the simulation regards internal lanes
    static SUMOTime gIgnoreJunctionBlocker;

    /** information whether the network shall check for collisions */
    static bool gCheck4Accidents;

    /** information whether the routes shall be checked for connectivity */
    static bool gCheckRoutes;

    /** information Duration of a lane change maneuver */
    static SUMOTime gLaneChangeDuration;

    /** Lateral resolution within a lane */
    static SUMOReal gLateralResolution;

    /// Information whether a state has been loaded
    static bool gStateLoaded;

    /** Information whether mesosim shall be used */
    static bool gUseMesoSim;

    /** Information whether limited junction control shall be used */
    static bool gMesoLimitedJunctionControl;

    /** Information whether overtaking is enabled in the mesoscopic simulation */
    static bool gMesoOvertaking;

    /** scaling factor for macroscopic time penalty when passing tls controlled intersection */
    static SUMOReal gMesoTLSPenalty;

    /// mesoscopic simulation infrastructure
    static MELoop* gMesoNet;

    /// length of memory for waiting times (in millisecs)
    static SUMOTime gWaitingTimeMemory;

};


#endif

/****************************************************************************/

