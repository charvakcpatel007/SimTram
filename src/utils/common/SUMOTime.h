/****************************************************************************/
/// @file    SUMOTime.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Fri, 29.04.2005
/// @version $Id: SUMOTime.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Variables, methods, and tools for internal time representation
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2001-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef SUMOTime_h
#define SUMOTime_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <limits>
#include <string>
#include "UtilExceptions.h"


// ===========================================================================
// type definitions
// ===========================================================================
typedef long long int SUMOTime;
#define SUMOTime_MAX std::numeric_limits<SUMOTime>::max()
#define SUMOTime_MIN std::numeric_limits<SUMOTime>::min()
#define SUMOTIME_MAXSTRING "9223372036854774" // SUMOTime_MAX / 1000 - 1 (because of rounding errors)

// the step length in ms
extern SUMOTime DELTA_T;

// the step length in seconds as SUMOReal
#define TS (static_cast<SUMOReal>(DELTA_T/1000.))

// x*deltaT
#define SPEED2DIST(x) ((x)*TS)
// x/deltaT
#define DIST2SPEED(x) ((x)/TS)
// x*deltaT*deltaT
#define ACCEL2DIST(x) ((x)*TS*TS)
// x*deltaT
#define ACCEL2SPEED(x) ((x)*TS)
// x*deltaT
#define SPEED2ACCEL(x) ((x)/TS)

#define STEPS2TIME(x) (static_cast<SUMOReal>((x)/1000.))
#define TIME2STEPS(x) (static_cast<SUMOTime>((x)*1000))
#define STEPFLOOR(x) (int(x/DELTA_T)*DELTA_T)
#define STEPS2MS(x) (x)

#define SIMTIME STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep())

// ===========================================================================
// method declarations
// ===========================================================================
SUMOTime string2time(const std::string& r);
std::string time2string(SUMOTime t);


#endif

/****************************************************************************/

