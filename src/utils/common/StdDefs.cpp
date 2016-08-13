/****************************************************************************/
/// @file    StdDefs.cpp
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    2014-01-07
/// @version $Id: StdDefs.cpp 20550 2016-04-26 10:57:45Z namdre $
///
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2014-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include "RGBColor.h"
#include "StdDefs.h"


#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

/* -------------------------------------------------------------------------
 * color constants for link states
 * ----------------------------------------------------------------------- */
const RGBColor SUMO_color_TL_GREEN_MAJOR(0, 255, 0);
const RGBColor SUMO_color_TL_GREEN_MINOR(0, 179, 0);
const RGBColor SUMO_color_TL_RED(255, 0, 0);
const RGBColor SUMO_color_TL_REDYELLOW(255, 128, 0);
const RGBColor SUMO_color_TL_YELLOW_MAJOR(255, 255, 0);
const RGBColor SUMO_color_TL_YELLOW_MINOR(255, 255, 0);
const RGBColor SUMO_color_TL_OFF_BLINKING(128, 64, 0);
const RGBColor SUMO_color_TL_OFF_NOSIGNAL(0, 255, 255);
const RGBColor SUMO_color_MAJOR(255, 255, 255);
const RGBColor SUMO_color_MINOR(51, 51, 51);
const RGBColor SUMO_color_EQUAL(128, 128, 128);
const RGBColor SUMO_color_STOP(128, 0, 128);
const RGBColor SUMO_color_ALLWAY_STOP(0, 0, 192);
const RGBColor SUMO_color_ZIPPER(192, 128, 64);
const RGBColor SUMO_color_DEADEND(0, 0, 0);

const RGBColor& getLinkColor(const LinkState& ls) {
    switch (ls) {
        case LINKSTATE_TL_GREEN_MAJOR:
            return SUMO_color_TL_GREEN_MAJOR;
        case LINKSTATE_TL_GREEN_MINOR:
            return SUMO_color_TL_GREEN_MINOR;
        case LINKSTATE_TL_RED:
            return SUMO_color_TL_RED;
        case LINKSTATE_TL_REDYELLOW:
            return SUMO_color_TL_REDYELLOW;
        case LINKSTATE_TL_YELLOW_MAJOR:
            return SUMO_color_TL_YELLOW_MAJOR;
        case LINKSTATE_TL_YELLOW_MINOR:
            return SUMO_color_TL_YELLOW_MINOR;
        case LINKSTATE_TL_OFF_BLINKING:
            return SUMO_color_TL_OFF_BLINKING;
        case LINKSTATE_TL_OFF_NOSIGNAL:
            return SUMO_color_TL_OFF_NOSIGNAL;
        case LINKSTATE_MAJOR:
            return SUMO_color_MAJOR;
        case LINKSTATE_MINOR:
            return SUMO_color_MINOR;
        case LINKSTATE_EQUAL:
            return SUMO_color_EQUAL;
        case LINKSTATE_STOP:
            return SUMO_color_STOP;
        case LINKSTATE_ALLWAY_STOP:
            return SUMO_color_ALLWAY_STOP;
        case LINKSTATE_ZIPPER:
            return SUMO_color_ZIPPER;
        case LINKSTATE_DEADEND:
            return SUMO_color_DEADEND;
        default:
            throw ProcessError("No color defined for LinkState '" + std::string(ls, 1) + "'");
    }
}


bool gDebugFlag1 = false;
bool gDebugFlag2 = false;
bool gDebugFlag3 = false;
bool gDebugFlag4 = false;
std::string gDebugSelectedVehicle = "";

/****************************************************************************/

