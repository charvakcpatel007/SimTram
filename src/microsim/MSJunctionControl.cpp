/****************************************************************************/
/// @file    MSJunctionControl.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, 06 Mar 2001
/// @version $Id: MSJunctionControl.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// Container for junctions; performs operations on all stored junctions
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


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <algorithm>
#include "MSInternalJunction.h"
#include "MSJunctionControl.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
MSJunctionControl::MSJunctionControl() {
}


MSJunctionControl::~MSJunctionControl() {
}


void
MSJunctionControl::postloadInitContainer() {
    const std::vector<MSJunction*>& junctions = buildAndGetStaticVector();
#ifdef HAVE_INTERNAL_LANES
    // initialize normal junctions before internal junctions
    // (to allow calling getIndex() during initialization of internal junction links)
    for (std::vector<MSJunction*>::const_iterator i = junctions.begin(); i != junctions.end(); ++i) {
        if (dynamic_cast<MSInternalJunction*>(*i) == 0) {
            (*i)->postloadInit();
        }
    }
    for (std::vector<MSJunction*>::const_iterator i = junctions.begin(); i != junctions.end(); ++i) {
        if (dynamic_cast<MSInternalJunction*>(*i) != 0) {
            (*i)->postloadInit();
        }
    }
#else
    for (std::vector<MSJunction*>::const_iterator i = junctions.begin(); i != junctions.end(); ++i) {
        (*i)->postloadInit();
    }
#endif
}


/****************************************************************************/

