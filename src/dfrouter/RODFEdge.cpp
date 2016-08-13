/****************************************************************************/
/// @file    RODFEdge.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @author  Yun-Pang Floetteroed
/// @date    Thu, 16.03.2006
/// @version $Id: RODFEdge.cpp 21182 2016-07-18 06:46:01Z behrisch $
///
// An edge within the DFROUTER
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2006-2016 DLR (http://www.dlr.de/) and contributors
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
#include <utils/common/MsgHandler.h>
#include "RODFEdge.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
RODFEdge::RODFEdge(const std::string& id, RONode* from, RONode* to, int index, const int priority)
    : ROEdge(id, from, to, index, priority) {}


RODFEdge::~RODFEdge() {}


void
RODFEdge::setFlows(const std::vector<FlowDef>& flows) {
    myFlows = flows;
}


const std::vector<FlowDef>&
RODFEdge::getFlows() const {
    return myFlows;
}


/****************************************************************************/

