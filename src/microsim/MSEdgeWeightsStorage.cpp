/****************************************************************************/
/// @file    MSEdgeWeightsStorage.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @date    02.11.2009
/// @version $Id: MSEdgeWeightsStorage.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// A storage for edge travel times and efforts
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

#include "MSEdgeWeightsStorage.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSEdgeWeightsStorage::MSEdgeWeightsStorage() {
}


MSEdgeWeightsStorage::~MSEdgeWeightsStorage() {
}


bool
MSEdgeWeightsStorage::retrieveExistingTravelTime(const MSEdge* const e, const SUMOReal t, SUMOReal& value) const {
    std::map<const MSEdge*, ValueTimeLine<SUMOReal> >::const_iterator i = myTravelTimes.find(e);
    if (i == myTravelTimes.end()) {
        return false;
    }
    const ValueTimeLine<SUMOReal>& tl = (*i).second;
    if (!tl.describesTime(t)) {
        return false;
    }
    value = tl.getValue(t);
    return true;
}


bool
MSEdgeWeightsStorage::retrieveExistingEffort(const MSEdge* const e, const SUMOReal t, SUMOReal& value) const {
    std::map<const MSEdge*, ValueTimeLine<SUMOReal> >::const_iterator i = myEfforts.find(e);
    if (i == myEfforts.end()) {
        return false;
    }
    const ValueTimeLine<SUMOReal>& tl = (*i).second;
    if (!tl.describesTime(t)) {
        return false;
    }
    value = tl.getValue(t);
    return true;
}


void
MSEdgeWeightsStorage::addTravelTime(const MSEdge* const e,
                                    SUMOReal begin, SUMOReal end,
                                    SUMOReal value) {
    std::map<const MSEdge*, ValueTimeLine<SUMOReal> >::iterator i = myTravelTimes.find(e);
    if (i == myTravelTimes.end()) {
        myTravelTimes[e] = ValueTimeLine<SUMOReal>();
        i = myTravelTimes.find(e);
    }
    (*i).second.add(begin, end, value);
}


void
MSEdgeWeightsStorage::addEffort(const MSEdge* const e,
                                SUMOReal begin, SUMOReal end,
                                SUMOReal value) {
    std::map<const MSEdge*, ValueTimeLine<SUMOReal> >::iterator i = myEfforts.find(e);
    if (i == myEfforts.end()) {
        myEfforts[e] = ValueTimeLine<SUMOReal>();
        i = myEfforts.find(e);
    }
    (*i).second.add(begin, end, value);
}


void
MSEdgeWeightsStorage::removeTravelTime(const MSEdge* const e) {
    std::map<const MSEdge*, ValueTimeLine<SUMOReal> >::iterator i = myTravelTimes.find(e);
    if (i != myTravelTimes.end()) {
        myTravelTimes.erase(i);
    }
}


void
MSEdgeWeightsStorage::removeEffort(const MSEdge* const e) {
    std::map<const MSEdge*, ValueTimeLine<SUMOReal> >::iterator i = myEfforts.find(e);
    if (i != myEfforts.end()) {
        myEfforts.erase(i);
    }
}


bool
MSEdgeWeightsStorage::knowsTravelTime(const MSEdge* const e) const {
    return myTravelTimes.find(e) != myTravelTimes.end();
}


bool
MSEdgeWeightsStorage::knowsEffort(const MSEdge* const e) const {
    return myEfforts.find(e) != myEfforts.end();
}



/****************************************************************************/

