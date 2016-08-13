/****************************************************************************/
/// @file    MSRouteProbe.cpp
/// @author  Michael Behrisch
/// @author  Daniel Krajzewicz
/// @author  Tino Morenz
/// @author  Jakob Erdmann
/// @date    Thu, 04.12.2008
/// @version $Id: MSRouteProbe.cpp 21217 2016-07-22 10:57:44Z behrisch $
///
// Writes route distributions at a certain edge
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors
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

#include <string>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSRoute.h>
#include <microsim/MSVehicle.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#include "MSRouteProbe.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSRouteProbe::MSRouteProbe(const std::string& id, const MSEdge* edge, const std::string& distID, const std::string& lastID) :
    MSDetectorFileOutput(id), MSMoveReminder(id) {
    myCurrentRouteDistribution = std::make_pair(distID, MSRoute::distDictionary(distID));
    if (myCurrentRouteDistribution.second == 0) {
        myCurrentRouteDistribution.second = new RandomDistributor<const MSRoute*>();
        MSRoute::dictionary(distID, myCurrentRouteDistribution.second, false);
    }
    myLastRouteDistribution = std::make_pair(lastID, MSRoute::distDictionary(lastID));
    if (MSGlobals::gUseMesoSim) {
        MESegment* seg = MSGlobals::gMesoNet->getSegmentForEdge(*edge);
        while (seg != 0) {
            seg->addDetector(this);
            seg = seg->getNextSegment();
        }
        return;
    }
    for (std::vector<MSLane*>::const_iterator it = edge->getLanes().begin(); it != edge->getLanes().end(); ++it) {
        (*it)->addMoveReminder(this);
    }
}


MSRouteProbe::~MSRouteProbe() {
}


bool
MSRouteProbe::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason) {
    if (reason != MSMoveReminder::NOTIFICATION_SEGMENT && reason != MSMoveReminder::NOTIFICATION_LANE_CHANGE) {
        if (myCurrentRouteDistribution.second->add(1., &veh.getRoute())) {
            veh.getRoute().addReference();
        }
    }
    return false;
}


void
MSRouteProbe::writeXMLOutput(OutputDevice& dev,
                             SUMOTime startTime, SUMOTime stopTime) {
    if (myCurrentRouteDistribution.second->getOverallProb() > 0) {
        dev.openTag("routeDistribution") << " id=\"" << getID() + "_" + time2string(startTime) << "\"";
        const std::vector<const MSRoute*>& routes = myCurrentRouteDistribution.second->getVals();
        const std::vector<SUMOReal>& probs = myCurrentRouteDistribution.second->getProbs();
        for (int j = 0; j < (int)routes.size(); ++j) {
            const MSRoute* r = routes[j];
            dev.openTag("route") << " id=\"" << r->getID() + "_" + time2string(startTime) << "\" edges=\"";
            for (MSRouteIterator i = r->begin(); i != r->end(); ++i) {
                if (i != r->begin()) {
                    dev << " ";
                }
                dev << (*i)->getID();
            }
            dev << "\" probability=\"" << probs[j] << "\"";
            dev.closeTag();
        }
        dev.closeTag();
        if (myLastRouteDistribution.second != 0) {
            MSRoute::checkDist(myLastRouteDistribution.first);
        }
        myLastRouteDistribution = myCurrentRouteDistribution;
        myCurrentRouteDistribution.first = getID() + "_" + toString(stopTime);
        myCurrentRouteDistribution.second = new RandomDistributor<const MSRoute*>();
        MSRoute::dictionary(myCurrentRouteDistribution.first, myCurrentRouteDistribution.second, false);
    }
}


void
MSRouteProbe::writeXMLDetectorProlog(OutputDevice& dev) const {
    dev.writeXMLHeader("route-probes");
}


const MSRoute*
MSRouteProbe::getRoute() const {
    if (myLastRouteDistribution.second == 0) {
        if (myCurrentRouteDistribution.second->getOverallProb() > 0) {
            return myCurrentRouteDistribution.second->get();
        }
        return 0;
    }
    return myLastRouteDistribution.second->get();
}
