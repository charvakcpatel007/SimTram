/****************************************************************************/
/// @file    MSRailCrossing.cpp
/// @author  Jakob Erdmann
/// @date    Dez 2015
/// @version $Id: MSRailCrossing.cpp 21182 2016-07-18 06:46:01Z behrisch $
///
// A rail signal logic
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

#include <cassert>
#include <utility>
#include <vector>
#include <bitset>
#include <microsim/MSEventControl.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include "MSTrafficLightLogic.h"
#include "MSRailCrossing.h"
#include <microsim/MSLane.h>
#include "MSPhaseDefinition.h"
#include "MSTLLogicControl.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSRailCrossing::MSRailCrossing(MSTLLogicControl& tlcontrol,
                               const std::string& id, const std::string& subid,
                               const std::map<std::string, std::string>& parameters) :
    MSSimpleTrafficLightLogic(tlcontrol, id, subid, Phases(), 0, DELTA_T, parameters),
    // XXX make this configurable
    mySecurityGap(TIME2STEPS(15)),
    myMinGreenTime(TIME2STEPS(5)),
    /// XXX compute reasonable time depending on link length
    myYellowTime(TIME2STEPS(5)) {
    // dummy phase, used to avoid crashing in MSTrafficLightLogic::setTrafficLightSignals()
    myPhases.push_back(new MSPhaseDefinition(1, 1, 1, std::string(SUMO_MAX_CONNECTIONS, 'X')));
}

void
MSRailCrossing::init(NLDetectorBuilder&) {
    delete myPhases.front();
    myPhases.clear();
    myPhases.push_back(new MSPhaseDefinition(1, 1, 1, std::string(myLinks.size(), 'G')));
    myPhases.push_back(new MSPhaseDefinition(myYellowTime, myYellowTime, myYellowTime, std::string(myLinks.size(), 'y')));
    myPhases.push_back(new MSPhaseDefinition(1, 1, 1, std::string(myLinks.size(), 'r')));
    // init phases
    updateCurrentPhase();
    setTrafficLightSignals(MSNet::getInstance()->getCurrentTimeStep());
}


MSRailCrossing::~MSRailCrossing() {}


// ----------- Handling of controlled links
void
MSRailCrossing::adaptLinkInformationFrom(const MSTrafficLightLogic& logic) {
    MSTrafficLightLogic::adaptLinkInformationFrom(logic);
    updateCurrentPhase();
}


// ------------ Switching and setting current rows
SUMOTime
MSRailCrossing::trySwitch() {
    SUMOTime nextTry = updateCurrentPhase();
    setTrafficLightSignals(MSNet::getInstance()->getCurrentTimeStep());
    //if (getID() == "cluster_1088529493_1260626727") std::cout << " myStep=" << myStep << " nextTry=" << nextTry << "\n";
    return nextTry;
}


SUMOTime
MSRailCrossing::updateCurrentPhase() {
    const SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    SUMOTime stayRedUntil = now;
    // check rail links for approaching foes to determine whether and how long
    // the crossing must remain closed
    for (std::vector<MSLink*>::const_iterator it_link = myIncomingRailLinks.begin(); it_link != myIncomingRailLinks.end(); ++it_link) {
        for (std::map<const SUMOVehicle*, MSLink::ApproachingVehicleInformation>::const_iterator
                it_avi = (*it_link)->getApproaching().begin();
                it_avi != (*it_link)->getApproaching().end(); ++it_avi) {
            const MSLink::ApproachingVehicleInformation& avi = it_avi->second;
            if (avi.arrivalTime - myYellowTime - now < mySecurityGap) {
                stayRedUntil = MAX2(stayRedUntil, avi.leavingTime);
            }
        }
#ifdef HAVE_INTERNAL_LANES
        if ((*it_link)->getViaLane() != 0 && (*it_link)->getViaLane()->getVehicleNumberWithPartials() > 0) {
            // do not open if there is still a train on the crossing
            stayRedUntil = MAX2(stayRedUntil, now + DELTA_T);
        }
#endif
    }
    //if (getID() == "cluster_1088529493_1260626727") std::cout << SIMTIME << " stayRedUntil=" << stayRedUntil;
    const SUMOTime wait = stayRedUntil - now;

    if (myStep == 0) {
        // 'G': check whether the crossing can stay open
        if (wait == 0) {
            return DELTA_T;
        } else {
            myStep++;
            return myYellowTime;
        }
    } else if (myStep == 1) {
        // 'y': yellow time is over. switch to red
        myStep++;
        return MAX2(DELTA_T, wait);
    } else {
        // 'r': check whether we may open again
        if (wait == 0) {
            myStep = 0;
            return myMinGreenTime;
        } else {
            return wait;
        }
    }
}


// ------------ Conversion between time and phase
SUMOTime
MSRailCrossing::getPhaseIndexAtTime(SUMOTime) const {
    return 0;
}

SUMOTime
MSRailCrossing::getOffsetFromIndex(int) const {
    return 0;
}

int
MSRailCrossing::getIndexFromOffset(SUMOTime) const {
    return 0;
}


void
MSRailCrossing::addLink(MSLink* link, MSLane* lane, int pos) {
    if (pos >= 0) {
        MSTrafficLightLogic::addLink(link, lane, pos);
    } else {
        myIncomingRailLinks.push_back(link);
    }
}


/****************************************************************************/

