/****************************************************************************/
/// @file    NBTrafficLightLogic.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: NBTrafficLightLogic.cpp 21210 2016-07-21 10:02:38Z behrisch $
///
// A SUMO-compliant built logic for a traffic light
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

#include <vector>
#include <bitset>
#include <utility>
#include <string>
#include <sstream>
#include <cassert>
#include "NBEdge.h"
#include "NBEdgeCont.h"
#include "NBTrafficLightLogic.h"
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/ToString.h>
#include <utils/common/StringTokenizer.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// static members
// ===========================================================================
const char NBTrafficLightLogic::allowedStatesInitializer[] = {LINKSTATE_TL_GREEN_MAJOR,
                                                              LINKSTATE_TL_GREEN_MINOR,
                                                              LINKSTATE_STOP, // used for NODETYPE_TRAFFIC_LIGHT_RIGHT_ON_RED
                                                              LINKSTATE_TL_RED,
                                                              LINKSTATE_TL_REDYELLOW,
                                                              LINKSTATE_TL_YELLOW_MAJOR,
                                                              LINKSTATE_TL_YELLOW_MINOR,
                                                              LINKSTATE_TL_OFF_BLINKING,
                                                              LINKSTATE_TL_OFF_NOSIGNAL
                                                             };

const std::string NBTrafficLightLogic::ALLOWED_STATES(NBTrafficLightLogic::allowedStatesInitializer, 9);

// ===========================================================================
// member method definitions
// ===========================================================================
NBTrafficLightLogic::NBTrafficLightLogic(const std::string& id,
        const std::string& subid, int noLinks,
        SUMOTime offset, TrafficLightType type) :
    Named(id), myNumLinks(noLinks), mySubID(subid),
    myOffset(offset),
    myType(type) {}

NBTrafficLightLogic::NBTrafficLightLogic(const NBTrafficLightLogic* logic) :
    Named(logic->getID()),
    myNumLinks(logic->myNumLinks),
    mySubID(logic->getProgramID()),
    myOffset(logic->getOffset()),
    myPhases(logic->myPhases.begin(), logic->myPhases.end()),
    myType(logic->getType()) {}


NBTrafficLightLogic::~NBTrafficLightLogic() {}


void
NBTrafficLightLogic::addStep(SUMOTime duration, const std::string& state, int index) {
    // check state size
    if (myNumLinks == 0) {
        // initialize
        myNumLinks = (int)state.size();
    } else if ((int)state.size() != myNumLinks) {
        throw ProcessError("When adding phase to tlLogic '" + getID() + "': state length of " + toString(state.size()) +
                           " does not match declared number of links " + toString(myNumLinks));
    }
    // check state contents
    const std::string::size_type illegal = state.find_first_not_of(ALLOWED_STATES);
    if (std::string::npos != illegal) {
        throw ProcessError("When adding phase: illegal character '" + toString(state[illegal]) + "' in state");
    }
    // interpret index
    if (index < 0 || index >= (int)myPhases.size()) {
        // insert at the end
        index = (int)myPhases.size();
    }
    myPhases.insert(myPhases.begin() + index, PhaseDefinition(duration, state));
}


void
NBTrafficLightLogic::deletePhase(int index) {
    if (index >= (int)myPhases.size()) {
        throw InvalidArgument("Index " + toString(index) + " out of range for logic with "
                              + toString(myPhases.size()) + " phases.");
    }
    myPhases.erase(myPhases.begin() + index);
}


void
NBTrafficLightLogic::resetPhases() {
    myNumLinks = 0;
    myPhases.clear();
}


SUMOTime
NBTrafficLightLogic::getDuration() const {
    SUMOTime duration = 0;
    for (PhaseDefinitionVector::const_iterator i = myPhases.begin(); i != myPhases.end(); ++i) {
        duration += (*i).duration;
    }
    return duration;
}


void
NBTrafficLightLogic::closeBuilding() {
    for (int i = 0; i < (int)myPhases.size() - 1;) {
        if (myPhases[i].state != myPhases[i + 1].state) {
            ++i;
            continue;
        }
        myPhases[i].duration += myPhases[i + 1].duration;
        myPhases.erase(myPhases.begin() + i + 1);
    }
}


void
NBTrafficLightLogic::setPhaseState(int phaseIndex, int tlIndex, LinkState linkState) {
    assert(phaseIndex < (int)myPhases.size());
    std::string& phaseState = myPhases[phaseIndex].state;
    assert(tlIndex < (int)phaseState.size());
    phaseState[tlIndex] = (char)linkState;
}


void
NBTrafficLightLogic::setPhaseDuration(int phaseIndex, SUMOTime duration) {
    assert(phaseIndex < (int)myPhases.size());
    myPhases[phaseIndex].duration = duration;
}


/****************************************************************************/

