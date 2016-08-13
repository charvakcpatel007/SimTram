/****************************************************************************/
/// @file    NBLoadedTLDef.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Tue, 29.05.2005
/// @version $Id: NBLoadedTLDef.cpp 21210 2016-07-21 10:02:38Z behrisch $
///
// A loaded (complete) traffic light logic
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
#include <set>
#include <cassert>
#include <iterator>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/options/OptionsCont.h>
#include "NBTrafficLightLogic.h"
#include "NBTrafficLightDefinition.h"
#include "NBLoadedTLDef.h"
#include "NBNode.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * NBLoadedTLDef::SignalGroup-methods
 * ----------------------------------------------------------------------- */
NBLoadedTLDef::SignalGroup::SignalGroup(const std::string& id)
    : Named(id) {}

NBLoadedTLDef::SignalGroup::~SignalGroup() {}

void
NBLoadedTLDef::SignalGroup::addConnection(const NBConnection& c) {
    assert(c.getFromLane() < 0 || c.getFrom()->getNumLanes() > c.getFromLane());
    myConnections.push_back(c);
}


void
NBLoadedTLDef::SignalGroup::addPhaseBegin(SUMOTime time, TLColor color) {
    myPhases.push_back(PhaseDef(time, color));
}


void
NBLoadedTLDef::SignalGroup::setYellowTimes(SUMOTime tRedYellow, SUMOTime tYellow) {
    myTRedYellow = tRedYellow;
    myTYellow = tYellow;
}


void
NBLoadedTLDef::SignalGroup::sortPhases() {
    sort(myPhases.begin(), myPhases.end(), phase_by_time_sorter());
}


void
NBLoadedTLDef::SignalGroup::patchTYellow(int tyellow, bool forced) {
    if (myTYellow < 0) {
        // was not set before (was not loaded)
        myTYellow = tyellow;
    } else if (forced && myTYellow < tyellow) {
        WRITE_WARNING("TYellow of signal group '" + getID() + "' was less than the computed one; patched (was:" + toString(myTYellow) + ", is:" + toString(tyellow) + ")");
        myTYellow = tyellow;
    }
}


std::vector<SUMOReal>
NBLoadedTLDef::SignalGroup::getTimes(SUMOTime cycleDuration) const {
    // within the phase container, we should have the green and red phases add their times
    std::vector<SUMOReal> ret; // !!! time vector
    for (std::vector<PhaseDef>::const_iterator i = myPhases.begin(); i != myPhases.end(); i++) {
        ret.push_back((SUMOReal)(*i).myTime);
    }
    // further, we possibly should set the yellow phases
    if (myTYellow > 0) {
        for (std::vector<PhaseDef>::const_iterator i = myPhases.begin(); i != myPhases.end(); i++) {
            if ((*i).myColor == TLCOLOR_RED) {
                SUMOTime time = (SUMOTime)(*i).myTime + myTYellow;
                if (time > cycleDuration) {
                    time = time - cycleDuration;
                }
                ret.push_back((SUMOReal) time);
            }
        }
    }
    return ret;
}


int
NBLoadedTLDef::SignalGroup::getLinkNo() const {
    return (int) myConnections.size();
}


bool
NBLoadedTLDef::SignalGroup::mayDrive(SUMOTime time) const {
    assert(myPhases.size() != 0);
    for (std::vector<PhaseDef>::const_reverse_iterator i = myPhases.rbegin(); i != myPhases.rend(); i++) {
        SUMOTime nextTime = (*i).myTime;
        if (time >= nextTime) {
            return (*i).myColor == TLCOLOR_GREEN;
        }
    }
    return (*(myPhases.end() - 1)).myColor == TLCOLOR_GREEN;
}


bool
NBLoadedTLDef::SignalGroup::hasYellow(SUMOTime time) const {
    bool has_red_now = !mayDrive(time);
    bool had_green = mayDrive(time - myTYellow);
    return has_red_now && had_green;
}


bool
NBLoadedTLDef::SignalGroup::containsConnection(NBEdge* from, NBEdge* to) const {
    for (NBConnectionVector::const_iterator i = myConnections.begin(); i != myConnections.end(); i++) {
        if ((*i).getFrom() == from && (*i).getTo() == to) {
            return true;
        }
    }
    return false;

}


const NBConnection&
NBLoadedTLDef::SignalGroup::getConnection(int pos) const {
    assert(pos < (int)myConnections.size());
    return myConnections[pos];
}


bool
NBLoadedTLDef::SignalGroup::containsIncoming(NBEdge* from) const {
    for (NBConnectionVector::const_iterator i = myConnections.begin(); i != myConnections.end(); i++) {
        if ((*i).getFrom() == from) {
            return true;
        }
    }
    return false;
}


void
NBLoadedTLDef::SignalGroup::remapIncoming(NBEdge* which, const EdgeVector& by) {
    NBConnectionVector newConns;
    for (NBConnectionVector::iterator i = myConnections.begin(); i != myConnections.end();) {
        if ((*i).getFrom() == which) {
            NBConnection conn((*i).getFrom(), (*i).getTo());
            i = myConnections.erase(i);
            for (EdgeVector::const_iterator j = by.begin(); j != by.end(); j++) {
                NBConnection curr(conn);
                if (!curr.replaceFrom(which, *j)) {
                    throw ProcessError("Could not replace edge '" + which->getID() + "' by '" + (*j)->getID() + "'.\nUndefined...");
                }
                newConns.push_back(curr);
            }
        } else {
            i++;
        }
    }
    copy(newConns.begin(), newConns.end(),
         back_inserter(myConnections));
}


bool
NBLoadedTLDef::SignalGroup::containsOutgoing(NBEdge* to) const {
    for (NBConnectionVector::const_iterator i = myConnections.begin(); i != myConnections.end(); i++) {
        if ((*i).getTo() == to) {
            return true;
        }
    }
    return false;
}


void
NBLoadedTLDef::SignalGroup::remapOutgoing(NBEdge* which, const EdgeVector& by) {
    NBConnectionVector newConns;
    for (NBConnectionVector::iterator i = myConnections.begin(); i != myConnections.end();) {
        if ((*i).getTo() == which) {
            NBConnection conn((*i).getFrom(), (*i).getTo());
            i = myConnections.erase(i);
            for (EdgeVector::const_iterator j = by.begin(); j != by.end(); j++) {
                NBConnection curr(conn);
                if (!curr.replaceTo(which, *j)) {
                    throw ProcessError("Could not replace edge '" + which->getID() + "' by '" + (*j)->getID() + "'.\nUndefined...");
                }
                newConns.push_back(curr);
            }
        } else {
            i++;
        }
    }
    copy(newConns.begin(), newConns.end(),
         back_inserter(myConnections));
}


void
NBLoadedTLDef::SignalGroup::remap(NBEdge* removed, int removedLane,
                                  NBEdge* by, int byLane) {
    for (NBConnectionVector::iterator i = myConnections.begin(); i != myConnections.end(); i++) {
        if ((*i).getTo() == removed
                &&
                ((*i).getToLane() == removedLane
                 ||
                 (*i).getToLane() == -1)) {
            (*i).replaceTo(removed, removedLane, by, byLane);

        } else if ((*i).getTo() == removed && removedLane == -1) {
            (*i).replaceTo(removed, by);
        }

        if ((*i).getFrom() == removed
                &&
                ((*i).getFromLane() == removedLane
                 ||
                 (*i).getFromLane() == -1)) {
            (*i).replaceFrom(removed, removedLane, by, byLane);

        } else if ((*i).getFrom() == removed && removedLane == -1) {
            (*i).replaceFrom(removed, by);
        }
    }
}


/* -------------------------------------------------------------------------
 * NBLoadedTLDef::Phase-methods
 * ----------------------------------------------------------------------- */
NBLoadedTLDef::NBLoadedTLDef(const NBEdgeCont& ec, const std::string& id,
                             const std::vector<NBNode*>& junctions, SUMOTime offset, TrafficLightType type) :
    NBTrafficLightDefinition(id, junctions, DefaultProgramID, offset, type),
    myEdgeCont(&ec) {
}


NBLoadedTLDef::NBLoadedTLDef(const NBEdgeCont& ec, const std::string& id, NBNode* junction, SUMOTime offset, TrafficLightType type) :
    NBTrafficLightDefinition(id, junction, DefaultProgramID, offset, type),
    myEdgeCont(&ec) {
}


NBLoadedTLDef::NBLoadedTLDef(const NBEdgeCont& ec, const std::string& id, SUMOTime offset, TrafficLightType type) :
    NBTrafficLightDefinition(id, DefaultProgramID, offset, type),
    myEdgeCont(&ec) {
}


NBLoadedTLDef::~NBLoadedTLDef() {
    for (SignalGroupCont::iterator i = mySignalGroups.begin(); i != mySignalGroups.end(); ++i) {
        delete(*i).second;
    }
}


NBTrafficLightLogic*
NBLoadedTLDef::myCompute(int brakingTimeSeconds) {
    MsgHandler::getWarningInstance()->clear(); // !!!
    NBLoadedTLDef::SignalGroupCont::const_iterator i;
    // compute the switching times
    std::set<SUMOReal> tmpSwitchTimes;
    for (i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        NBLoadedTLDef::SignalGroup* group = (*i).second;
        // needed later
        group->sortPhases();
        // patch the yellow time for this group
        group->patchTYellow(brakingTimeSeconds, OptionsCont::getOptions().getBool("tls.yellow.patch-small"));
        // copy the now valid times into the container
        //  both the given red and green phases are added and also the
        //  yellow times
        std::vector<SUMOReal> gtimes = group->getTimes(myCycleDuration);
        for (std::vector<SUMOReal>::const_iterator k = gtimes.begin(); k != gtimes.end(); k++) {
            tmpSwitchTimes.insert(*k);
        }
    }
    std::vector<SUMOReal> switchTimes;
    copy(tmpSwitchTimes.begin(), tmpSwitchTimes.end(), back_inserter(switchTimes));
    sort(switchTimes.begin(), switchTimes.end());

    // count the signals
    int noSignals = 0;
    for (i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        noSignals += (*i).second->getLinkNo();
    }
    // build the phases
    NBTrafficLightLogic* logic = new NBTrafficLightLogic(getID(), getProgramID(), noSignals, myOffset, myType);
    for (std::vector<SUMOReal>::iterator l = switchTimes.begin(); l != switchTimes.end(); l++) {
        // compute the duration of the current phase
        int duration;
        if (l != switchTimes.end() - 1) {
            // get from the difference to the next switching time
            duration = (int)((*(l + 1)) - (*l));
        } else {
            // get from the differenc to the first switching time
            duration = (int)(myCycleDuration - (*l) + * (switchTimes.begin()));
        }
        // no information about yellow times will be generated
        assert((*l) >= 0);
        logic->addStep(TIME2STEPS(duration), buildPhaseState((int)(*l)));
    }
    // check whether any warnings were printed
    if (MsgHandler::getWarningInstance()->wasInformed()) {
        WRITE_WARNING("During computation of traffic light '" + getID() + "'.");
    }
    logic->closeBuilding();

    // initialize myNeedsContRelation
    myNeedsContRelation.clear();
    const bool controlledWithin = !OptionsCont::getOptions().getBool("tls.uncontrolled-within");
    const std::vector<NBTrafficLightLogic::PhaseDefinition> phases = logic->getPhases();
    for (std::vector<NBTrafficLightLogic::PhaseDefinition>::const_iterator it = phases.begin(); it != phases.end(); it++) {
        const std::string state = (*it).state;
        for (NBConnectionVector::const_iterator it1 = myControlledLinks.begin(); it1 != myControlledLinks.end(); it1++) {
            const NBConnection& c1 = *it1;
            const int i1 = c1.getTLIndex();
            if (i1 == NBConnection::InvalidTlIndex || state[i1] != 'g' || c1.getFrom() == 0 || c1.getTo() == 0) {
                continue;
            }
            for (NBConnectionVector::const_iterator it2 = myControlledLinks.begin(); it2 != myControlledLinks.end(); it2++) {
                const NBConnection& c2 = *it2;
                const int i2 = c2.getTLIndex();
                if (i2 != NBConnection::InvalidTlIndex
                        && i2 != i1
                        && (state[i2] == 'G' || state[i2] == 'g')
                        && c2.getFrom() != 0 && c2.getTo() != 0) {
                    const bool rightTurnConflict = NBNode::rightTurnConflict(
                                                       c1.getFrom(), c1.getTo(), c1.getFromLane(), c2.getFrom(), c2.getTo(), c2.getFromLane());
                    if (forbids(c2.getFrom(), c2.getTo(), c1.getFrom(), c1.getTo(), true, controlledWithin) || rightTurnConflict) {
                        myNeedsContRelation.insert(StreamPair(c1.getFrom(), c1.getTo(), c2.getFrom(), c2.getTo()));
                    }
                }
            }
        }
    }
    myNeedsContRelationReady = true;

    return logic;
}


void
NBLoadedTLDef::setTLControllingInformation() const {
    // assign the tl-indices to the edge connections
    for (NBConnectionVector::const_iterator it = myControlledLinks.begin(); it != myControlledLinks.end(); it++) {
        const NBConnection& c = *it;
        if (c.getTLIndex() != NBConnection::InvalidTlIndex) {
            c.getFrom()->setControllingTLInformation(c, getID());
        }
    }
}


std::string
NBLoadedTLDef::buildPhaseState(int time) const {
    int pos = 0;
    std::string state;
    // set the green and yellow information first;
    //  the information whether other have to break needs those masks
    //  completely filled
    for (SignalGroupCont::const_iterator i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        SignalGroup* group = (*i).second;
        int linkNo = group->getLinkNo();
        bool mayDrive = group->mayDrive(time);
        bool hasYellow = group->hasYellow(time);
        char c = 'r';
        if (mayDrive) {
            c = 'g';
        }
        if (hasYellow) {
            c = 'y';
        }
        for (int j = 0; j < linkNo; j++) {
            const NBConnection& conn = group->getConnection(j);
            NBConnection assConn(conn);
            // assert that the connection really exists
            if (assConn.check(*myEdgeCont)) {
                state = state + c;
                ++pos;
            }
        }
    }
    // set the braking mask
    pos = 0;
    for (SignalGroupCont::const_iterator i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        SignalGroup* group = (*i).second;
        int linkNo = group->getLinkNo();
        for (int j = 0; j < linkNo; j++) {
            const NBConnection& conn = group->getConnection(j);
            NBConnection assConn(conn);
            if (assConn.check(*myEdgeCont)) {
                if (!mustBrake(assConn, state, pos)) {
                    if (state[pos] == 'g') {
                        state[pos] = 'G';
                    }
                    if (state[pos] == 'y') {
                        state[pos] = 'Y';
                    }
                }
                pos++;
            }
        }
    }
    return state;
}


bool
NBLoadedTLDef::mustBrake(const NBConnection& possProhibited,
                         const std::string& state,
                         int strmpos) const {
    // check whether the stream has red
    if (state[strmpos] != 'g' && state[strmpos] != 'G') {
        return true;
    }

    // check whether another stream which has green is a higher
    //  priorised foe to the given
    int pos = 0;
    for (SignalGroupCont::const_iterator i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        SignalGroup* group = (*i).second;
        // get otherlinks that have green
        int linkNo = group->getLinkNo();
        for (int j = 0; j < linkNo; j++) {
            // get the current connection (possible foe)
            const NBConnection& other = group->getConnection(j);
            NBConnection possProhibitor(other);
            // if the connction ist still valid ...
            if (possProhibitor.check(*myEdgeCont)) {
                // ... do nothing if it starts at the same edge
                if (possProhibited.getFrom() == possProhibitor.getFrom()) {
                    pos++;
                    continue;
                }
                if (state[pos] == 'g' || state[pos] == 'G') {
                    if (NBTrafficLightDefinition::mustBrake(possProhibited, possProhibitor, true)) {
                        return true;
                    }
                }
                pos++;
            }
        }
    }
    return false;
}


void
NBLoadedTLDef::collectNodes() {
    myControlledNodes.clear();
    SignalGroupCont::const_iterator m;
    for (m = mySignalGroups.begin(); m != mySignalGroups.end(); m++) {
        SignalGroup* group = (*m).second;
        int linkNo = group->getLinkNo();
        for (int j = 0; j < linkNo; j++) {
            const NBConnection& conn = group->getConnection(j);
            NBEdge* edge = conn.getFrom();
            NBNode* node = edge->getToNode();
            myControlledNodes.push_back(node);
        }
    }
    std::sort(myControlledNodes.begin(), myControlledNodes.end(), NBNode::nodes_by_id_sorter());
}


void
NBLoadedTLDef::collectLinks() {
    myControlledLinks.clear();
    // build the list of links which are controled by the traffic light
    for (EdgeVector::iterator i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
        NBEdge* incoming = *i;
        int noLanes = incoming->getNumLanes();
        for (int j = 0; j < noLanes; j++) {
            std::vector<NBEdge::Connection> elv = incoming->getConnectionsFromLane(j);
            for (std::vector<NBEdge::Connection>::iterator k = elv.begin(); k != elv.end(); k++) {
                NBEdge::Connection el = *k;
                if (el.toEdge != 0) {
                    myControlledLinks.push_back(NBConnection(incoming, j, el.toEdge, el.toLane));
                }
            }
        }
    }

    // assign tl-indices to myControlledLinks
    int pos = 0;
    for (SignalGroupCont::const_iterator m = mySignalGroups.begin(); m != mySignalGroups.end(); m++) {
        SignalGroup* group = (*m).second;
        int linkNo = group->getLinkNo();
        for (int j = 0; j < linkNo; j++) {
            const NBConnection& conn = group->getConnection(j);
            assert(conn.getFromLane() < 0 || (int) conn.getFrom()->getNumLanes() > conn.getFromLane());
            NBConnection tst(conn);
            tst.setTLIndex(pos);
            if (tst.check(*myEdgeCont)) {
                if (tst.getFrom()->mayBeTLSControlled(tst.getFromLane(), tst.getTo(), tst.getToLane())) {
                    for (NBConnectionVector::iterator it = myControlledLinks.begin(); it != myControlledLinks.end(); it++) {
                        NBConnection& c = *it;
                        if (c.getTLIndex() == NBConnection::InvalidTlIndex
                                && tst.getFrom() == c.getFrom() && tst.getTo() == c.getTo()
                                && (tst.getFromLane() < 0 || tst.getFromLane() == c.getFromLane())
                                && (tst.getToLane() < 0 || tst.getToLane() == c.getToLane())) {
                            c.setTLIndex(pos);
                        }
                    }
                    //std::cout << getID() << " group=" << (*m).first << " tst=" << tst << "\n";
                    pos++;
                }
            } else {
                WRITE_WARNING("Could not set signal on connection (signal: " + getID() + ", group: " + group->getID() + ")");
            }
        }
    }
}


NBLoadedTLDef::SignalGroup*
NBLoadedTLDef::findGroup(NBEdge* from, NBEdge* to) const {
    for (SignalGroupCont::const_iterator i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        if ((*i).second->containsConnection(from, to)) {
            return (*i).second;
        }
    }
    return 0;
}


bool
NBLoadedTLDef::addToSignalGroup(const std::string& groupid,
                                const NBConnection& connection) {
    if (mySignalGroups.find(groupid) == mySignalGroups.end()) {
        return false;
    }
    mySignalGroups[groupid]->addConnection(connection);
    NBNode* n1 = connection.getFrom()->getToNode();
    if (n1 != 0) {
        addNode(n1);
        n1->addTrafficLight(this);
    }
    NBNode* n2 = connection.getTo()->getFromNode();
    if (n2 != 0) {
        addNode(n2);
        n2->addTrafficLight(this);
    }
    return true;
}


bool
NBLoadedTLDef::addToSignalGroup(const std::string& groupid,
                                const NBConnectionVector& connections) {
    bool ok = true;
    for (NBConnectionVector::const_iterator i = connections.begin(); i != connections.end(); i++) {
        ok &= addToSignalGroup(groupid, *i);
    }
    return ok;
}


void
NBLoadedTLDef::addSignalGroup(const std::string& id) {
    assert(mySignalGroups.find(id) == mySignalGroups.end());
    mySignalGroups[id] = new SignalGroup(id);
}


void
NBLoadedTLDef::addSignalGroupPhaseBegin(const std::string& groupid, SUMOTime time,
                                        TLColor color) {
    assert(mySignalGroups.find(groupid) != mySignalGroups.end());
    mySignalGroups[groupid]->addPhaseBegin(time, color);
}

void
NBLoadedTLDef::setSignalYellowTimes(const std::string& groupid,
                                    SUMOTime myTRedYellow, SUMOTime myTYellow) {
    assert(mySignalGroups.find(groupid) != mySignalGroups.end());
    mySignalGroups[groupid]->setYellowTimes(myTRedYellow, myTYellow);
}


void
NBLoadedTLDef::setCycleDuration(int cycleDur) {
    myCycleDuration = cycleDur;
}


void
NBLoadedTLDef::remapRemoved(NBEdge* removed,
                            const EdgeVector& incoming,
                            const EdgeVector& outgoing) {
    for (SignalGroupCont::const_iterator i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        SignalGroup* group = (*i).second;
        if (group->containsIncoming(removed)) {
            group->remapIncoming(removed, incoming);
        }
        if (group->containsOutgoing(removed)) {
            group->remapOutgoing(removed, outgoing);
        }
    }
}


void
NBLoadedTLDef::replaceRemoved(NBEdge* removed, int removedLane,
                              NBEdge* by, int byLane) {
    for (SignalGroupCont::const_iterator i = mySignalGroups.begin(); i != mySignalGroups.end(); i++) {
        SignalGroup* group = (*i).second;
        if (group->containsIncoming(removed) || group->containsOutgoing(removed)) {
            group->remap(removed, removedLane, by, byLane);
        }
    }
}


void
NBLoadedTLDef::initNeedsContRelation() const {
    if (!myNeedsContRelationReady) {
        throw ProcessError("myNeedsContRelation was not propperly initialized\n");
    }
}


/****************************************************************************/

