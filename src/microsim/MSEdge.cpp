/****************************************************************************/
/// @file    MSEdge.cpp
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Christoph Sommer
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Sascha Krieg
/// @date    Tue, 06 Mar 2001
/// @version $Id: MSEdge.cpp 21201 2016-07-19 11:57:22Z behrisch $
///
// A road/street connecting two junctions
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
#include <iostream>
#include <cassert>
#include <utils/common/StringTokenizer.h>
#include <utils/options/OptionsCont.h>
#include "MSEdge.h"
#include "MSInsertionControl.h"
#include "MSJunction.h"
#include "MSLane.h"
#include "MSLaneChanger.h"
#include "MSLaneChangerSublane.h"
#include "MSGlobals.h"
#include "MSNet.h"
#include "MSVehicle.h"
#include "MSLeaderInfo.h"
#include "MSContainer.h"
#include "MSEdgeWeightsStorage.h"
#include <microsim/devices/MSDevice_Routing.h>

#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#include <mesosim/MEVehicle.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// static member definitions
// ===========================================================================
MSEdge::DictType MSEdge::myDict;
MSEdgeVector MSEdge::myEdges;


// ===========================================================================
// member method definitions
// ===========================================================================
MSEdge::MSEdge(const std::string& id, int numericalID,
               const EdgeBasicFunction function,
               const std::string& streetName,
               const std::string& edgeType,
               int priority) :
    Named(id), myNumericalID(numericalID), myLanes(0),
    myLaneChanger(0), myFunction(function), myVaporizationRequests(0),
    myLastFailedInsertionTime(-1),
    myFromJunction(0), myToJunction(0),
    myStreetName(streetName),
    myEdgeType(edgeType),
    myPriority(priority),
    myWidth(0),
    myLength(-1.),
    myEmptyTraveltime(-1.),
    myAmDelayed(false),
    myAmRoundabout(false) {}


MSEdge::~MSEdge() {
    delete myLaneChanger;
    for (AllowedLanesCont::iterator i1 = myAllowed.begin(); i1 != myAllowed.end(); i1++) {
        delete(*i1).second;
    }
    for (ClassedAllowedLanesCont::iterator i2 = myClassedAllowed.begin(); i2 != myClassedAllowed.end(); i2++) {
        for (AllowedLanesCont::iterator i1 = (*i2).second.begin(); i1 != (*i2).second.end(); i1++) {
            delete(*i1).second;
        }
    }
    delete myLanes;
    // Note: Lanes are delete using MSLane::clear();
}


void
MSEdge::initialize(const std::vector<MSLane*>* lanes) {
    assert(lanes != 0);
    myLanes = lanes;
    if (!lanes->empty()) {
        recalcCache();
    }
    if (myFunction == EDGEFUNCTION_DISTRICT) {
        myCombinedPermissions = SVCAll;
    }
    for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        myWidth += (*i)->getWidth();
    }
    if (MSGlobals::gLateralResolution > 0 || MSGlobals::gLaneChangeDuration > 0) {
        SUMOReal widthBefore = 0;
        for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            (*i)->setRightSideOnEdge(widthBefore, (int)mySublaneSides.size());
            MSLeaderInfo ahead(*i);
            for (int j = 0; j < ahead.numSublanes(); ++j) {
                mySublaneSides.push_back(widthBefore + j * MSGlobals::gLateralResolution);
            }
            widthBefore += (*i)->getWidth();
        }
    }
}


void MSEdge::recalcCache() {
    if (myLanes->empty()) {
        return;
    }
    myLength = myLanes->front()->getLength();
    myEmptyTraveltime = myLength / MAX2(getSpeedLimit(), NUMERICAL_EPS);

    if (MSGlobals::gMesoTLSPenalty > 0) {
        // add tls penalties to the minimum travel time
        SUMOTime minPenalty = -1;
        for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            MSLane* l = *i;
            const MSLinkCont& lc = l->getLinkCont();
            for (MSLinkCont::const_iterator j = lc.begin(); j != lc.end(); ++j) {
                MSLink* link = *j;
                if (minPenalty == -1) {
                    minPenalty = link->getMesoTLSPenalty();
                } else {
                    minPenalty = MIN2(minPenalty, link->getMesoTLSPenalty());
                }
            }
        }
        if (minPenalty > 0) {
            myEmptyTraveltime += STEPS2TIME(minPenalty);
        }
    }
}


void
MSEdge::closeBuilding() {
    myAllowed[0] = new std::vector<MSLane*>();
    for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        myAllowed[0]->push_back(*i);
        const MSLinkCont& lc = (*i)->getLinkCont();
        for (MSLinkCont::const_iterator j = lc.begin(); j != lc.end(); ++j) {
            (*j)->initParallelLinks();
            MSLane* toL = (*j)->getLane();
            if (toL != 0) {
                MSEdge& to = toL->getEdge();
                //
                if (std::find(mySuccessors.begin(), mySuccessors.end(), &to) == mySuccessors.end()) {
                    mySuccessors.push_back(&to);
                }
                if (std::find(to.myPredecessors.begin(), to.myPredecessors.end(), this) == to.myPredecessors.end()) {
                    to.myPredecessors.push_back(this);
                }
                //
                if (myAllowed.find(&to) == myAllowed.end()) {
                    myAllowed[&to] = new std::vector<MSLane*>();
                }
                myAllowed[&to]->push_back(*i);
            }
#ifdef HAVE_INTERNAL_LANES
            toL = (*j)->getViaLane();
            if (toL != 0) {
                MSEdge& to = toL->getEdge();
                if (std::find(to.myPredecessors.begin(), to.myPredecessors.end(), this) == to.myPredecessors.end()) {
                    to.myPredecessors.push_back(this);
                }
            }
#endif
        }
    }
    std::sort(mySuccessors.begin(), mySuccessors.end(), by_id_sorter());
    rebuildAllowedLanes();
    // segment building depends on the finished list of successors (for multi-queue)
    if (MSGlobals::gUseMesoSim && !myLanes->empty()) {
        MSGlobals::gMesoNet->buildSegmentsFor(*this, OptionsCont::getOptions());
    }
}


void
MSEdge::buildLaneChanger() {
    if (!myLanes->empty()) {
        const bool allowSwap = OptionsCont::getOptions().getBool("lanechange.allow-swap");
        const bool allowChanging = allowsLaneChanging();
        if (MSGlobals::gLateralResolution > 0) {
            // may always initiate sublane-change
            myLaneChanger = new MSLaneChangerSublane(myLanes, allowChanging, allowSwap);
        } else {
            if (MSGlobals::gLaneChangeDuration > 0) {
                myLaneChanger = new MSLaneChanger(myLanes, allowChanging, allowSwap);
            } else if (myLanes->size() > 1 || canChangeToOpposite()) {
                myLaneChanger = new MSLaneChanger(myLanes, allowChanging, allowSwap);
            }
        }
    }
}


bool
MSEdge::allowsLaneChanging() {
    if (myFunction == EDGEFUNCTION_INTERNAL) {
        // allow changing only if all links leading to this internal lane have priority
        // or they are controlled by a traffic light
        for (std::vector<MSLane*>::const_iterator it = myLanes->begin(); it != myLanes->end(); ++it) {
            MSLane* pred = (*it)->getLogicalPredecessorLane();
            MSLink* link = MSLinkContHelper::getConnectingLink(*pred, **it);
            assert(link != 0);
            LinkState state = link->getState();
            if (state == LINKSTATE_MINOR
                    || state == LINKSTATE_EQUAL
                    || state == LINKSTATE_STOP
                    || state == LINKSTATE_ALLWAY_STOP
                    || state == LINKSTATE_DEADEND) {
                return false;
            }
        }
    }
    return true;
}


void
MSEdge::rebuildAllowedLanes() {
    // clear myClassedAllowed.
    // it will be rebuilt on demand
    for (ClassedAllowedLanesCont::iterator i2 = myClassedAllowed.begin(); i2 != myClassedAllowed.end(); i2++) {
        for (AllowedLanesCont::iterator i1 = (*i2).second.begin(); i1 != (*i2).second.end(); i1++) {
            delete(*i1).second;
        }
    }
    myClassedAllowed.clear();
    myClassesSuccessorMap.clear();
    // rebuild myMinimumPermissions and myCombinedPermissions
    myMinimumPermissions = SVCAll;
    myCombinedPermissions = 0;
    for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        myMinimumPermissions &= (*i)->getPermissions();
        myCombinedPermissions |= (*i)->getPermissions();
    }
}


// ------------ Access to the edge's lanes
MSLane*
MSEdge::leftLane(const MSLane* const lane) const {
    return parallelLane(lane, 1);
}


MSLane*
MSEdge::rightLane(const MSLane* const lane) const {
    return parallelLane(lane, -1);
}


MSLane*
MSEdge::parallelLane(const MSLane* const lane, int offset) const {
    const int index = (int)(find(myLanes->begin(), myLanes->end(), lane) - myLanes->begin());
    if (index == (int)myLanes->size()) {
        return 0;
    }
    const int resultIndex = index + offset;
    if (resultIndex >= (int)myLanes->size() || resultIndex < 0) {
        return 0;
    } else {
        return (*myLanes)[resultIndex];
    }
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(const MSEdge& destination, SUMOVehicleClass vclass) const {
    return allowedLanes(&destination, vclass);
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(SUMOVehicleClass vclass) const {
    return allowedLanes(0, vclass);
}


const std::vector<MSLane*>*
MSEdge::getAllowedLanesWithDefault(const AllowedLanesCont& c, const MSEdge* dest) const {
    AllowedLanesCont::const_iterator it = c.find(dest);
    if (it == c.end()) {
        return 0;
    }
    return it->second;
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(const MSEdge* destination, SUMOVehicleClass vclass) const {
    if (destination == 0 && (myMinimumPermissions & vclass) == vclass) {
        // all lanes allow vclass
        return getAllowedLanesWithDefault(myAllowed, destination);
    }
    // look up cached result in myClassedAllowed
    ClassedAllowedLanesCont::const_iterator i = myClassedAllowed.find(vclass);
    if (i != myClassedAllowed.end()) {
        // can use cached value
        const AllowedLanesCont& c = (*i).second;
        return getAllowedLanesWithDefault(c, destination);
    } else {
        // this vclass is requested for the first time. rebuild all destinations
        // go through connected edges
#ifdef HAVE_FOX
        if (MSDevice_Routing::isParallel()) {
            MSDevice_Routing::lock();
        }
#endif
        for (AllowedLanesCont::const_iterator i1 = myAllowed.begin(); i1 != myAllowed.end(); ++i1) {
            const MSEdge* edge = i1->first;
            const std::vector<MSLane*>* lanes = i1->second;
            myClassedAllowed[vclass][edge] = new std::vector<MSLane*>();
            // go through lanes approaching current edge
            for (std::vector<MSLane*>::const_iterator i2 = lanes->begin(); i2 != lanes->end(); ++i2) {
                // origin lane allows the current vehicle class?
                if ((*i2)->allowsVehicleClass(vclass)) {
                    if (edge == 0) {
                        myClassedAllowed[vclass][edge]->push_back(*i2);
                    } else {
                        // target lane allows the current vehicle class?
                        const MSLinkCont& lc = (*i2)->getLinkCont();
                        for (MSLinkCont::const_iterator it_link = lc.begin(); it_link != lc.end(); ++it_link) {
                            const MSLane* targetLane = (*it_link)->getLane();
                            if ((&(targetLane->getEdge()) == edge) && targetLane->allowsVehicleClass(vclass)) {
                                // -> may be used
                                myClassedAllowed[vclass][edge]->push_back(*i2);
                                break;
                            }
                        }
                    }
                }
            }
            // assert that 0 is returned if no connection is allowed for a class
            if (myClassedAllowed[vclass][edge]->size() == 0) {
                delete myClassedAllowed[vclass][edge];
                myClassedAllowed[vclass][edge] = 0;
            }
        }
#ifdef HAVE_FOX
        if (MSDevice_Routing::isParallel()) {
            MSDevice_Routing::unlock();
        }
#endif
        return myClassedAllowed[vclass][destination];
    }
}


// ------------
SUMOTime
MSEdge::incVaporization(SUMOTime) {
    ++myVaporizationRequests;
    return 0;
}


SUMOTime
MSEdge::decVaporization(SUMOTime) {
    --myVaporizationRequests;
    return 0;
}


MSLane*
MSEdge::getFreeLane(const std::vector<MSLane*>* allowed, const SUMOVehicleClass vclass) const {
    if (allowed == 0) {
        allowed = allowedLanes(vclass);
    }
    MSLane* res = 0;
    if (allowed != 0) {
        SUMOReal leastOccupancy = std::numeric_limits<SUMOReal>::max();;
        for (std::vector<MSLane*>::const_iterator i = allowed->begin(); i != allowed->end(); ++i) {
            const SUMOReal occupancy = (*i)->getBruttoOccupancy();
            if (occupancy < leastOccupancy) {
                res = (*i);
                leastOccupancy = occupancy;
            }
        }
    }
    return res;
}


MSLane*
MSEdge::getDepartLane(MSVehicle& veh) const {
    switch (veh.getParameter().departLaneProcedure) {
        case DEPART_LANE_GIVEN:
            if ((int) myLanes->size() <= veh.getParameter().departLane || !(*myLanes)[veh.getParameter().departLane]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
                return 0;
            }
            return (*myLanes)[veh.getParameter().departLane];
        case DEPART_LANE_RANDOM:
            return RandHelper::getRandomFrom(*allowedLanes(veh.getVehicleType().getVehicleClass()));
        case DEPART_LANE_FREE:
            return getFreeLane(0, veh.getVehicleType().getVehicleClass());
        case DEPART_LANE_ALLOWED_FREE:
            if (veh.getRoute().size() == 1) {
                return getFreeLane(0, veh.getVehicleType().getVehicleClass());
            } else {
                return getFreeLane(allowedLanes(**(veh.getRoute().begin() + 1)), veh.getVehicleType().getVehicleClass());
            }
        case DEPART_LANE_BEST_FREE: {
            veh.updateBestLanes(false, myLanes->front());
            const std::vector<MSVehicle::LaneQ>& bl = veh.getBestLanes();
            SUMOReal bestLength = -1;
            for (std::vector<MSVehicle::LaneQ>::const_iterator i = bl.begin(); i != bl.end(); ++i) {
                if ((*i).length > bestLength) {
                    bestLength = (*i).length;
                }
            }
            std::vector<MSLane*>* bestLanes = new std::vector<MSLane*>();
            for (std::vector<MSVehicle::LaneQ>::const_iterator i = bl.begin(); i != bl.end(); ++i) {
                if ((*i).length == bestLength) {
                    bestLanes->push_back((*i).lane);
                }
            }
            MSLane* ret = getFreeLane(bestLanes, veh.getVehicleType().getVehicleClass());
            delete bestLanes;
            return ret;
        }
        case DEPART_LANE_DEFAULT:
        case DEPART_LANE_FIRST_ALLOWED:
            for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
                if ((*i)->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
                    return *i;
                }
            }
            return 0;
        default:
            break;
    }
    if (!(*myLanes)[0]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
        return 0;
    }
    return (*myLanes)[0];
}


bool
MSEdge::insertVehicle(SUMOVehicle& v, SUMOTime time, const bool checkOnly) const {
    // when vaporizing, no vehicles are inserted, but checking needs to be successful to trigger removal
    if (isVaporizing()) {
        return checkOnly;
    }
    const SUMOVehicleParameter& pars = v.getParameter();
    const MSVehicleType& type = v.getVehicleType();
    if (pars.departSpeedProcedure == DEPART_SPEED_GIVEN && pars.departSpeed > getVehicleMaxSpeed(&v)) {
        if (type.getSpeedDeviation() > 0 && pars.departSpeed <= type.getSpeedFactor() * getSpeedLimit() * (2 * type.getSpeedDeviation() + 1.)) {
            WRITE_WARNING("Choosing new speed factor for vehicle '" + pars.id + "' to match departure speed.");
            v.setChosenSpeedFactor(type.computeChosenSpeedDeviation(0, pars.departSpeed / (type.getSpeedFactor() * getSpeedLimit())));
        } else {
            throw ProcessError("Departure speed for vehicle '" + pars.id +
                               "' is too high for the departure edge '" + getID() + "'.");
        }
    }
    if (checkOnly && v.getEdge()->getPurpose() == MSEdge::EDGEFUNCTION_DISTRICT) {
        return true;
    }
    if (!checkOnly) {
        std::string msg;
        if (!v.hasValidRoute(msg)) {
            if (MSGlobals::gCheckRoutes) {
                throw ProcessError("Vehicle '" + v.getID() + "' has no valid route. " + msg);
            } else if (v.getEdge()->getPurpose() == MSEdge::EDGEFUNCTION_DISTRICT) {
                WRITE_WARNING("Removing vehicle '" + pars.id + "' which has no valid route.");
                MSNet::getInstance()->getInsertionControl().descheduleDeparture(&v);
                return false;
            }
        }
    }
    if (MSGlobals::gUseMesoSim) {
        SUMOReal pos = 0.0;
        switch (pars.departPosProcedure) {
            case DEPART_POS_GIVEN:
                if (pars.departPos >= 0.) {
                    pos = pars.departPos;
                } else {
                    pos = pars.departPos + getLength();
                }
                if (pos < 0 || pos > getLength()) {
                    WRITE_WARNING("Invalid departPos " + toString(pos) + " given for vehicle '" +
                                  v.getID() + "'. Inserting at lane end instead.");
                    pos = getLength();
                }
                break;
            case DEPART_POS_RANDOM:
            case DEPART_POS_RANDOM_FREE:
                pos = RandHelper::rand(getLength());
                break;
            default:
                break;
        }
        bool result = false;
        MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this, pos);
        MEVehicle* veh = static_cast<MEVehicle*>(&v);
        if (pars.departPosProcedure == DEPART_POS_FREE) {
            while (segment != 0 && !result) {
                if (checkOnly) {
                    result = segment->hasSpaceFor(veh, time, true);
                } else {
                    result = segment->initialise(veh, time);
                }
                segment = segment->getNextSegment();
            }
        } else {
            if (checkOnly) {
                result = segment->hasSpaceFor(veh, time, true);
            } else {
                result = segment->initialise(veh, time);
            }
        }
        return result;
    }
    if (checkOnly) {
        switch (v.getParameter().departLaneProcedure) {
            case DEPART_LANE_GIVEN:
            case DEPART_LANE_DEFAULT:
            case DEPART_LANE_FIRST_ALLOWED: {
                const SUMOReal occupancy = getDepartLane(static_cast<MSVehicle&>(v))->getBruttoOccupancy();
                return occupancy == (SUMOReal)0 || occupancy * myLength + v.getVehicleType().getLengthWithGap() <= myLength;
            }
            default:
                for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
                    const SUMOReal occupancy = (*i)->getBruttoOccupancy();
                    if (occupancy == (SUMOReal)0 || occupancy * myLength + v.getVehicleType().getLengthWithGap() <= myLength) {
                        return true;
                    }
                }
        }
        return false;
    }
    MSLane* insertionLane = getDepartLane(static_cast<MSVehicle&>(v));
    return insertionLane != 0 && insertionLane->insertVehicle(static_cast<MSVehicle&>(v));
}


void
MSEdge::changeLanes(SUMOTime t) {
    if (myLaneChanger == 0) {
        return;
    }
    if (myFunction == EDGEFUNCTION_INTERNAL) {
        // allow changing only if all links leading to this internal lane have priority
        // or they are controlled by a traffic light
        for (std::vector<MSLane*>::const_iterator it = myLanes->begin(); it != myLanes->end(); ++it) {
            MSLane* pred = (*it)->getLogicalPredecessorLane();
            MSLink* link = MSLinkContHelper::getConnectingLink(*pred, **it);
            assert(link != 0);
            LinkState state = link->getState();
            if (state == LINKSTATE_MINOR
                    || state == LINKSTATE_EQUAL
                    || state == LINKSTATE_STOP
                    || state == LINKSTATE_ALLWAY_STOP
                    || state == LINKSTATE_ZIPPER
                    || state == LINKSTATE_DEADEND) {
                return;
            }
        }
    }
    myLaneChanger->laneChange(t);
}



#ifdef HAVE_INTERNAL_LANES
const MSEdge*
MSEdge::getInternalFollowingEdge(const MSEdge* followerAfterInternal) const {
    //@todo to be optimized
    for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        MSLane* l = *i;
        const MSLinkCont& lc = l->getLinkCont();
        for (MSLinkCont::const_iterator j = lc.begin(); j != lc.end(); ++j) {
            MSLink* link = *j;
            if (&link->getLane()->getEdge() == followerAfterInternal) {
                if (link->getViaLane() != 0) {
                    return &link->getViaLane()->getEdge();
                } else {
                    return 0; // network without internal links
                }
            }
        }
    }
    return 0;
}
#endif


SUMOReal
MSEdge::getMeanSpeed() const {
    SUMOReal v = 0;
    SUMOReal no = 0;
    if (MSGlobals::gUseMesoSim) {
        for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != 0; segment = segment->getNextSegment()) {
            const SUMOReal vehNo = (SUMOReal) segment->getCarNumber();
            v += vehNo * segment->getMeanSpeed();
            no += vehNo;
        }
        if (no == 0) {
            return getLength() / myEmptyTraveltime; // may include tls-penalty
        }
    } else {
        for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            const SUMOReal vehNo = (SUMOReal)(*i)->getVehicleNumber();
            v += vehNo * (*i)->getMeanSpeed();
            no += vehNo;
        }
        if (no == 0) {
            return getSpeedLimit();
        }
    }
    return v / no;
}


SUMOReal
MSEdge::getCurrentTravelTime(SUMOReal minSpeed) const {
    assert(minSpeed > 0);
    if (!myAmDelayed) {
        return myEmptyTraveltime;
    }
    return getLength() / MAX2(minSpeed, getMeanSpeed());
}


SUMOReal
MSEdge::getRoutingSpeed() const {
    return MSDevice_Routing::getAssumedSpeed(this);
}



bool
MSEdge::dictionary(const std::string& id, MSEdge* ptr) {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        myDict[id] = ptr;
        while ((int)myEdges.size() < ptr->getNumericalID() + 1) {
            myEdges.push_back(0);
        }
        myEdges[ptr->getNumericalID()] = ptr;
        return true;
    }
    return false;
}


MSEdge*
MSEdge::dictionary(const std::string& id) {
    DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict.
        return 0;
    }
    return it->second;
}


int
MSEdge::dictSize() {
    return (int)myDict.size();
}


const MSEdgeVector&
MSEdge::getAllEdges() {
    return myEdges;
}


void
MSEdge::clear() {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        delete(*i).second;
    }
    myDict.clear();
}


void
MSEdge::insertIDs(std::vector<std::string>& into) {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        into.push_back((*i).first);
    }
}


void
MSEdge::parseEdgesList(const std::string& desc, ConstMSEdgeVector& into,
                       const std::string& rid) {
    if (desc[0] == BinaryFormatter::BF_ROUTE) {
        std::istringstream in(desc, std::ios::binary);
        char c;
        in >> c;
        FileHelpers::readEdgeVector(in, into, rid);
    } else {
        StringTokenizer st(desc);
        parseEdgesList(st.getVector(), into, rid);
    }
}


void
MSEdge::parseEdgesList(const std::vector<std::string>& desc, ConstMSEdgeVector& into,
                       const std::string& rid) {
    for (std::vector<std::string>::const_iterator i = desc.begin(); i != desc.end(); ++i) {
        const MSEdge* edge = MSEdge::dictionary(*i);
        // check whether the edge exists
        if (edge == 0) {
            throw ProcessError("The edge '" + *i + "' within the route " + rid + " is not known."
                               + "\n The route can not be build.");
        }
        into.push_back(edge);
    }
}


SUMOReal
MSEdge::getDistanceTo(const MSEdge* other) const {
    if (getLanes().size() > 0 && other->getLanes().size() > 0) {
        return getToJunction()->getPosition().distanceTo2D(other->getFromJunction()->getPosition());
    } else {
        return 0; // optimism is just right for astar
    }
}


SUMOReal
MSEdge::getSpeedLimit() const {
    // @note lanes might have different maximum speeds in theory
    return getLanes()[0]->getSpeedLimit();
}


SUMOReal
MSEdge::getVehicleMaxSpeed(const SUMOVehicle* const veh) const {
    // @note lanes might have different maximum speeds in theory
    return getLanes()[0]->getVehicleMaxSpeed(veh);
}


void
MSEdge::setMaxSpeed(SUMOReal val) const {
    if (myLanes != 0) {
        for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            (*i)->setMaxSpeed(val);
        }
    }
}



std::vector<MSTransportable*>
MSEdge::getSortedPersons(SUMOTime timestep) const {
    std::vector<MSTransportable*> result(myPersons.begin(), myPersons.end());
    sort(result.begin(), result.end(), transportable_by_position_sorter(timestep));
    return result;
}


std::vector<MSTransportable*>
MSEdge::getSortedContainers(SUMOTime timestep) const {
    std::vector<MSTransportable*> result(myContainers.begin(), myContainers.end());
    sort(result.begin(), result.end(), transportable_by_position_sorter(timestep));
    return result;
}


int
MSEdge::transportable_by_position_sorter::operator()(const MSTransportable* const c1, const MSTransportable* const c2) const {
    const SUMOReal pos1 = c1->getCurrentStage()->getEdgePos(myTime);
    const SUMOReal pos2 = c2->getCurrentStage()->getEdgePos(myTime);
    if (pos1 != pos2) {
        return pos1 < pos2;
    }
    return c1->getID() < c2->getID();
}

const MSEdgeVector&
MSEdge::getSuccessors(SUMOVehicleClass vClass) const {
    if (vClass == SVC_IGNORING || !MSNet::getInstance()->hasPermissions() || myFunction == EDGEFUNCTION_DISTRICT) {
        return mySuccessors;
    }
#ifdef HAVE_FOX
    if (MSDevice_Routing::isParallel()) {
        MSDevice_Routing::lock();
    }
#endif
    std::map<SUMOVehicleClass, MSEdgeVector>::iterator i = myClassesSuccessorMap.find(vClass);
    if (i == myClassesSuccessorMap.end()) {
        // instantiate vector
        myClassesSuccessorMap[vClass];
        i = myClassesSuccessorMap.find(vClass);
        // this vClass is requested for the first time. rebuild all successors
        for (MSEdgeVector::const_iterator it = mySuccessors.begin(); it != mySuccessors.end(); ++it) {
            if ((*it)->getPurpose() == EDGEFUNCTION_DISTRICT) {
                i->second.push_back(*it);
            } else {
                const std::vector<MSLane*>* allowed = allowedLanes(*it, vClass);
                if (allowed != 0 && allowed->size() > 0) {
                    i->second.push_back(*it);
                }
            }
        }
    }
    // can use cached value
#ifdef HAVE_FOX
    if (MSDevice_Routing::isParallel()) {
        MSDevice_Routing::unlock();
    }
#endif
    return i->second;
}


bool
MSEdge::canChangeToOpposite() {
    return !myLanes->empty() && myLanes->back()->getOpposite() != 0;
}

/****************************************************************************/

