/****************************************************************************/
/// @file    MSCalibrator.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Tue, May 2005
/// @version $Id: MSCalibrator.cpp 21217 2016-07-22 10:57:44Z behrisch $
///
// Calibrates the flow on an edge by removing an inserting vehicles
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2005-2016 DLR (http://www.dlr.de/) and contributors
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
#include <algorithm>
#include <cmath>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSEventControl.h>
#include <microsim/output/MSRouteProbe.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/common/ToString.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StringTokenizer.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/common/TplConvert.h>
#include <utils/options/OptionsCont.h>
#include <utils/xml/SUMOVehicleParserHelper.h>
#include <utils/common/RandomDistributor.h>
#include <utils/vehicle/SUMOVehicleParameter.h>
#include "MSCalibrator.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

//#define MSCalibrator_DEBUG

// ===========================================================================
// static members
// ===========================================================================
std::vector<MSMoveReminder*> MSCalibrator::LeftoverReminders;
std::vector<SUMOVehicleParameter*> MSCalibrator::LeftoverVehicleParameters;

// ===========================================================================
// method definitions
// ===========================================================================
MSCalibrator::MSCalibrator(const std::string& id,
                           const MSEdge* const edge, const SUMOReal pos,
                           const std::string& aXMLFilename,
                           const std::string& outputFilename,
                           const SUMOTime freq, const SUMOReal length,
                           const MSRouteProbe* probe,
                           const bool addLaneMeanData) :
    MSTrigger(id),
    MSRouteHandler(aXMLFilename, false),
    myEdge(edge), myPos(pos), myProbe(probe),
    myEdgeMeanData(0, length, false),
    myOutput(0), myFrequency(freq), myRemoved(0),
    myInserted(0), myClearedInJam(0),
    mySpeedIsDefault(true), myDidSpeedAdaption(false), myDidInit(false),
    myDefaultSpeed(myEdge->getSpeedLimit()),
    myHaveWarnedAboutClearingJam(false),
    myAmActive(false) {
    if (outputFilename != "") {
        myOutput = &OutputDevice::getDevice(outputFilename);
        myOutput->writeXMLHeader("calibratorstats");
    }
    if (aXMLFilename != "") {
        XMLSubSys::runParser(*this, aXMLFilename);
        if (!myDidInit) {
            init();
        }
    }
    if (addLaneMeanData) {
        for (int i = 0; i < (int)myEdge->getLanes().size(); ++i) {
            MSLane* lane = myEdge->getLanes()[i];
            MSMeanData_Net::MSLaneMeanDataValues* laneData = new MSMeanData_Net::MSLaneMeanDataValues(lane, myEdge->getLength(), true);
            laneData->setDescription("meandata_calibrator_" + lane->getID());
            LeftoverReminders.push_back(laneData);
            myLaneMeanData.push_back(laneData);
            VehicleRemover* remover = new VehicleRemover(lane, (int)i, this);
            LeftoverReminders.push_back(remover);
            myVehicleRemovers.push_back(remover);
        }
    }
}


void
MSCalibrator::init() {
    if (myIntervals.size() > 0) {
        if (myIntervals.back().end == -1) {
            myIntervals.back().end = SUMOTime_MAX;
        }
        myCurrentStateInterval = myIntervals.begin();
        // calibration should happen after regular insertions have taken place
        MSNet::getInstance()->getEndOfTimestepEvents()->addEvent(this,
                MSNet::getInstance()->getCurrentTimeStep(),
                MSEventControl::ADAPT_AFTER_EXECUTION);
    } else {
        WRITE_WARNING("No flow intervals in calibrator '" + myID + "'.");
    }
    myDidInit = true;
}


MSCalibrator::~MSCalibrator() {
    if (myIntervals.size() > 0 && myCurrentStateInterval != myIntervals.end()) {
        writeXMLOutput();
    }
    for (std::vector<VehicleRemover*>::iterator it = myVehicleRemovers.begin(); it != myVehicleRemovers.end(); ++it) {
        (*it)->disable();
    }
}


void
MSCalibrator::myStartElement(int element,
                             const SUMOSAXAttributes& attrs) {
    if (element == SUMO_TAG_FLOW) {
        AspiredState state;
        SUMOTime lastEnd = -1;
        if (myIntervals.size() > 0) {
            lastEnd = myIntervals.back().end;
            if (lastEnd == -1) {
                lastEnd = myIntervals.back().begin;
            }
        }
        try {
            bool ok = true;
            state.q = attrs.getOpt<SUMOReal>(SUMO_ATTR_VEHSPERHOUR, 0, ok, -1.);
            state.v = attrs.getOpt<SUMOReal>(SUMO_ATTR_SPEED, 0, ok, -1.);
            state.begin = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, myID.c_str(), ok);
            if (state.begin < lastEnd) {
                WRITE_ERROR("Overlapping or unsorted intervals in calibrator '" + myID + "'.");
            }
            state.end = attrs.getOptSUMOTimeReporting(SUMO_ATTR_END, myID.c_str(), ok, -1);
            state.vehicleParameter = SUMOVehicleParserHelper::parseVehicleAttributes(attrs, true, true);
            LeftoverVehicleParameters.push_back(state.vehicleParameter);
            // vehicles should be inserted with max speed unless stated otherwise
            if (state.vehicleParameter->departSpeedProcedure == DEPART_SPEED_DEFAULT) {
                state.vehicleParameter->departSpeedProcedure = DEPART_SPEED_MAX;
            }
            // vehicles should be inserted on any lane unless stated otherwise
            if (state.vehicleParameter->departLaneProcedure == DEPART_LANE_DEFAULT) {
                state.vehicleParameter->departLaneProcedure = DEPART_LANE_ALLOWED_FREE;
            }
            if (state.vehicleParameter->vtypeid != DEFAULT_VTYPE_ID &&
                    MSNet::getInstance()->getVehicleControl().getVType(state.vehicleParameter->vtypeid) == 0) {
                WRITE_ERROR("Unknown vehicle type '" + state.vehicleParameter->vtypeid + "' in calibrator '" + myID + "'.");
            }
        } catch (EmptyData) {
            WRITE_ERROR("Mandatory attribute missing in definition of calibrator '" + myID + "'.");
        } catch (NumberFormatException) {
            WRITE_ERROR("Non-numeric value for numeric attribute in definition of calibrator '" + myID + "'.");
        }
        if (state.q < 0 && state.v < 0) {
            WRITE_ERROR("Either 'vehsPerHour' or 'speed' has to be given in flow definition of calibrator '" + myID + "'.");
        }
        if (myIntervals.size() > 0 && myIntervals.back().end == -1) {
            myIntervals.back().end = state.begin;
        }
        myIntervals.push_back(state);
    } else {
        MSRouteHandler::myStartElement(element, attrs);
    }
}


void
MSCalibrator::myEndElement(int element) {
    if (element == SUMO_TAG_CALIBRATOR) {
        if (!myDidInit) {
            init();
        }
    } else if (element != SUMO_TAG_FLOW) {
        MSRouteHandler::myEndElement(element);
    }
}


void
MSCalibrator::writeXMLOutput() {
    if (myOutput != 0) {
        updateMeanData();
        const int p = passed();
        // meandata will be off if vehicles are removed on the next edge instead of this one
        const int discrepancy = myEdgeMeanData.nVehEntered + myEdgeMeanData.nVehDeparted - myEdgeMeanData.nVehVaporized - passed();
        assert(discrepancy >= 0);
        const std::string ds = (discrepancy > 0 ? "\" vaporizedOnNextEdge=\"" + toString(discrepancy) : "");
        const SUMOReal durationSeconds = STEPS2TIME(myCurrentStateInterval->end - myCurrentStateInterval->begin);
        (*myOutput) << "   <interval begin=\"" << time2string(myCurrentStateInterval->begin) <<
                    "\" end=\"" << time2string(myCurrentStateInterval->end) <<
                    "\" id=\"" << myID <<
                    "\" nVehContrib=\"" << p <<
                    "\" removed=\"" << myRemoved <<
                    "\" inserted=\"" << myInserted <<
                    "\" cleared=\"" << myClearedInJam <<
                    "\" flow=\"" << p * 3600.0 / durationSeconds <<
                    "\" aspiredFlow=\"" << myCurrentStateInterval->q <<
                    "\" speed=\"" << myEdgeMeanData.travelledDistance / myEdgeMeanData.getSamples() <<
                    "\" aspiredSpeed=\"" << myCurrentStateInterval->v <<
                    ds << //optional
                    "\"/>\n";
    }
    myDidSpeedAdaption = false;
    myInserted = 0;
    myRemoved = 0;
    myClearedInJam = 0;
    myHaveWarnedAboutClearingJam = false;
    reset();
}


bool
MSCalibrator::isCurrentStateActive(SUMOTime time) {
    while (myCurrentStateInterval != myIntervals.end() && myCurrentStateInterval->end <= time) {
        // XXX what about skipped intervals?
        myCurrentStateInterval++;
    }
    return myCurrentStateInterval != myIntervals.end() &&
           myCurrentStateInterval->begin <= time && myCurrentStateInterval->end > time;
}

int
MSCalibrator::totalWished() const {
    if (myCurrentStateInterval != myIntervals.end()) {
        const SUMOReal totalHourFraction = STEPS2TIME(myCurrentStateInterval->end - myCurrentStateInterval->begin) / (SUMOReal) 3600.;
        return (int)std::floor(myCurrentStateInterval->q * totalHourFraction + 0.5); // round to closest int
    } else {
        return -1;
    }
}


bool
MSCalibrator::removePending() {
    if (myToRemove.size() > 0) {
        MSVehicleControl& vc = MSNet::getInstance()->getVehicleControl();
        // it is not save to remove the vehicles inside
        // VehicleRemover::notifyEnter so we do it here
        for (std::set<std::string>::iterator it = myToRemove.begin(); it != myToRemove.end(); ++it) {
            MSVehicle* vehicle = dynamic_cast<MSVehicle*>(vc.getVehicle(*it));
            if (vehicle != 0) {
                vehicle->onRemovalFromNet(MSMoveReminder::NOTIFICATION_VAPORIZED);
                vehicle->getLane()->removeVehicle(vehicle, MSMoveReminder::NOTIFICATION_VAPORIZED);
                vc.scheduleVehicleRemoval(vehicle);
            } else {
                WRITE_WARNING("Calibrator '" + getID() + "' could not remove vehicle '" + *it + "'.");
            }
        }
        myToRemove.clear();
        return true;
    }
    return false;
}


SUMOTime
MSCalibrator::execute(SUMOTime currentTime) {
    // get current simulation values (valid for the last simulation second)
    // XXX could we miss vehicle movements if this is called less often than every DELTA_T (default) ?
    updateMeanData();
    const bool hadRemovals = removePending();
    // check whether an adaptation value exists
    if (isCurrentStateActive(currentTime)) {
        myAmActive = true;
        // all happens in isCurrentStateActive()
    } else {
        myAmActive = false;
        reset();
        if (!mySpeedIsDefault) {
            // reset speed to default
            myEdge->setMaxSpeed(myDefaultSpeed);
            mySpeedIsDefault = true;
        }
        if (myCurrentStateInterval == myIntervals.end()) {
            // keep calibrator alive for gui but do not call again
            return TIME2STEPS(86400);
        }
        return myFrequency;
    }
    // we are active
    if (!myDidSpeedAdaption && myCurrentStateInterval->v >= 0) {
        myEdge->setMaxSpeed(myCurrentStateInterval->v);
        mySpeedIsDefault = false;
        myDidSpeedAdaption = true;
    }

    const bool calibrateFlow = myCurrentStateInterval->q >= 0;
    const int totalWishedNum = totalWished();
    int adaptedNum = passed() + myClearedInJam;
#ifdef MSCalibrator_DEBUG
    std::cout << time2string(currentTime) << " " << myID
              << " q=" << myCurrentStateInterval->q
              << " totalWished=" << totalWishedNum
              << " adapted=" << adaptedNum
              << " jam=" << invalidJam(-1)
              << " entered=" << myEdgeMeanData.nVehEntered
              << " departed=" << myEdgeMeanData.nVehDeparted
              << " arrived=" << myEdgeMeanData.nVehArrived
              << " left=" << myEdgeMeanData.nVehLeft
              << " waitSecs=" << myEdgeMeanData.waitSeconds
              << " vaporized=" << myEdgeMeanData.nVehVaporized
              << "\n";
#endif
    if (calibrateFlow && adaptedNum < totalWishedNum && !hadRemovals) {
        // we need to insert some vehicles
        const SUMOReal hourFraction = STEPS2TIME(currentTime - myCurrentStateInterval->begin + DELTA_T) / (SUMOReal) 3600.;
        const int wishedNum = (int)std::floor(myCurrentStateInterval->q * hourFraction + 0.5); // round to closest int
        // only the difference between inflow and aspiredFlow should be added, thus
        // we should not count vehicles vaporized from a jam here
        // if we have enough time left we can add missing vehicles later
        const int relaxedInsertion = (int)std::floor(STEPS2TIME(myCurrentStateInterval->end - currentTime) / 3);
        const int insertionSlack = MAX2(0, adaptedNum + relaxedInsertion - totalWishedNum);
        // increase number of vehicles
#ifdef MSCalibrator_DEBUG
        std::cout
                << "   wished:" << wishedNum
                << " slack:" << insertionSlack
                << " before:" << adaptedNum
                << "\n";
#endif
        while (wishedNum > adaptedNum + insertionSlack) {
            SUMOVehicleParameter* pars = myCurrentStateInterval->vehicleParameter;
            const MSRoute* route = myProbe != 0 ? myProbe->getRoute() : 0;
            if (route == 0) {
                route = MSRoute::dictionary(pars->routeid);
            }
            if (route == 0) {
                WRITE_WARNING("No valid routes in calibrator '" + myID + "'.");
                break;
            }
            if (!route->contains(myEdge)) {
                WRITE_WARNING("Route '" + route->getID() + "' in calibrator '" + myID + "' does not contain edge '" + myEdge->getID() + "'.");
                break;
            }
            const int routeIndex = (int)std::distance(route->begin(),
                                            std::find(route->begin(), route->end(), myEdge));
            MSVehicleType* vtype = MSNet::getInstance()->getVehicleControl().getVType(pars->vtypeid);
            assert(route != 0 && vtype != 0);
            // build the vehicle
            SUMOVehicleParameter* newPars = new SUMOVehicleParameter(*pars);
            newPars->id = myID + "." + toString((int)STEPS2TIME(myCurrentStateInterval->begin)) + "." + toString(myInserted);
            newPars->depart = currentTime;
            newPars->routeid = route->getID();
            MSVehicle* vehicle = dynamic_cast<MSVehicle*>(MSNet::getInstance()->getVehicleControl().buildVehicle(
                                     newPars, route, vtype, true, false));
#ifdef MSCalibrator_DEBUG
            std::cout << " resetting route pos: " << routeIndex << "\n";
#endif
            vehicle->resetRoutePosition(routeIndex);
            if (myEdge->insertVehicle(*vehicle, currentTime)) {
                if (!MSNet::getInstance()->getVehicleControl().addVehicle(vehicle->getID(), vehicle)) {
                    throw ProcessError("Emission of vehicle '" + vehicle->getID() + "' in calibrator '" + getID() + "'failed!");
                }
                myInserted++;
                adaptedNum++;
#ifdef MSCalibrator_DEBUG
                std::cout << "I ";
#endif
            } else {
                // could not insert vehicle
#ifdef MSCalibrator_DEBUG
                std::cout << "F ";
#endif
                MSNet::getInstance()->getVehicleControl().deleteVehicle(vehicle, true);
                break;
            }
        }
    }
    if (myCurrentStateInterval->end <= currentTime + myFrequency) {
        writeXMLOutput();
    }
    return myFrequency;
}

void
MSCalibrator::reset() {
    myEdgeMeanData.reset();
    for (std::vector<MSMeanData_Net::MSLaneMeanDataValues*>::iterator it = myLaneMeanData.begin(); it != myLaneMeanData.end(); ++it) {
        (*it)->reset();
    }
}


bool
MSCalibrator::invalidJam(int laneIndex) const {
    if (laneIndex < 0) {
        const int numLanes = (int)myEdge->getLanes().size();
        for (int i = 0; i < numLanes; ++i) {
            if (invalidJam(i)) {
                return true;
            }
        }
        return false;
    }
    assert(laneIndex < (int)myEdge->getLanes().size());
    const MSLane* const lane = myEdge->getLanes()[laneIndex];
    if (lane->getVehicleNumber() < 4) {
        // cannot reliably detect invalid jams
        return false;
    }
    // maxSpeed reflects the calibration target
    const bool toSlow = lane->getMeanSpeed() < 0.5 * myEdge->getSpeedLimit();
    return toSlow && remainingVehicleCapacity(laneIndex) < 1;
}


int
MSCalibrator::remainingVehicleCapacity(int laneIndex) const {
    if (laneIndex < 0) {
        const int numLanes = (int)myEdge->getLanes().size();
        int result = 0;
        for (int i = 0; i < numLanes; ++i) {
            result = MAX2(result, remainingVehicleCapacity(i));
        }
        return result;
    }
    assert(laneIndex < (int)myEdge->getLanes().size());
    MSLane* lane = myEdge->getLanes()[laneIndex];
    MSVehicle* last = lane->getLastFullVehicle();
    const SUMOVehicleParameter* pars = myCurrentStateInterval->vehicleParameter;
    const MSVehicleType* vtype = MSNet::getInstance()->getVehicleControl().getVType(pars->vtypeid);
    const SUMOReal spacePerVehicle = vtype->getLengthWithGap() + myEdge->getSpeedLimit() * vtype->getCarFollowModel().getHeadwayTime();
    if (last == 0) {
        // ensure vehicles can be inserted on short edges
        return MAX2(1, (int)(myEdge->getLength() / spacePerVehicle));
    } else {
        return (int)(last->getPositionOnLane() / spacePerVehicle);
    }
}


void
MSCalibrator::cleanup() {
    for (std::vector<MSMoveReminder*>::iterator it = LeftoverReminders.begin(); it != LeftoverReminders.end(); ++it) {
        delete *it;
    }
    LeftoverReminders.clear();
    for (std::vector<SUMOVehicleParameter*>::iterator it = LeftoverVehicleParameters.begin();
            it != LeftoverVehicleParameters.end(); ++it) {
        delete *it;
    }
    LeftoverVehicleParameters.clear();
}


void
MSCalibrator::updateMeanData() {
    myEdgeMeanData.reset();
    for (std::vector<MSMeanData_Net::MSLaneMeanDataValues*>::iterator it = myLaneMeanData.begin();
            it != myLaneMeanData.end(); ++it) {
        (*it)->addTo(myEdgeMeanData);
    }
}

bool MSCalibrator::VehicleRemover::notifyEnter(SUMOVehicle& veh, Notification /* reason */) {
    if (myParent == 0) {
        return false;
    }
    if (myParent->isActive()) {
        myParent->updateMeanData();
        const bool calibrateFlow = myParent->myCurrentStateInterval->q >= 0;
        const int totalWishedNum = myParent->totalWished();
        int adaptedNum = myParent->passed() + myParent->myClearedInJam;
        MSVehicle* vehicle = dynamic_cast<MSVehicle*>(&veh);
        if (calibrateFlow && adaptedNum > totalWishedNum) {
#ifdef MSCalibrator_DEBUG
            std::cout << time2string(MSNet::getInstance()->getCurrentTimeStep()) << " " << myParent->getID()
                      << " vaporizing " << vehicle->getID() << " to reduce flow\n";
#endif
            if (myParent->scheduleRemoval(vehicle)) {
                myParent->myRemoved++;
            }
        } else if (myParent->invalidJam(myLaneIndex)) {
#ifdef MSCalibrator_DEBUG
            std::cout << time2string(MSNet::getInstance()->getCurrentTimeStep()) << " " << myParent->getID()
                      << " vaporizing " << vehicle->getID() << " to clear jam\n";
#endif
            if (!myParent->myHaveWarnedAboutClearingJam) {
                WRITE_WARNING("Clearing jam at calibrator '" + myParent->myID + "' at time "
                              + time2string(MSNet::getInstance()->getCurrentTimeStep()));
                myParent->myHaveWarnedAboutClearingJam = true;
            }
            if (myParent->scheduleRemoval(vehicle)) {
                myParent->myClearedInJam++;
            }
        }
    }
    return true;
}



/****************************************************************************/

