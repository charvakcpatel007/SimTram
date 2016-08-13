/****************************************************************************/
/// @file    MSDevice_Tripinfo.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Fri, 30.01.2009
/// @version $Id: MSDevice_Tripinfo.cpp 20674 2016-05-09 14:44:37Z namdre $
///
// A device which collects info on the vehicle trip
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2009-2016 DLR (http://www.dlr.de/) and contributors
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

#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSDevice_Tripinfo.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

#define NOT_ARRIVED TIME2STEPS(-1)


// ===========================================================================
// static members
// ===========================================================================
MSDevice_Tripinfo::DeviceSet MSDevice_Tripinfo::myPendingOutput;

SUMOReal MSDevice_Tripinfo::myVehicleCount(0);
SUMOReal MSDevice_Tripinfo::myTotalRouteLength(0);
SUMOTime MSDevice_Tripinfo::myTotalDuration(0);
SUMOTime MSDevice_Tripinfo::myTotalWaitingTime(0);
SUMOTime MSDevice_Tripinfo::myTotalTimeLoss(0);
SUMOTime MSDevice_Tripinfo::myTotalDepartDelay(0);

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Tripinfo::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into) {
    if (OptionsCont::getOptions().isSet("tripinfo-output") || OptionsCont::getOptions().getBool("duration-log.statistics")) {
        MSDevice_Tripinfo* device = new MSDevice_Tripinfo(v, "tripinfo_" + v.getID());
        into.push_back(device);
        myPendingOutput.insert(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_Tripinfo-methods
// ---------------------------------------------------------------------------
MSDevice_Tripinfo::MSDevice_Tripinfo(SUMOVehicle& holder, const std::string& id) :
    MSDevice(holder, id),
    myDepartLane(""),
    myDepartSpeed(-1),
    myWaitingSteps(0),
    myArrivalTime(NOT_ARRIVED),
    myArrivalLane(""),
    myArrivalPos(-1),
    myArrivalSpeed(-1),
    myTimeLoss(0) {
}


MSDevice_Tripinfo::~MSDevice_Tripinfo() {
    // ensure clean up for vaporized vehicles which do not generate output
    myPendingOutput.erase(this);
}


bool
MSDevice_Tripinfo::notifyMove(SUMOVehicle& veh, SUMOReal /*oldPos*/,
                              SUMOReal /*newPos*/, SUMOReal newSpeed) {
    if (veh.isStopped()) {
        return true;
    }
    if (newSpeed <= SUMO_const_haltingSpeed) {
        myWaitingSteps++;
    }
    // @note we are including the speed factor here, thus myTimeLoss can never be
    // negative. The value is that of a driver who compares his travel time when
    // the road is clear (which includes speed factor) with the actual travel time.
    // @todo It might be usefull to recognize a departing vehicle and not
    // count the time spent accelerating towards time loss since it is unavoidable
    // (current interfaces do not give access to maximum acceleration)
    const SUMOReal vmax = MIN2(veh.getMaxSpeed(), veh.getEdge()->getVehicleMaxSpeed(&veh));
    if (vmax > 0) {
        myTimeLoss += TIME2STEPS(TS * (vmax - newSpeed) / vmax);
    }
    return true;
}


void
MSDevice_Tripinfo::notifyMoveInternal(SUMOVehicle& veh, SUMOReal timeOnLane, SUMOReal speed) {
    // called by meso
    const SUMOReal vmax = veh.getEdge()->getVehicleMaxSpeed(&veh);
    if (vmax > 0) {
        myTimeLoss += TIME2STEPS(timeOnLane - (timeOnLane * speed / vmax));
    }
    myWaitingSteps += veh.getWaitingTime() / DELTA_T;
}


bool
MSDevice_Tripinfo::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason) {
    if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
        if (!MSGlobals::gUseMesoSim) {
            myDepartLane = static_cast<MSVehicle&>(veh).getLane()->getID();
        }
        myDepartSpeed = veh.getSpeed();
    }
    return true;
}


bool
MSDevice_Tripinfo::notifyLeave(SUMOVehicle& veh, SUMOReal /*lastPos*/,
                               MSMoveReminder::Notification reason) {
    if (reason >= MSMoveReminder::NOTIFICATION_ARRIVED) {
        myArrivalTime = MSNet::getInstance()->getCurrentTimeStep();
        if (!MSGlobals::gUseMesoSim) {
            myArrivalLane = static_cast<MSVehicle&>(veh).getLane()->getID();
        }
        // @note vehicle may have moved past its arrivalPos during the last step
        // due to non-zero arrivalspeed but we consider it as arrived at the desired position
        // However, vaporization may happen anywhere (via TraCI)
        if (reason == MSMoveReminder::NOTIFICATION_VAPORIZED) {
            myArrivalPos = veh.getPositionOnLane();
        } else {
            myArrivalPos = myHolder.getArrivalPos();
        }
        myArrivalSpeed = veh.getSpeed();
    }
    return true;
}

void
MSDevice_Tripinfo::computeLengthAndDuration(SUMOReal& routeLength, SUMOTime& duration) const {
    SUMOTime finalTime;
    SUMOReal finalPos;
    SUMOReal finalPosOnInternal = 0;
    if (myArrivalTime == NOT_ARRIVED) {
        finalTime = MSNet::getInstance()->getCurrentTimeStep();
        finalPos = myHolder.getPositionOnLane();
        if (!MSGlobals::gUseMesoSim) {
            const MSLane* lane = static_cast<MSVehicle&>(myHolder).getLane();
            if (lane->getEdge().isInternal()) {
                finalPosOnInternal = finalPos;
                finalPos = myHolder.getEdge()->getLength();
            }
        }
    } else {
        finalTime = myArrivalTime;
        finalPos = myArrivalPos;
    }
    const bool includeInternalLengths = MSGlobals::gUsingInternalLanes && MSNet::getInstance()->hasInternalLinks();
    routeLength = myHolder.getRoute().getDistanceBetween(myHolder.getDepartPos(), finalPos,
                  myHolder.getRoute().begin(), myHolder.getCurrentRouteEdge(), includeInternalLengths) + finalPosOnInternal;

    duration = finalTime - myHolder.getDeparture();
}


void
MSDevice_Tripinfo::generateOutput() const {
    updateStatistics();
    if (!OptionsCont::getOptions().isSet("tripinfo-output")) {
        return;
    }
    myPendingOutput.erase(this);
    SUMOReal routeLength;
    SUMOTime duration;
    computeLengthAndDuration(routeLength, duration);

    // write
    OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-output");
    os.openTag("tripinfo").writeAttr("id", myHolder.getID());
    os.writeAttr("depart", time2string(myHolder.getDeparture()));
    os.writeAttr("departLane", myDepartLane);
    os.writeAttr("departPos", myHolder.getDepartPos());
    os.writeAttr("departSpeed", myDepartSpeed);
    os.writeAttr("departDelay", time2string(myHolder.getDepartDelay()));
    os.writeAttr("arrival", time2string(myArrivalTime));
    os.writeAttr("arrivalLane", myArrivalLane);
    os.writeAttr("arrivalPos", myArrivalPos);
    os.writeAttr("arrivalSpeed", myArrivalSpeed);
    os.writeAttr("duration", time2string(duration));
    os.writeAttr("routeLength", routeLength);
    os.writeAttr("waitSteps", myWaitingSteps);
    os.writeAttr("timeLoss", time2string(myTimeLoss));
    os.writeAttr("rerouteNo", myHolder.getNumberReroutes());
    const std::vector<MSDevice*>& devices = myHolder.getDevices();
    std::ostringstream str;
    for (std::vector<MSDevice*>::const_iterator i = devices.begin(); i != devices.end(); ++i) {
        if (i != devices.begin()) {
            str << ' ';
        }
        str << (*i)->getID();
    }
    os.writeAttr("devices", str.str());
    os.writeAttr("vType", myHolder.getVehicleType().getID());
    os.writeAttr("vaporized", (myHolder.getEdge() == *(myHolder.getRoute().end() - 1) ? "" : "0"));
    // cannot close tag because emission device output might follow
}


void
MSDevice_Tripinfo::generateOutputForUnfinished() {
    while (myPendingOutput.size() > 0) {
        const MSDevice_Tripinfo* d = *myPendingOutput.begin();
        if (d->myHolder.hasDeparted()) {
            d->generateOutput();
            if (!OptionsCont::getOptions().isSet("tripinfo-output")) {
                return;
            }
            // @todo also generate emission output if holder has a device
            OutputDevice::getDeviceByOption("tripinfo-output").closeTag();
        } else {
            myPendingOutput.erase(d);
        }
    }
}


void
MSDevice_Tripinfo::updateStatistics() const {
    SUMOReal routeLength;
    SUMOTime duration;
    computeLengthAndDuration(routeLength, duration);

    myVehicleCount++;
    myTotalRouteLength += routeLength;
    myTotalDuration += duration;
    myTotalWaitingTime += (SUMOTime)(myWaitingSteps * DELTA_T);
    myTotalTimeLoss += myTimeLoss;
    myTotalDepartDelay += myHolder.getDepartDelay();
}


std::string
MSDevice_Tripinfo::printStatistics() {
    std::ostringstream msg;
    msg.setf(msg.fixed);
    msg.precision(OUTPUT_ACCURACY);
    msg << "Statistics (avg):\n"
        << " RouteLength: " << getAvgRouteLength() << "\n"
        << " Duration: " << getAvgDuration() << "\n"
        << " WaitingTime: " << getAvgWaitingTime() << "\n"
        << " TimeLoss: " << getAvgTimeLoss() << "\n"
        << " DepartDelay: " << getAvgDepartDelay() << "\n";
    return msg.str();
}


SUMOReal
MSDevice_Tripinfo::getAvgRouteLength() {
    if (myVehicleCount > 0) {
        return myTotalRouteLength / myVehicleCount;
    } else {
        return 0;
    }
}

SUMOReal
MSDevice_Tripinfo::getAvgDuration() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalDuration / myVehicleCount);
    } else {
        return 0;
    }
}

SUMOReal
MSDevice_Tripinfo::getAvgWaitingTime() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalWaitingTime / myVehicleCount);
    } else {
        return 0;
    }
}

SUMOReal
MSDevice_Tripinfo::getAvgTimeLoss() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalTimeLoss / myVehicleCount);
    } else {
        return 0;
    }
}

SUMOReal
MSDevice_Tripinfo::getAvgDepartDelay() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalDepartDelay / myVehicleCount);
    } else {
        return 0;
    }
}


/****************************************************************************/

