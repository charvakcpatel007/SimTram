/****************************************************************************/
/// @file    MSMeanData.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Mon, 10.05.2004
/// @version $Id: MSMeanData.cpp 20482 2016-04-18 20:49:42Z behrisch $
///
// Data collector for edges/lanes
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

#include <limits>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSNet.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSMeanData_Amitran.h"
#include "MSMeanData.h"

#include <microsim/MSGlobals.h>
#include <mesosim/MESegment.h>
#include <mesosim/MELoop.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// MSMeanData::MeanDataValues - methods
// ---------------------------------------------------------------------------
MSMeanData::MeanDataValues::MeanDataValues(
    MSLane* const lane, const SUMOReal length, const bool doAdd,
    const std::set<std::string>* const vTypes) :
    MSMoveReminder("meandata_" + (lane == 0 ? "NULL" :  lane->getID()), lane, doAdd),
    myLaneLength(length),
    sampleSeconds(0),
    travelledDistance(0),
    myVehicleTypes(vTypes) {}


MSMeanData::MeanDataValues::~MeanDataValues() {
}


bool
MSMeanData::MeanDataValues::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason) {
    UNUSED_PARAMETER(reason);
    return vehicleApplies(veh);
}


bool
MSMeanData::MeanDataValues::notifyMove(SUMOVehicle& veh, SUMOReal oldPos, SUMOReal newPos, SUMOReal newSpeed) {
    // if the vehicle has arrived, the reminder must be kept so it can be
    // notified of the arrival subsequently
    SUMOReal timeOnLane = TS;
    bool ret = true;
    if (oldPos < 0 && newSpeed != 0) {
        timeOnLane = newPos / newSpeed;
    }
    if (newPos - veh.getVehicleType().getLength() > myLaneLength && newSpeed != 0) {
        timeOnLane -= (newPos - veh.getVehicleType().getLength() - myLaneLength) / newSpeed;
        if (fabs(timeOnLane) < 0.001) { // reduce rounding errors
            timeOnLane = 0.;
        }
        ret = veh.hasArrived();
    }
    if (timeOnLane < 0) {
        WRITE_ERROR("Negative vehicle step fraction for '" + veh.getID() + "' on lane '" + getLane()->getID() + "'.");
        return veh.hasArrived();
    }
    if (timeOnLane == 0) {
        return veh.hasArrived();
    }
    notifyMoveInternal(veh, timeOnLane, newSpeed);
    return ret;
}


bool
MSMeanData::MeanDataValues::notifyLeave(SUMOVehicle& /*veh*/, SUMOReal /*lastPos*/, MSMoveReminder::Notification reason) {
    if (MSGlobals::gUseMesoSim) {
        return false; // reminder is re-added on every segment (@recheck for performance)
    }
    return reason == MSMoveReminder::NOTIFICATION_JUNCTION;
}


bool
MSMeanData::MeanDataValues::vehicleApplies(const SUMOVehicle& veh) const {
    return myVehicleTypes == 0 || myVehicleTypes->empty() ||
           myVehicleTypes->find(veh.getVehicleType().getID()) != myVehicleTypes->end();
}


bool
MSMeanData::MeanDataValues::isEmpty() const {
    return sampleSeconds == 0;
}


void
MSMeanData::MeanDataValues::update() {
}


SUMOReal
MSMeanData::MeanDataValues::getSamples() const {
    return sampleSeconds;
}


// ---------------------------------------------------------------------------
// MSMeanData::MeanDataValueTracker - methods
// ---------------------------------------------------------------------------
MSMeanData::MeanDataValueTracker::MeanDataValueTracker(MSLane* const lane,
        const SUMOReal length,
        const std::set<std::string>* const vTypes,
        const MSMeanData* const parent)
    : MSMeanData::MeanDataValues(lane, length, true, vTypes), myParent(parent) {
    myCurrentData.push_back(new TrackerEntry(parent->createValues(lane, length, false)));
}


MSMeanData::MeanDataValueTracker::~MeanDataValueTracker() {
    std::list<TrackerEntry*>::iterator i;
    for (i = myCurrentData.begin(); i != myCurrentData.end(); i++) {
        delete *i;
    }

    // FIXME: myTrackedData may still hold some undeleted TrackerEntries. When to delete those? (Leo), refers to #2251
    // code below fails

//	std::map<SUMOVehicle*, TrackerEntry*>::iterator j;
//	for(j=myTrackedData.begin(); j!=myTrackedData.end();j++){
//		delete j->second;
//	}
}


void
MSMeanData::MeanDataValueTracker::reset(bool afterWrite) {
    if (afterWrite) {
        if (myCurrentData.begin() != myCurrentData.end()) {
            myCurrentData.pop_front();
        }
    } else {
        myCurrentData.push_back(new TrackerEntry(myParent->createValues(myLane, myLaneLength, false)));
    }
}


void
MSMeanData::MeanDataValueTracker::addTo(MSMeanData::MeanDataValues& val) const {
    myCurrentData.front()->myValues->addTo(val);
}


void
MSMeanData::MeanDataValueTracker::notifyMoveInternal(SUMOVehicle& veh, SUMOReal timeOnLane, SUMOReal speed) {
    myTrackedData[&veh]->myValues->notifyMoveInternal(veh, timeOnLane, speed);
}


bool
MSMeanData::MeanDataValueTracker::notifyLeave(SUMOVehicle& veh, SUMOReal lastPos, MSMoveReminder::Notification reason) {
    if (myParent == 0 || reason != MSMoveReminder::NOTIFICATION_SEGMENT) {
        myTrackedData[&veh]->myNumVehicleLeft++;
    }
    return myTrackedData[&veh]->myValues->notifyLeave(veh, lastPos, reason);
}


bool
MSMeanData::MeanDataValueTracker::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason) {
    if (reason == MSMoveReminder::NOTIFICATION_SEGMENT) {
        return true;
    }
    if (vehicleApplies(veh) && myTrackedData.find(&veh) == myTrackedData.end()) {
        myTrackedData[&veh] = myCurrentData.back();
        myTrackedData[&veh]->myNumVehicleEntered++;
        if (!myTrackedData[&veh]->myValues->notifyEnter(veh, reason)) {
            myTrackedData[&veh]->myNumVehicleLeft++;
            myTrackedData.erase(&veh);
            return false;
        }
        return true;
    }
    return false;
}


bool
MSMeanData::MeanDataValueTracker::isEmpty() const {
    return myCurrentData.front()->myValues->isEmpty();
}


void
MSMeanData::MeanDataValueTracker::write(OutputDevice& dev,
                                        const SUMOTime period,
                                        const SUMOReal numLanes,
                                        const SUMOReal defaultTravelTime,
                                        const int /*numVehicles*/) const {
    myCurrentData.front()->myValues->write(dev, period, numLanes,
                                           defaultTravelTime,
                                           myCurrentData.front()->myNumVehicleEntered);
}


int
MSMeanData::MeanDataValueTracker::getNumReady() const {
    int result = 0;
    for (std::list<TrackerEntry*>::const_iterator it = myCurrentData.begin(); it != myCurrentData.end(); ++it) {
        if ((*it)->myNumVehicleEntered == (*it)->myNumVehicleLeft) {
            result++;
        } else {
            break;
        }
    }
    return result;
}


SUMOReal
MSMeanData::MeanDataValueTracker::getSamples() const {
    return myCurrentData.front()->myValues->getSamples();
}


// ---------------------------------------------------------------------------
// MSMeanData - methods
// ---------------------------------------------------------------------------
MSMeanData::MSMeanData(const std::string& id,
                       const SUMOTime dumpBegin, const SUMOTime dumpEnd,
                       const bool useLanes, const bool withEmpty,
                       const bool printDefaults, const bool withInternal, const bool trackVehicles,
                       const SUMOReal maxTravelTime,
                       const SUMOReal minSamples,
                       const std::set<std::string> vTypes) :
    MSDetectorFileOutput(id),
    myMinSamples(minSamples),
    myMaxTravelTime(maxTravelTime),
    myVehicleTypes(vTypes),
    myDumpEmpty(withEmpty),
    myAmEdgeBased(!useLanes),
    myDumpBegin(dumpBegin),
    myDumpEnd(dumpEnd),
    myPrintDefaults(printDefaults),
    myDumpInternal(withInternal),
    myTrackVehicles(trackVehicles) {
}


void
MSMeanData::init() {
    const MSEdgeVector& edges = MSNet::getInstance()->getEdgeControl().getEdges();
    for (MSEdgeVector::const_iterator e = edges.begin(); e != edges.end(); ++e) {
        const MSEdge::EdgeBasicFunction efun = (*e)->getPurpose();
        if ((myDumpInternal || efun != MSEdge::EDGEFUNCTION_INTERNAL)
                && efun != MSEdge::EDGEFUNCTION_CROSSING && efun != MSEdge::EDGEFUNCTION_WALKINGAREA) {
            myEdges.push_back(*e);
            myMeasures.push_back(std::vector<MeanDataValues*>());
            const std::vector<MSLane*>& lanes = (*e)->getLanes();
            if (MSGlobals::gUseMesoSim) {
                MeanDataValues* data;
                if (myTrackVehicles) {
                    data = new MeanDataValueTracker(0, lanes[0]->getLength(), &myVehicleTypes, this);
                } else {
                    data = createValues(0, lanes[0]->getLength(), false);
                }
                data->setDescription("meandata_" + (*e)->getID());
                myMeasures.back().push_back(data);
                MESegment* s = MSGlobals::gMesoNet->getSegmentForEdge(**e);
                while (s != 0) {
                    s->addDetector(data);
                    s->prepareDetectorForWriting(*data);
                    s = s->getNextSegment();
                }
                data->reset();
                data->reset(true);
                continue;
            }
            if (myAmEdgeBased && myTrackVehicles) {
                myMeasures.back().push_back(new MeanDataValueTracker(0, lanes[0]->getLength(), &myVehicleTypes, this));
            }
            for (std::vector<MSLane*>::const_iterator lane = lanes.begin(); lane != lanes.end(); ++lane) {
                if (myTrackVehicles) {
                    if (myAmEdgeBased) {
                        (*lane)->addMoveReminder(myMeasures.back().back());
                    } else {
                        myMeasures.back().push_back(new MeanDataValueTracker(*lane, (*lane)->getLength(), &myVehicleTypes, this));
                    }
                } else {
                    myMeasures.back().push_back(createValues(*lane, (*lane)->getLength(), true));
                }
            }
        }
    }
}


MSMeanData::~MSMeanData() {
    for (std::vector<std::vector<MeanDataValues*> >::const_iterator i = myMeasures.begin(); i != myMeasures.end(); ++i) {
        for (std::vector<MeanDataValues*>::const_iterator j = (*i).begin(); j != (*i).end(); ++j) {
            delete *j;
        }
    }
}


void
MSMeanData::resetOnly(SUMOTime stopTime) {
    UNUSED_PARAMETER(stopTime);
    if (MSGlobals::gUseMesoSim) {
        MSEdgeVector::iterator edge = myEdges.begin();
        for (std::vector<std::vector<MeanDataValues*> >::const_iterator i = myMeasures.begin(); i != myMeasures.end(); ++i, ++edge) {
            MESegment* s = MSGlobals::gMesoNet->getSegmentForEdge(**edge);
            MeanDataValues* data = i->front();
            while (s != 0) {
                s->prepareDetectorForWriting(*data);
                s = s->getNextSegment();
            }
            data->reset();
        }
        return;
    }
    for (std::vector<std::vector<MeanDataValues*> >::const_iterator i = myMeasures.begin(); i != myMeasures.end(); ++i) {
        for (std::vector<MeanDataValues*>::const_iterator j = (*i).begin(); j != (*i).end(); ++j) {
            (*j)->reset();
        }
    }
}


std::string
MSMeanData::getEdgeID(const MSEdge* const edge) {
    return edge->getID();
}


void
MSMeanData::writeEdge(OutputDevice& dev,
                      const std::vector<MeanDataValues*>& edgeValues,
                      MSEdge* edge, SUMOTime startTime, SUMOTime stopTime) {
    if (MSGlobals::gUseMesoSim) {
        MESegment* s = MSGlobals::gMesoNet->getSegmentForEdge(*edge);
        MeanDataValues* data = edgeValues.front();
        while (s != 0) {
            s->prepareDetectorForWriting(*data);
            s = s->getNextSegment();
        }
        if (writePrefix(dev, *data, SUMO_TAG_EDGE, getEdgeID(edge))) {
            data->write(dev, stopTime - startTime,
                        (SUMOReal)edge->getLanes().size(),
                        myPrintDefaults ? edge->getLength() / edge->getSpeedLimit() : -1.);
        }
        data->reset(true);
        return;
    }
    std::vector<MeanDataValues*>::const_iterator lane;
    if (!myAmEdgeBased) {
        bool writeCheck = myDumpEmpty;
        if (!writeCheck) {
            for (lane = edgeValues.begin(); lane != edgeValues.end(); ++lane) {
                if (!(*lane)->isEmpty()) {
                    writeCheck = true;
                    break;
                }
            }
        }
        if (writeCheck) {
            dev.openTag(SUMO_TAG_EDGE).writeAttr(SUMO_ATTR_ID, edge->getID());
        }
        for (lane = edgeValues.begin(); lane != edgeValues.end(); ++lane) {
            MeanDataValues& meanData = **lane;
            if (writePrefix(dev, meanData, SUMO_TAG_LANE, meanData.getLane()->getID())) {
                meanData.write(dev, stopTime - startTime, 1.f, myPrintDefaults ? meanData.getLane()->getLength() / meanData.getLane()->getSpeedLimit() : -1.);
            }
            meanData.reset(true);
        }
        if (writeCheck) {
            dev.closeTag();
        }
    } else {
        if (myTrackVehicles) {
            MeanDataValues& meanData = **edgeValues.begin();
            if (writePrefix(dev, meanData, SUMO_TAG_EDGE, edge->getID())) {
                meanData.write(dev, stopTime - startTime, (SUMOReal)edge->getLanes().size(), myPrintDefaults ? edge->getLength() / edge->getSpeedLimit() : -1.);
            }
            meanData.reset(true);
        } else {
            MeanDataValues* sumData = createValues(0, edge->getLength(), false);
            for (lane = edgeValues.begin(); lane != edgeValues.end(); ++lane) {
                MeanDataValues& meanData = **lane;
                meanData.addTo(*sumData);
                meanData.reset();
            }
            if (writePrefix(dev, *sumData, SUMO_TAG_EDGE, getEdgeID(edge))) {
                sumData->write(dev, stopTime - startTime, (SUMOReal)edge->getLanes().size(), myPrintDefaults ? edge->getLength() / edge->getSpeedLimit() : -1.);
            }
            delete sumData;
        }
    }
}


void
MSMeanData::openInterval(OutputDevice& dev, const SUMOTime startTime, const SUMOTime stopTime) {
    dev.openTag(SUMO_TAG_INTERVAL).writeAttr(SUMO_ATTR_BEGIN, STEPS2TIME(startTime)).writeAttr(SUMO_ATTR_END, STEPS2TIME(stopTime));
    dev.writeAttr(SUMO_ATTR_ID, myID);
}


bool
MSMeanData::writePrefix(OutputDevice& dev, const MeanDataValues& values, const SumoXMLTag tag, const std::string id) const {
    if (myDumpEmpty || !values.isEmpty()) {
        dev.openTag(tag).writeAttr(SUMO_ATTR_ID, id).writeAttr("sampledSeconds", values.getSamples());
        return true;
    }
    return false;
}


void
MSMeanData::writeXMLOutput(OutputDevice& dev,
                           SUMOTime startTime, SUMOTime stopTime) {
    // check whether this dump shall be written for the current time
    int numReady = myDumpBegin < stopTime && myDumpEnd - DELTA_T >= startTime ? 1 : 0;
    if (myTrackVehicles && myDumpBegin < stopTime) {
        myPendingIntervals.push_back(std::make_pair(startTime, stopTime));
        numReady = (int)myPendingIntervals.size();
        for (std::vector<std::vector<MeanDataValues*> >::const_iterator i = myMeasures.begin(); i != myMeasures.end(); ++i) {
            for (std::vector<MeanDataValues*>::const_iterator j = (*i).begin(); j != (*i).end(); ++j) {
                numReady = MIN2(numReady, ((MeanDataValueTracker*)*j)->getNumReady());
                if (numReady == 0) {
                    break;
                }
            }
            if (numReady == 0) {
                break;
            }
        }
    }
    if (numReady == 0 || myTrackVehicles) {
        resetOnly(stopTime);
    }
    while (numReady-- > 0) {
        if (!myPendingIntervals.empty()) {
            startTime = myPendingIntervals.front().first;
            stopTime = myPendingIntervals.front().second;
            myPendingIntervals.pop_front();
        }
        openInterval(dev, startTime, stopTime);
        MSEdgeVector::iterator edge = myEdges.begin();
        for (std::vector<std::vector<MeanDataValues*> >::const_iterator i = myMeasures.begin(); i != myMeasures.end(); ++i, ++edge) {
            writeEdge(dev, (*i), *edge, startTime, stopTime);
        }
        dev.closeTag();
    }
}


void
MSMeanData::writeXMLDetectorProlog(OutputDevice& dev) const {
    dev.writeXMLHeader("meandata", "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.dlr.de/xsd/meandata_file.xsd\"");
}


void
MSMeanData::detectorUpdate(const SUMOTime step) {
    if (step + DELTA_T == myDumpBegin) {
        init();
    }
}


/****************************************************************************/

