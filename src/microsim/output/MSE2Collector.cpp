/****************************************************************************/
/// @file    MSE2Collector.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Robbin Blokpoel
/// @author  Jakob Erdmann
/// @date    Mon Feb 03 2014 10:13 CET
/// @version $Id: MSE2Collector.cpp 21201 2016-07-19 11:57:22Z behrisch $
///
// An areal (along a single lane) detector
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2014 DLR (http://www.dlr.de/) and contributors
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
#include <algorithm>
#include "MSE2Collector.h"
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleType.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MSE2Collector::MSE2Collector(const std::string& id, DetectorUsage usage,
                             MSLane* const lane, SUMOReal startPos, SUMOReal detLength,
                             SUMOTime haltingTimeThreshold,
                             SUMOReal haltingSpeedThreshold,
                             SUMOReal jamDistThreshold) :
    MSMoveReminder(id, lane),
    MSDetectorFileOutput(id),
    myJamHaltingSpeedThreshold(haltingSpeedThreshold),
    myJamHaltingTimeThreshold(haltingTimeThreshold),
    myJamDistanceThreshold(jamDistThreshold),
    myStartPos(startPos), myEndPos(startPos + detLength),
    myUsage(usage),
    myCurrentOccupancy(0), myCurrentMeanSpeed(-1), myCurrentJamNo(0),
    myCurrentMaxJamLengthInMeters(0), myCurrentMaxJamLengthInVehicles(0),
    myCurrentJamLengthInMeters(0), myCurrentJamLengthInVehicles(0), myCurrentStartedHalts(0),
    myCurrentHaltingsNumber(0), myPassedVeh(0)

{
    assert(myLane != 0);
    assert(myStartPos >= 0 && myStartPos < myLane->getLength());
    assert(myEndPos - myStartPos > 0 && myEndPos <= myLane->getLength());
    reset();
}


MSE2Collector::~MSE2Collector() {
    myKnownVehicles.clear();
}


bool
MSE2Collector::notifyMove(SUMOVehicle& veh, SUMOReal oldPos,
                          SUMOReal newPos, SUMOReal) {
    if (newPos <= myStartPos) {
        // detector not yet reached
        return true;
    }
    if (newPos > myStartPos && oldPos <= myStartPos) {
        if (find(myKnownVehicles.begin(), myKnownVehicles.end(), &veh) == myKnownVehicles.end()) {
            std::string type = veh.getVehicleType().getID(); // get vehicle's type
            if (type.find("COLOMBO_undetectable") == std::string::npos) {
                myKnownVehicles.push_back(&veh);
                //Detection entering the sensor
                myPassedVeh++;
                DBG(
                    std::ostringstream str;
                    str << time2string(MSNet::getInstance()->getCurrentTimeStep())
                    << " MSE2Collector::notifyMove::"
                    << " lane " << myLane->getID()
                    << " passedVeh " << myPassedVeh ;
                    WRITE_MESSAGE(str.str());
                )
            }
        }
    }
    if (newPos - veh.getVehicleType().getLength() > myEndPos) {
        std::list<SUMOVehicle*>::iterator i = find(myKnownVehicles.begin(), myKnownVehicles.end(), &veh);
        if (i != myKnownVehicles.end()) {
            myKnownVehicles.erase(i);
        }
        return false;
    }
    return true;
}


bool
MSE2Collector::notifyLeave(SUMOVehicle& veh, SUMOReal lastPos, MSMoveReminder::Notification reason) {
    if (reason != MSMoveReminder::NOTIFICATION_JUNCTION || (lastPos > myStartPos && lastPos - veh.getVehicleType().getLength() < myEndPos)) {
        std::list<SUMOVehicle*>::iterator i = find(myKnownVehicles.begin(), myKnownVehicles.end(), &veh);
        if (i != myKnownVehicles.end()) {
            myKnownVehicles.erase(i);
        }
        return false;
    }
    return true;
}


bool
MSE2Collector::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason) {
    if (!veh.isOnRoad()) {
        // vehicle is teleporting over the edge
        return false;
    }
    if (veh.getBackPositionOnLane(myLane) >= myEndPos) {
        // vehicle is beyond the detector
        return false;
    }
    if (reason != MSMoveReminder::NOTIFICATION_JUNCTION && veh.getPositionOnLane() > myStartPos) {
        // the junction case is handled in the notifyMove
        // vehicle is on the detector, being already beyond was checked before
        std::string type = veh.getVehicleType().getID(); // get vehicle's type
        if (type.find("COLOMBO_undetectable") == std::string::npos) {
            myKnownVehicles.push_back(&veh);
            //Detection entering the sensor
            myPassedVeh++;
            DBG(
                std::ostringstream str;
                str << time2string(MSNet::getInstance()->getCurrentTimeStep())
                << " MSE2Collector::notifyEnter::"
                << " lane " << myLane->getID()
                << " passedVeh " << myPassedVeh ;
                WRITE_MESSAGE(str.str());
            )
            return true;
        }
    }
    if (veh.getPositionOnLane() - veh.getVehicleType().getLength() > myEndPos) {
        // vehicle is beyond detector
        return false;
    }
    // vehicle is in front of the detector
    return true;
}


void
MSE2Collector::reset() {
    mySpeedSum = 0;
    myStartedHalts = 0;
    myJamLengthInMetersSum = 0;
    myJamLengthInVehiclesSum = 0;
    myVehicleSamples = 0;
    myOccupancySum = 0;
    myMaxOccupancy = 0;
    myMeanMaxJamInVehicles = 0;
    myMeanMaxJamInMeters = 0;
    myMaxJamInVehicles = 0;
    myMaxJamInMeters = 0;
    myTimeSamples = 0;
    myMeanVehicleNumber = 0;
    myMaxVehicleNumber = 0;
    for (std::map<const SUMOVehicle*, SUMOTime>::iterator i = myIntervalHaltingVehicleDurations.begin(); i != myIntervalHaltingVehicleDurations.end(); ++i) {
        (*i).second = 0;
    }
    myPastStandingDurations.clear();
    myPastIntervalStandingDurations.clear();
    myPassedVeh = 0;
}



void
MSE2Collector::detectorUpdate(const SUMOTime /* step */) {
    JamInfo* currentJam = 0;
    std::map<const SUMOVehicle*, SUMOTime> haltingVehicles;
    std::map<const SUMOVehicle*, SUMOTime> intervalHaltingVehicles;
    std::vector<JamInfo*> jams;

    SUMOReal lengthSum = 0;
    myCurrentMeanSpeed = 0;
    myCurrentMeanLength = 0;
    myCurrentStartedHalts = 0;
    myCurrentHaltingsNumber = 0;

    // go through the (sorted) list of vehicles positioned on the detector
    //  sum up values and prepare the list of jams
    myKnownVehicles.sort(by_vehicle_position_sorter(getLane()));
    for (std::list<SUMOVehicle*>::const_iterator i = myKnownVehicles.begin(); i != myKnownVehicles.end(); ++i) {
        const MSVehicle* const veh = static_cast<MSVehicle*>(*i);

        SUMOReal length = veh->getVehicleType().getLength();
        if (veh->getLane() == getLane()) {
            if (veh->getBackPositionOnLane(myLane) < myStartPos) {
                // vehicle entered detector partially
                length -= (veh->getVehicleType().getLength() - (veh->getPositionOnLane() - myStartPos));
            }
            if (veh->getPositionOnLane() > myEndPos && veh->getPositionOnLane() - veh->getVehicleType().getLength() <= myEndPos) {
                // vehicle left detector partially
                length -= (veh->getPositionOnLane() - myEndPos);
            }
        } else {
            // ok, the vehicle is only partially still on the detector, has already moved to the
            //  next lane; still, we do not know how far away it is
            length = myEndPos - veh->getBackPositionOnLane(myLane);
        }
        assert(length >= 0);

        mySpeedSum += veh->getSpeed();
        myCurrentMeanSpeed += veh->getSpeed();
        lengthSum += length;
        myCurrentMeanLength += length;

        // jam-checking begins
        bool isInJam = false;
        // first, check whether the vehicle is slow enough to be states as halting
        if (veh->getSpeed() < myJamHaltingSpeedThreshold) {
            myCurrentHaltingsNumber++;
            // we have to track the time it was halting;
            //  so let's look up whether it was halting before and compute the overall halting time
            bool wasHalting = myHaltingVehicleDurations.find(veh) != myHaltingVehicleDurations.end();
            if (wasHalting) {
                haltingVehicles[veh] = myHaltingVehicleDurations[veh] + DELTA_T;
                intervalHaltingVehicles[veh] = myIntervalHaltingVehicleDurations[veh] + DELTA_T;
            } else {
                haltingVehicles[veh] = DELTA_T;
                intervalHaltingVehicles[veh] = DELTA_T;
                myCurrentStartedHalts++;
                myStartedHalts++;
            }
            // we now check whether the halting time is large enough
            if (haltingVehicles[veh] > myJamHaltingTimeThreshold) {
                // yep --> the vehicle is a part of a jam
                isInJam = true;
            }
        } else {
            // is not standing anymore; keep duration information
            std::map<const SUMOVehicle*, SUMOTime>::iterator v = myHaltingVehicleDurations.find(veh);
            if (v != myHaltingVehicleDurations.end()) {
                myPastStandingDurations.push_back((*v).second);
                myHaltingVehicleDurations.erase(v);
            }
            v = myIntervalHaltingVehicleDurations.find(veh);
            if (v != myIntervalHaltingVehicleDurations.end()) {
                myPastIntervalStandingDurations.push_back((*v).second);
                myIntervalHaltingVehicleDurations.erase(v);
            }
        }

        // jam-building
        if (isInJam) {
            // the vehicle is in a jam;
            //  it may be a new one or already an existing one
            if (currentJam == 0) {
                // the vehicle is the first vehicle in a jam
                currentJam = new JamInfo();
                currentJam->firstStandingVehicle = i;
            } else {
                // ok, we have a jam already. But - maybe it is too far away
                //  ... honestly, I can hardly find a reason for doing this,
                //  but jams were defined this way in an earlier version...
                if (veh->getPositionOnLane() - (*currentJam->lastStandingVehicle)->getPositionOnLane() > myJamDistanceThreshold) {
                    // yep, yep, yep - it's a new one...
                    //  close the frist, build a new
                    jams.push_back(currentJam);
                    currentJam = new JamInfo;
                    currentJam->firstStandingVehicle = i;
                }
            }
            currentJam->lastStandingVehicle = i;
        } else {
            // the vehicle is not part of a jam...
            //  maybe we have to close an already computed jam
            if (currentJam != 0) {
                jams.push_back(currentJam);
                currentJam = 0;
            }
        }
    }
    if (currentJam != 0) {
        jams.push_back(currentJam);
        currentJam = 0;
    }

    myCurrentMaxJamLengthInMeters = 0;
    myCurrentMaxJamLengthInVehicles = 0;
    myCurrentJamLengthInMeters = 0;
    myCurrentJamLengthInVehicles = 0;
    // process jam information
    for (std::vector<JamInfo*>::iterator i = jams.begin(); i != jams.end(); ++i) {
        // compute current jam's values
        SUMOReal jamLengthInMeters =
            (*(*i)->firstStandingVehicle)->getPositionOnLane()
            - (*(*i)->lastStandingVehicle)->getPositionOnLane()
            + (*(*i)->lastStandingVehicle)->getVehicleType().getLengthWithGap();
        if ((*(*i)->firstStandingVehicle)->getLane() != myLane) {
            // vehicle is partial occupator, discount the length that is not on
            // this lane
            jamLengthInMeters -= ((*(*i)->firstStandingVehicle)->getVehicleType().getLengthWithGap() -
                                  (myLane->getLength() - (*(*i)->firstStandingVehicle)->getBackPositionOnLane(myLane)));
        }
        int jamLengthInVehicles = (int)distance((*i)->firstStandingVehicle, (*i)->lastStandingVehicle) + 1;
        // apply them to the statistics
        myCurrentMaxJamLengthInMeters = MAX2(myCurrentMaxJamLengthInMeters, jamLengthInMeters);
        myCurrentMaxJamLengthInVehicles = MAX2(myCurrentMaxJamLengthInVehicles, jamLengthInVehicles);
        myJamLengthInMetersSum += jamLengthInMeters;
        myJamLengthInVehiclesSum += jamLengthInVehicles;
        myCurrentJamLengthInMeters += jamLengthInMeters;
        myCurrentJamLengthInVehicles += jamLengthInVehicles;
    }
    myCurrentJamNo = (int) jams.size();

    const int numVehicles = (int) myKnownVehicles.size();
    myVehicleSamples += numVehicles;
    myTimeSamples += 1;
    // compute occupancy values
    SUMOReal currentOccupancy = lengthSum / (myEndPos - myStartPos) * (SUMOReal) 100.;
    myCurrentOccupancy = currentOccupancy;
    myOccupancySum += currentOccupancy;
    myMaxOccupancy = MAX2(myMaxOccupancy, currentOccupancy);
    // compute jam values
    myMeanMaxJamInVehicles += myCurrentMaxJamLengthInVehicles;
    myMeanMaxJamInMeters += myCurrentMaxJamLengthInMeters;
    myMaxJamInVehicles = MAX2(myMaxJamInVehicles, myCurrentMaxJamLengthInVehicles);
    myMaxJamInMeters = MAX2(myMaxJamInMeters, myCurrentMaxJamLengthInMeters);
    // save information about halting vehicles
    myHaltingVehicleDurations = haltingVehicles;
    myIntervalHaltingVehicleDurations = intervalHaltingVehicles;
    // compute information about vehicle numbers
    myMeanVehicleNumber += numVehicles;
    myMaxVehicleNumber = MAX2(numVehicles, myMaxVehicleNumber);
    // norm current values
    myCurrentMeanSpeed = numVehicles != 0 ? myCurrentMeanSpeed / (SUMOReal) numVehicles : -1;
    myCurrentMeanLength = numVehicles != 0 ? myCurrentMeanLength / (SUMOReal) numVehicles : -1;

    // clean up
    for (std::vector<JamInfo*>::iterator i = jams.begin(); i != jams.end(); ++i) {
        delete *i;
    }
    jams.clear();
}



void
MSE2Collector::writeXMLOutput(OutputDevice& dev, SUMOTime startTime, SUMOTime stopTime) {
    dev << "   <interval begin=\"" << time2string(startTime) << "\" end=\"" << time2string(stopTime) << "\" " << "id=\"" << getID() << "\" ";

    const SUMOReal meanSpeed = myVehicleSamples != 0 ? mySpeedSum / (SUMOReal) myVehicleSamples : -1;
    const SUMOReal meanOccupancy = myTimeSamples != 0 ? myOccupancySum / (SUMOReal) myTimeSamples : 0;
    const SUMOReal meanJamLengthInMeters = myTimeSamples != 0 ? myMeanMaxJamInMeters / (SUMOReal) myTimeSamples : 0;
    const SUMOReal meanJamLengthInVehicles = myTimeSamples != 0 ? myMeanMaxJamInVehicles / (SUMOReal) myTimeSamples : 0;
    const SUMOReal meanVehicleNumber = myTimeSamples != 0 ? (SUMOReal) myMeanVehicleNumber / (SUMOReal) myTimeSamples : 0;

    SUMOTime haltingDurationSum = 0;
    SUMOTime maxHaltingDuration = 0;
    int haltingNo = 0;
    for (std::vector<SUMOTime>::iterator i = myPastStandingDurations.begin(); i != myPastStandingDurations.end(); ++i) {
        haltingDurationSum += (*i);
        maxHaltingDuration = MAX2(maxHaltingDuration, (*i));
        haltingNo++;
    }
    for (std::map<const SUMOVehicle*, SUMOTime> ::iterator i = myHaltingVehicleDurations.begin(); i != myHaltingVehicleDurations.end(); ++i) {
        haltingDurationSum += (*i).second;
        maxHaltingDuration = MAX2(maxHaltingDuration, (*i).second);
        haltingNo++;
    }
    const SUMOTime meanHaltingDuration = haltingNo != 0 ? haltingDurationSum / haltingNo : 0;

    SUMOTime intervalHaltingDurationSum = 0;
    SUMOTime intervalMaxHaltingDuration = 0;
    int intervalHaltingNo = 0;
    for (std::vector<SUMOTime>::iterator i = myPastIntervalStandingDurations.begin(); i != myPastIntervalStandingDurations.end(); ++i) {
        intervalHaltingDurationSum += (*i);
        intervalMaxHaltingDuration = MAX2(intervalMaxHaltingDuration, (*i));
        intervalHaltingNo++;
    }
    for (std::map<const SUMOVehicle*, SUMOTime> ::iterator i = myIntervalHaltingVehicleDurations.begin(); i != myIntervalHaltingVehicleDurations.end(); ++i) {
        intervalHaltingDurationSum += (*i).second;
        intervalMaxHaltingDuration = MAX2(intervalMaxHaltingDuration, (*i).second);
        intervalHaltingNo++;
    }
    const SUMOTime intervalMeanHaltingDuration = intervalHaltingNo != 0 ? intervalHaltingDurationSum / intervalHaltingNo : 0;

    dev << "nSamples=\"" << myVehicleSamples << "\" "
        << "meanSpeed=\"" << meanSpeed << "\" "
        << "meanOccupancy=\"" << meanOccupancy << "\" "
        << "maxOccupancy=\"" << myMaxOccupancy << "\" "
        << "meanMaxJamLengthInVehicles=\"" << meanJamLengthInVehicles << "\" "
        << "meanMaxJamLengthInMeters=\"" << meanJamLengthInMeters << "\" "
        << "maxJamLengthInVehicles=\"" << myMaxJamInVehicles << "\" "
        << "maxJamLengthInMeters=\"" << myMaxJamInMeters << "\" "
        << "jamLengthInVehiclesSum=\"" << myJamLengthInVehiclesSum << "\" "
        << "jamLengthInMetersSum=\"" << myJamLengthInMetersSum << "\" "
        << "meanHaltingDuration=\"" << STEPS2TIME(meanHaltingDuration) << "\" "
        << "maxHaltingDuration=\"" << STEPS2TIME(maxHaltingDuration) << "\" "
        << "haltingDurationSum=\"" << STEPS2TIME(haltingDurationSum) << "\" "
        << "meanIntervalHaltingDuration=\"" << STEPS2TIME(intervalMeanHaltingDuration) << "\" "
        << "maxIntervalHaltingDuration=\"" << STEPS2TIME(intervalMaxHaltingDuration) << "\" "
        << "intervalHaltingDurationSum=\"" << STEPS2TIME(intervalHaltingDurationSum) << "\" "
        << "startedHalts=\"" << myStartedHalts << "\" "
        << "meanVehicleNumber=\"" << meanVehicleNumber << "\" "
        << "maxVehicleNumber=\"" << myMaxVehicleNumber << "\" "
        << "/>\n";
    reset();
}


void
MSE2Collector::writeXMLDetectorProlog(OutputDevice& dev) const {
    dev.writeXMLHeader("detector", "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.dlr.de/xsd/det_e2_file.xsd\"");
}


int
MSE2Collector::getCurrentVehicleNumber() const {
    return (int) myKnownVehicles.size();
}

int
MSE2Collector::getEstimatedCurrentVehicleNumber(SUMOReal speedThreshold) const {

    SUMOReal distance = 0;
    SUMOReal thresholdSpeed = myLane->getSpeedLimit() / speedThreshold;

    int count = 0;
    for (std::list<SUMOVehicle*>::const_iterator it = myKnownVehicles.begin();
            it != myKnownVehicles.end(); it++) {
        MSVehicle* veh = static_cast<MSVehicle*>(*it);
        SUMOReal acceleration = veh->getAcceleration();
        if (distance == 0) {
            distance = veh->getPositionOnLane();
        }
        if (veh->getPositionOnLane() < distance) {
            distance = veh->getPositionOnLane();
        }
        SUMOReal carLength = veh->getVehicleType().getLengthWithGap();
        SUMOReal vel = veh->getSpeed();
        SUMOReal realDistance = myLane->getLength() - distance; // the closer vehicle get to the light the greater is the distance
        if (vel <= thresholdSpeed || acceleration > 0) { //TODO speed less than half of the maximum speed for the lane NEED TUNING
            count = (int)(realDistance / carLength) + 1;
        }
    }

    return count;
}

SUMOReal
MSE2Collector::getEstimateQueueLength() const {

    if (myKnownVehicles.empty()) {
        return -1;
    }

    SUMOReal distance = 0;
    SUMOReal realDistance = 0;
    bool flowing =  true;
    for (std::list<SUMOVehicle*>::const_iterator it = myKnownVehicles.begin();
            it != myKnownVehicles.end(); it++) {
        MSVehicle* veh = static_cast<MSVehicle*>(*it);
        if (distance == 0) {
            distance = veh->getPositionOnLane();
        }
        if (veh->getPositionOnLane() < distance) {
            distance = veh->getPositionOnLane();
        }
        SUMOReal carLength = veh->getVehicleType().getLengthWithGap();
        SUMOReal vel = veh->getSpeed();
        //	SUMOReal distanceTemp = myLane->getLength() - distance;
        if (vel <= 0.5) {
            realDistance = distance - carLength;
            flowing = false;
        }
        DBG(
            std::ostringstream str;
            str << time2string(MSNet::getInstance()->getCurrentTimeStep())
            << " MSE2Collector::getEstimateQueueLength::"
            << " lane " << myLane->getID()
            << " vehicle " << veh->getID()
            << " positionOnLane " << veh->getPositionOnLane()
            << " vel " << veh->getSpeed()
            << " realDistance " << realDistance;
            WRITE_MESSAGE(str.str());
        )
    }
    if (flowing) {
        return 0;
    } else {
        return myLane->getLength() - realDistance;
    }
}


SUMOReal
MSE2Collector::getCurrentOccupancy() const {
    return myCurrentOccupancy;
}


SUMOReal
MSE2Collector::getCurrentMeanSpeed() const {
    return myCurrentMeanSpeed;
}


SUMOReal
MSE2Collector::getCurrentMeanLength() const {
    return myCurrentMeanLength;
}


int
MSE2Collector::getCurrentJamNumber() const {
    return myCurrentJamNo;
}


int
MSE2Collector::getCurrentMaxJamLengthInVehicles() const {
    return myCurrentMaxJamLengthInVehicles;
}


SUMOReal
MSE2Collector::getCurrentMaxJamLengthInMeters() const {
    return myCurrentMaxJamLengthInMeters;
}


int
MSE2Collector::getCurrentJamLengthInVehicles() const {
    return myCurrentJamLengthInVehicles;
}


SUMOReal
MSE2Collector::getCurrentJamLengthInMeters() const {
    return myCurrentJamLengthInMeters;
}


int
MSE2Collector::getCurrentStartedHalts() const {
    return myCurrentStartedHalts;
}


int
MSE2Collector::by_vehicle_position_sorter::operator()(const SUMOVehicle* v1, const SUMOVehicle* v2) {
    if (!v1->isFrontOnLane(myLane)) {
        return true;
    }
    if (!v2->isFrontOnLane(myLane)) {
        return false;
    }
    return v1->getPositionOnLane() > v2->getPositionOnLane();
}

int
MSE2Collector::getCurrentHaltingNumber() const {
    return myCurrentHaltingsNumber;
}


std::vector<std::string>
MSE2Collector::getCurrentVehicleIDs() const {
    std::vector<std::string> ret;
    for (std::list<SUMOVehicle*>::const_iterator i = myKnownVehicles.begin(); i != myKnownVehicles.end(); ++i) {
        MSVehicle* veh = static_cast<MSVehicle*>(*i);
        ret.push_back(veh->getID());
    }
    std::sort(ret.begin(), ret.end());
    return ret;
}

const std::list<SUMOVehicle*>& MSE2Collector::getCurrentVehicles() const {
    return myKnownVehicles;
}

/****************************************************************************/

