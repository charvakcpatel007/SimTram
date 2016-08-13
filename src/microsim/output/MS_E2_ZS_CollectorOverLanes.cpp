/****************************************************************************/
/// @file    MS_E2_ZS_CollectorOverLanes.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Oct 2003
/// @version $Id: MS_E2_ZS_CollectorOverLanes.cpp 21182 2016-07-18 06:46:01Z behrisch $
///
// A detector which joins E2Collectors over consecutive lanes (backward)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2003-2016 DLR (http://www.dlr.de/) and contributors
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
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include "MS_E2_ZS_CollectorOverLanes.h"
#include "MSE2Collector.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
MS_E2_ZS_CollectorOverLanes::MS_E2_ZS_CollectorOverLanes(const std::string& id,
        DetectorUsage usage,
        MSLane* lane,
        SUMOReal startPos,
        SUMOTime haltingTimeThreshold,
        SUMOReal haltingSpeedThreshold,
        SUMOReal jamDistThreshold)
    : MSDetectorFileOutput(id),
      startPosM(startPos), haltingTimeThresholdM(haltingTimeThreshold),
      haltingSpeedThresholdM(haltingSpeedThreshold), jamDistThresholdM(jamDistThreshold),
      myStartLaneID(lane->getID()), myUsage(usage) {}


void
MS_E2_ZS_CollectorOverLanes::init(MSLane* lane, SUMOReal detLength) {
    myLength = detLength;
    if (startPosM == 0) {
        startPosM = (SUMOReal) 0.1;
    }
    SUMOReal length = lane->getLength() - startPosM - (SUMOReal) 0.1;
    SUMOReal dlength = detLength;
    if (length > dlength) {
        length = dlength;
    }
    myLengths.push_back(length);
    myLaneCombinations.push_back(LaneVector());
    myLaneCombinations[0].push_back(lane);
    myDetectorCombinations.push_back(DetectorVector());
    MSE2Collector* c =
        buildCollector(0, 0, lane, startPosM, length);
    myDetectorCombinations[0].push_back(c);
    myAlreadyBuild[lane] = c;
    extendTo(detLength);
}


MS_E2_ZS_CollectorOverLanes::~MS_E2_ZS_CollectorOverLanes() {}


void
MS_E2_ZS_CollectorOverLanes::extendTo(SUMOReal length) {
    bool done = false;
    while (!done) {
        done = true;
        LengthVector::iterator leni = myLengths.begin();
        LaneVectorVector::iterator lanei = myLaneCombinations.begin();
        DetectorVectorVector::iterator deti = myDetectorCombinations.begin();
        for (; leni != myLengths.end(); leni++, lanei++, deti++) {
            if ((*leni) < length) {
                done = false;
                // copy current values
                LaneVector lv = *lanei;
                DetectorVector dv = *deti;
                SUMOReal clength = *leni;
                assert(lv.size() > 0);
                assert(dv.size() > 0);
                // erase previous elements
                assert(leni != myLengths.end());
                myLengths.erase(leni);
                myLaneCombinations.erase(lanei);
                myDetectorCombinations.erase(deti);
                // get the lane to look before
                MSLane* toExtend = lv.back();
                // and her predecessors
                std::vector<MSLane*> predeccessors = getLanePredeccessorLanes(toExtend);
                if (predeccessors.size() == 0) {
                    int off = 1;
                    MSEdge& e = toExtend->getEdge();
                    const std::vector<MSLane*>& lanes = e.getLanes();
                    int idx = (int) distance(lanes.begin(), find(lanes.begin(), lanes.end(), toExtend));
                    while (predeccessors.size() == 0) {
                        if (idx - off >= 0) {
                            MSLane* tryMe = lanes[idx - off];
                            predeccessors = getLanePredeccessorLanes(tryMe);
                        }
                        if (predeccessors.size() == 0 && idx + off < (int) lanes.size()) {
                            MSLane* tryMe = lanes[idx + off];
                            predeccessors = getLanePredeccessorLanes(tryMe);
                        }
                        off++;
                    }
                }

                /*                LaneContinuations::const_iterator conts =
                                    laneContinuations.find(toExtend->id());
                                assert(conts!=laneContinuations.end());
                                const std::vector<std::string> &predeccessors =
                                    (*conts).second;*/
                // go through the predeccessors and extend the detector
                for (std::vector<MSLane*>::const_iterator i = predeccessors.begin(); i != predeccessors.end(); i++) {
                    // get the lane
                    MSLane* l = *i;
                    // compute detector length
                    SUMOReal lanelen = length - clength;
                    if (lanelen > l->getLength()) {
                        lanelen = l->getLength() - (SUMOReal) 0.2;
                    }
                    // build new info
                    LaneVector nlv = lv;
                    nlv.push_back(l);
                    DetectorVector ndv = dv;
                    MSE2Collector* coll = 0;
                    if (myAlreadyBuild.find(l) == myAlreadyBuild.end()) {
                        coll = buildCollector(0, 0, l, (SUMOReal) 0.1, lanelen);
                    } else {
                        coll = myAlreadyBuild.find(l)->second;
                    }
                    myAlreadyBuild[l] = coll;
                    ndv.push_back(coll);
                    // store new info
                    myLaneCombinations.push_back(nlv);
                    myDetectorCombinations.push_back(ndv);
                    myLengths.push_back(clength + lanelen);
                }
                // restart
                leni = myLengths.end() - 1;
            }
        }
    }
}


std::vector<MSLane*>
MS_E2_ZS_CollectorOverLanes::getLanePredeccessorLanes(MSLane* l) {
    std::string eid = l->getEdge().getID();
    // get predecessing edges
    const MSEdgeVector& predEdges = l->getEdge().getIncomingEdges();
    std::vector<MSLane*> ret;
    // find predecessing lanes
    MSEdgeVector::const_iterator i = predEdges.begin();
    for (; i != predEdges.end(); ++i) {
        MSEdge* e = *i;
        assert(e != 0);
        typedef std::vector<MSLane*> LaneVector;
        const LaneVector* cl = e->allowedLanes(l->getEdge(), SVC_IGNORING);
        bool fastAbort = false;
        if (cl != 0) {
            for (LaneVector::const_iterator j = cl->begin(); !fastAbort && j != cl->end(); j++) {
                const MSLinkCont& lc = (*j)->getLinkCont();
                for (MSLinkCont::const_iterator k = lc.begin(); !fastAbort && k != lc.end(); k++) {
                    if ((*k)->getLane() == l) {
                        ret.push_back(*j);
                        fastAbort = true;
                    }
                }
            }
        }
    }
    return ret;
}


MSE2Collector*
MS_E2_ZS_CollectorOverLanes::buildCollector(int c, int r, MSLane* l,
        SUMOReal start, SUMOReal end) {
    std::string id = makeID(l->getID(), c, r);
    if (start + end < l->getLength()) {
        start = l->getLength() - end - (SUMOReal) 0.1;
    }
    return new MSE2Collector(id, myUsage,
                             l, start, end, haltingTimeThresholdM,
                             haltingSpeedThresholdM, jamDistThresholdM);
}


void
MS_E2_ZS_CollectorOverLanes::writeXMLOutput(OutputDevice& /*&dev*/,
        SUMOTime /*startTime*/, SUMOTime /*stopTime*/) {
    /*
    dev<<"<interval begin=\""<<time2string(startTime)<<"\" end=\""<<
    time2string(stopTime)<<"\" "<<"id=\""<<myID<<"\" ";
    if (hasDetector(E2::QUEUE_LENGTH_AHEAD_OF_TRAFFIC_LIGHTS_IN_VEHICLES)) {
        dev<<"collQueueLengthAheadOfTrafficLightsInVehiclesMax=\"";
        dev<<toString(getCurrent(E2::QUEUE_LENGTH_AHEAD_OF_TRAFFIC_LIGHTS_IN_VEHICLES));
        dev<<"\" ";
        resetQueueLengthAheadOfTrafficLights();
    }
    myDetectorCombinations[0][0]->writeXMLOutput(dev, startTime, stopTime);
    dev<<"/>\n";
    */
}


void
MS_E2_ZS_CollectorOverLanes::writeXMLDetectorProlog(OutputDevice& dev) const {
    dev.writeXMLHeader("detector");
}


int bla = 0;

std::string
MS_E2_ZS_CollectorOverLanes::makeID(const std::string& baseID ,
                                    int /*col*/, int /*row*/) const {
    std::string add;
    switch (myUsage) {
        case DU_USER_DEFINED:
            add = "(u)";
            break;
        case DU_SUMO_INTERNAL:
            add = "(i)";
            break;
        case DU_TL_CONTROL:
            add = "(c)";
            break;
        default:
            break;
    }
    std::string ret =  baseID + add + toString<int>(bla++);
    return ret;
}


const std::string&
MS_E2_ZS_CollectorOverLanes::getStartLaneID() const {
    return myStartLaneID;
}


/****************************************************************************/

