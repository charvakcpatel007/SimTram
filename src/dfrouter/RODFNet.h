/****************************************************************************/
/// @file    RODFNet.h
/// @author  Daniel Krajzewicz
/// @author  Eric Nicolay
/// @author  Michael Behrisch
/// @date    Thu, 16.03.2006
/// @version $Id: RODFNet.h 21182 2016-07-18 06:46:01Z behrisch $
///
// A DFROUTER-network
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
#ifndef RODFNet_h
#define RODFNet_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/options/OptionsCont.h>
#include <utils/common/StdDefs.h>
#include <utils/common/SUMOTime.h>
#include <router/ROEdge.h>
#include <router/RONet.h>
#include "RODFDetector.h"
#include "RODFRouteDesc.h"
#include "RODFRouteCont.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class RODFNet
 * @brief A DFROUTER-network
 */
class RODFNet : public RONet {
public:
    /** @brief Constructor
     * @param[in] amInHighwayMode Whether search for following edges shall stop at slow edges
     */
    RODFNet(bool amInHighwayMode);


    /// @brief Destructor
    ~RODFNet();


    void buildApproachList();

    void computeTypes(RODFDetectorCon& dets,
                      bool sourcesStrict) const;
    void buildRoutes(RODFDetectorCon& det, bool keepUnfoundEnds, bool includeInBetween,
                     bool keepShortestOnly, int maxFollowingLength) const;
    SUMOReal getAbsPos(const RODFDetector& det) const;

    void buildEdgeFlowMap(const RODFDetectorFlows& flows,
                          const RODFDetectorCon& detectors,
                          SUMOTime startTime, SUMOTime endTime, SUMOTime stepOffset);

    void revalidateFlows(const RODFDetectorCon& detectors,
                         RODFDetectorFlows& flows,
                         SUMOTime startTime, SUMOTime endTime, SUMOTime stepOffset);


    void removeEmptyDetectors(RODFDetectorCon& detectors,
                              RODFDetectorFlows& flows);

    void reportEmptyDetectors(RODFDetectorCon& detectors,
                              RODFDetectorFlows& flows);

    void buildDetectorDependencies(RODFDetectorCon& detectors);

    void mesoJoin(RODFDetectorCon& detectors, RODFDetectorFlows& flows);

    bool hasDetector(ROEdge* edge) const;
    const std::vector<std::string>& getDetectorList(ROEdge* edge) const;


protected:
    void revalidateFlows(const RODFDetector* detector,
                         RODFDetectorFlows& flows,
                         SUMOTime startTime, SUMOTime endTime, SUMOTime stepOffset);
    bool isSource(const RODFDetector& det,
                  const RODFDetectorCon& detectors, bool strict) const;
    bool isFalseSource(const RODFDetector& det,
                       const RODFDetectorCon& detectors) const;
    bool isDestination(const RODFDetector& det,
                       const RODFDetectorCon& detectors) const;

    ROEdge* getDetectorEdge(const RODFDetector& det) const;
    bool isSource(const RODFDetector& det, ROEdge* edge,
                  ROEdgeVector& seen, const RODFDetectorCon& detectors,
                  bool strict) const;
    bool isFalseSource(const RODFDetector& det, ROEdge* edge,
                       ROEdgeVector& seen, const RODFDetectorCon& detectors) const;
    bool isDestination(const RODFDetector& det, ROEdge* edge, ROEdgeVector& seen,
                       const RODFDetectorCon& detectors) const;

    void computeRoutesFor(ROEdge* edge, RODFRouteDesc& base, int no,
                          bool keepUnfoundEnds,
                          bool keepShortestOnly,
                          ROEdgeVector& visited, const RODFDetector& det,
                          RODFRouteCont& into, const RODFDetectorCon& detectors,
                          int maxFollowingLength,
                          ROEdgeVector& seen) const;

    void buildDetectorEdgeDependencies(RODFDetectorCon& dets) const;

    bool hasApproaching(ROEdge* edge) const;
    bool hasApproached(ROEdge* edge) const;

    bool hasInBetweenDetectorsOnly(ROEdge* edge,
                                   const RODFDetectorCon& detectors) const;
    bool hasSourceDetector(ROEdge* edge,
                           const RODFDetectorCon& detectors) const;

    struct IterationEdge {
        int depth;
        ROEdge* edge;
    };

protected:
    class DFRouteDescByTimeComperator {
    public:
        /// Constructor
        explicit DFRouteDescByTimeComperator() { }

        /// Destructor
        ~DFRouteDescByTimeComperator() { }

        /// Comparing method
        bool operator()(const RODFRouteDesc& nod1, const RODFRouteDesc& nod2) const {
            return nod1.duration_2 > nod2.duration_2;
        }
    };

private:
    /// @brief comparator for maps using edges as key, used only in myDetectorsOnEdges to make tests comparable
    struct idComp {
        bool operator()(ROEdge* const lhs, ROEdge* const rhs) const {
            return lhs->getID() < rhs->getID();
        }
    };

    /// @brief Map of edge name->list of names of this edge approaching edges
    std::map<ROEdge*, ROEdgeVector > myApproachingEdges;

    /// @brief Map of edge name->list of names of edges approached by this edge
    std::map<ROEdge*, ROEdgeVector > myApproachedEdges;

    mutable std::map<ROEdge*, std::vector<std::string>, idComp> myDetectorsOnEdges;
    mutable std::map<std::string, ROEdge*> myDetectorEdges;

    bool myAmInHighwayMode;
    mutable int mySourceNumber, mySinkNumber, myInBetweenNumber, myInvalidNumber;

    /// @brief List of ids of edges that shall not be used
    std::vector<std::string> myDisallowedEdges;


    bool myKeepTurnarounds;

};


#endif

/****************************************************************************/

