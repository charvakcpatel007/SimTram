/****************************************************************************/
/// @file    DijkstraRouterTT.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 25 July 2005
/// @version $Id: DijkstraRouterTT.h 21182 2016-07-18 06:46:01Z behrisch $
///
// Dijkstra shortest path algorithm using travel time
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
#ifndef DijkstraRouterTT_h
#define DijkstraRouterTT_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <cassert>
#include <string>
#include <functional>
#include <vector>
#include <deque>
#include <set>
#include <limits>
#include <algorithm>
#include <iterator>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StdDefs.h>
#include "SUMOAbstractRouter.h"

//#define DijkstraRouterTT_DEBUG_QUERY
//#define DijkstraRouterTT_DEBUG_QUERY_PERF

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class DijkstraRouterTT
 * @brief Computes the shortest path through a network using the Dijkstra algorithm.
 *
 * The template parameters are:
 * @param E The edge class to use (MSEdge/ROEdge)
 * @param V The vehicle class to use (MSVehicle/ROVehicle)
 * @param PF The prohibition function to use (prohibited_withPermissions/noProhibitions)
 * @param EC The class to retrieve the effort for an edge from
 *
 * The router is edge-based. It must know the number of edges for internal reasons
 *  and whether a missing connection between two given edges (unbuild route) shall
 *  be reported as an error or as a warning.
 *
 */
template<class E, class V, class PF>
class DijkstraRouterTT : public SUMOAbstractRouter<E, V>, public PF {

public:
    typedef SUMOReal(* Operation)(const E* const, const V* const, SUMOReal);

    /**
     * @struct EdgeInfo
     * A definition about a route's edge with the effort needed to reach it and
     *  the information about the previous edge.
     */
    class EdgeInfo {
    public:
        /// Constructor
        EdgeInfo(const E* e)
            : edge(e), traveltime(std::numeric_limits<SUMOReal>::max()), prev(0), visited(false) {}

        /// The current edge
        const E* edge;

        /// Effort to reach the edge
        SUMOReal traveltime;

        /// The previous edge
        EdgeInfo* prev;

        /// The previous edge
        bool visited;

        inline void reset() {
            traveltime = std::numeric_limits<SUMOReal>::max();
            visited = false;
        }
    };

    /**
     * @class EdgeInfoByEffortComparator
     * Class to compare (and so sort) nodes by their effort
     */
    class EdgeInfoByTTComparator {
    public:
        /// Comparing method
        bool operator()(const EdgeInfo* nod1, const EdgeInfo* nod2) const {
            if (nod1->traveltime == nod2->traveltime) {
                return nod1->edge->getNumericalID() > nod2->edge->getNumericalID();
            }
            return nod1->traveltime > nod2->traveltime;
        }
    };

    /// Constructor
    DijkstraRouterTT(const std::vector<E*>& edges, bool unbuildIsWarning, Operation operation) :
        SUMOAbstractRouter<E, V>(operation, "DijkstraRouterTT"),
        myErrorMsgHandler(unbuildIsWarning ?  MsgHandler::getWarningInstance() : MsgHandler::getErrorInstance()) {
        for (typename std::vector<E*>::const_iterator i = edges.begin(); i != edges.end(); ++i) {
            myEdgeInfos.push_back(EdgeInfo(*i));
        }
    }

    DijkstraRouterTT(const std::vector<EdgeInfo>& edgeInfos, bool unbuildIsWarning, Operation operation) :
        SUMOAbstractRouter<E, V>(operation, "DijkstraRouterTT"),
        myErrorMsgHandler(unbuildIsWarning ?  MsgHandler::getWarningInstance() : MsgHandler::getErrorInstance()) {
        for (typename std::vector<EdgeInfo>::const_iterator i = edgeInfos.begin(); i != edgeInfos.end(); ++i) {
            myEdgeInfos.push_back(*i);
        }
    }

    /// Destructor
    virtual ~DijkstraRouterTT() { }

    virtual SUMOAbstractRouter<E, V>* clone() {
        return new DijkstraRouterTT<E, V, PF>(myEdgeInfos, myErrorMsgHandler == MsgHandler::getWarningInstance(), this->myOperation);
    }

    void init() {
        // all EdgeInfos touched in the previous query are either in myFrontierList or myFound: clean those up
        for (typename std::vector<EdgeInfo*>::iterator i = myFrontierList.begin(); i != myFrontierList.end(); i++) {
            (*i)->reset();
        }
        myFrontierList.clear();
        for (typename std::vector<EdgeInfo*>::iterator i = myFound.begin(); i != myFound.end(); i++) {
            (*i)->reset();
        }
        myFound.clear();
    }


    /** @brief Builds the route between the given edges using the minimum effort at the given time
        The definition of the effort depends on the wished routing scheme */
    virtual bool compute(const E* from, const E* to, const V* const vehicle,
                         SUMOTime msTime, std::vector<const E*>& into) {
        assert(from != 0 && (vehicle == 0 || to != 0));
        // check whether from and to can be used
        if (PF::operator()(from, vehicle)) {
            myErrorMsgHandler->inform("Vehicle  '" + vehicle->getID() + "' is not allowed on from edge '" + from->getID() + "'.");
            return false;
        }
        if (PF::operator()(to, vehicle)) {
            myErrorMsgHandler->inform("Vehicle  '" + vehicle->getID() + "' is not allowed on to edge '" + to->getID() + "'.");
            return false;
        }
        this->startQuery();
        const SUMOVehicleClass vClass = vehicle == 0 ? SVC_IGNORING : vehicle->getVClass();
        const SUMOReal time = STEPS2TIME(msTime);
        if (this->myBulkMode) {
            const EdgeInfo& toInfo = myEdgeInfos[to->getNumericalID()];
            if (toInfo.visited) {
                buildPathFrom(&toInfo, into);
                this->endQuery(1);
                return true;
            }
        } else {
            init();
            // add begin node
            EdgeInfo* const fromInfo = &(myEdgeInfos[from->getNumericalID()]);
            fromInfo->traveltime = 0;
            fromInfo->prev = 0;
            myFrontierList.push_back(fromInfo);
        }
        // loop
        int num_visited = 0;
        while (!myFrontierList.empty()) {
            num_visited += 1;
            // use the node with the minimal length
            EdgeInfo* const minimumInfo = myFrontierList.front();
            const E* const minEdge = minimumInfo->edge;
            // check whether the destination node was already reached
            if (minEdge == to) {
                buildPathFrom(minimumInfo, into);
                this->endQuery(num_visited);
#ifdef DijkstraRouterTT_DEBUG_QUERY_PERF
                std::cout << "visited " + toString(num_visited) + " edges (final path length: " + toString(into.size()) + ")\n";
#endif
                return true;
            }
            pop_heap(myFrontierList.begin(), myFrontierList.end(), myComparator);
            myFrontierList.pop_back();
            myFound.push_back(minimumInfo);
            minimumInfo->visited = true;
#ifdef DijkstraRouterTT_DEBUG_QUERY
            std::cout << "DEBUG: hit '" << minEdge->getID() << "' TT: " << minimumInfo->traveltime << " Q: ";
            for (typename std::vector<EdgeInfo*>::iterator it = myFrontierList.begin(); it != myFrontierList.end(); it++) {
                std::cout << (*it)->traveltime << "," << (*it)->edge->getID() << " ";
            }
            std::cout << "\n";
#endif
            const SUMOReal traveltime = minimumInfo->traveltime + this->getEffort(minEdge, vehicle, time + minimumInfo->traveltime);
            // check all ways from the node with the minimal length
            const std::vector<E*>& successors = minEdge->getSuccessors(vClass);
            for (typename std::vector<E*>::const_iterator it = successors.begin(); it != successors.end(); ++it) {
                const E* const follower = *it;
                EdgeInfo* const followerInfo = &(myEdgeInfos[follower->getNumericalID()]);
                // check whether it can be used
                if (PF::operator()(follower, vehicle)) {
                    continue;
                }
                const SUMOReal oldEffort = followerInfo->traveltime;
                if (!followerInfo->visited && traveltime < oldEffort) {
                    followerInfo->traveltime = traveltime;
                    followerInfo->prev = minimumInfo;
                    if (oldEffort == std::numeric_limits<SUMOReal>::max()) {
                        myFrontierList.push_back(followerInfo);
                        push_heap(myFrontierList.begin(), myFrontierList.end(), myComparator);
                    } else {
                        push_heap(myFrontierList.begin(),
                                  find(myFrontierList.begin(), myFrontierList.end(), followerInfo) + 1,
                                  myComparator);
                    }
                }
            }
        }
        this->endQuery(num_visited);
#ifdef DijkstraRouterTT_DEBUG_QUERY_PERF
        std::cout << "visited " + toString(num_visited) + " edges (final path length: " + toString(into.size()) + ")\n";
#endif
        if (to != 0) {
            myErrorMsgHandler->inform("No connection between edge '" + from->getID() + "' and edge '" + to->getID() + "' found.");
        }
        return false;
    }


    SUMOReal recomputeCosts(const std::vector<const E*>& edges, const V* const v, SUMOTime msTime) const {
        const SUMOReal time = STEPS2TIME(msTime);
        SUMOReal costs = 0;
        for (typename std::vector<const E*>::const_iterator i = edges.begin(); i != edges.end(); ++i) {
            if (PF::operator()(*i, v)) {
                return -1;
            }
            costs += this->getEffort(*i, v, time + costs);
        }
        return costs;
    }

public:
    /// Builds the path from marked edges
    void buildPathFrom(const EdgeInfo* rbegin, std::vector<const E*>& edges) {
        std::vector<const E*> tmp;
        while (rbegin != 0) {
            tmp.push_back(rbegin->edge);
            rbegin = rbegin->prev;
        }
        std::copy(tmp.rbegin(), tmp.rend(), std::back_inserter(edges));
    }

    const EdgeInfo& getEdgeInfo(int index) const {
        return myEdgeInfos[index];
    }

private:
    /// The container of edge information
    std::vector<EdgeInfo> myEdgeInfos;

    /// A container for reusage of the min edge heap
    std::vector<EdgeInfo*> myFrontierList;
    /// @brief list of visited Edges (for resetting)
    std::vector<EdgeInfo*> myFound;

    EdgeInfoByTTComparator myComparator;

    /// @brief the handler for routing errors
    MsgHandler* const myErrorMsgHandler;
};


#endif

/****************************************************************************/

