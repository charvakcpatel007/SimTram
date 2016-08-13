/****************************************************************************/
/// @file    NBContHelper.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 17 Dec 2001
/// @version $Id: NBContHelper.h 21131 2016-07-08 07:59:22Z behrisch $
///
// Some methods for traversing lists of edges
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
#ifndef NBContHelper_h
#define NBContHelper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include "NBHelpers.h"
#include "NBCont.h"
#include "NBEdge.h"
#include "NBNode.h"
#include <utils/common/StdDefs.h>
#include <utils/geom/GeomHelper.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * NBContHelper
 * Some static helper methods that traverse a sorted list of edges in both
 * directions
 */
class NBContHelper {
public:
    /** Moves the given iterator clockwise within the given container
        of edges sorted clockwise */
    static void nextCW(const EdgeVector& edges,
                       EdgeVector::const_iterator& from);

    /** Moves the given iterator counter clockwise within the given container
        of edges sorted clockwise */
    static void nextCCW(const EdgeVector& edges,
                        EdgeVector::const_iterator& from);

    static SUMOReal getMaxSpeed(const EdgeVector& edges);

    static SUMOReal getMinSpeed(const EdgeVector& edges);

    /** writes the vector of bools to the given stream */
    static std::ostream& out(std::ostream& os, const std::vector<bool>& v);


    /**
     * relative_outgoing_edge_sorter
     * Class to sort edges by their angle in relation to the node the
     * edge using this class is incoming into. This is normally done to
     * sort edges outgoing from the node the using edge is incoming in
     * by their angle in relation to the using edge's angle (this angle
     * is the reference angle).
     */
    class relative_outgoing_edge_sorter {
    public:
        /// constructor
        explicit relative_outgoing_edge_sorter(NBEdge* e) : myEdge(e) {}

    public:
        /// comparing operation
        int operator()(NBEdge* e1, NBEdge* e2) const;

    private:
        /// the edge to compute the relative angle of
        NBEdge* myEdge;
    };


    /**
     * straightness_sorter
     * Class to sort edges according to how straight thei are in relation to the
     * reference edge
     */
    class straightness_sorter {
    public:
        /// constructor
        explicit straightness_sorter(NBEdge* e) :
            myReferencePos(e->getLaneShape(0).back()),
            myReferenceAngle(e->getShapeEndAngle()) {
        }

    public:
        /// comparing operation
        int operator()(NBEdge* e1, NBEdge* e2) const;

    private:
        Position myReferencePos;
        SUMOReal myReferenceAngle;
    };


    /**
     * relative_incoming_edge_sorter
     * Class to sort edges by their angle in relation to an outgoing edge.
     * This is normally done to sort edges incoming at the starting node of this edge
     * by their angle in relation to the using edge's angle (this angle
     * is the reference angle).
     */
    class relative_incoming_edge_sorter {
    public:
        /// constructor
        explicit relative_incoming_edge_sorter(NBEdge* e) : myEdge(e) {}

    public:
        /// comparing operation
        int operator()(NBEdge* e1, NBEdge* e2) const;

    private:
        /// the edge to compute the relative angle of
        NBEdge* myEdge;
    };


    /**
     * edge_by_priority_sorter
     * Class to sort edges by their priority
     */
    class edge_by_priority_sorter {
    public:
        /// comparing operator
        int operator()(NBEdge* e1, NBEdge* e2) const {
            if (e1->getPriority() != e2->getPriority()) {
                return e1->getPriority() > e2->getPriority();
            }
            if (e1->getSpeed() != e2->getSpeed()) {
                return e1->getSpeed() > e2->getSpeed();
            }
            return e1->getNumLanes() > e2->getNumLanes();
        }
    };

    // ---------------------------

    /**
     * @class edge_opposite_direction_sorter
     * @brief Class to sort edges by their angle in relation to the given edge
     *
     * The resulting sorted list has the edge in the most opposite direction
     *  to the given edge as her first entry.
     */
    class edge_opposite_direction_sorter {
    public:
        /** @brief Constructor
         * @param[in] e The edge to which the sorting relates
         * @param[in] n The node to consider
         */
        explicit edge_opposite_direction_sorter(const NBEdge* const e, const NBNode* const n)
            : myNode(n) {
            myAngle = getEdgeAngleAt(e, n);
        }

        /** @brief Comparing operation
         * @param[in] e1 The first edge to compare
         * @param[in] e2 The second edge to compare
         * @return Which edge is more opposite to the related one
         */
        int operator()(NBEdge* e1, NBEdge* e2) const {
            return getDiff(e1) > getDiff(e2);
        }

    protected:
        /** @brief Computes the angle difference between the related and the given edge
         * @param[in] e The edge to compare the angle difference of
         * @return The angle difference
         */
        SUMOReal getDiff(const NBEdge* const e) const {
            return fabs(GeomHelper::angleDiff(getEdgeAngleAt(e, myNode), myAngle));
        }

        /** @brief Returns the given edge's angle at the given node
         *
         * Please note that we always consider the "outgoing direction".
         * @param[in] e The edge to which the sorting relates
         * @param[in] n The node to consider
         */
        SUMOReal getEdgeAngleAt(const NBEdge* const e, const NBNode* const n) const {
            if (e->getFromNode() == n) {
                return e->getGeometry().angleAt2D(0);
            } else {
                return e->getGeometry()[-1].angleTo2D(e->getGeometry()[-2]);
            }
        }

    private:
        /// @brief The angle of the related edge at the given node
        SUMOReal myAngle;

        /// @brief The related node
        const NBNode* const myNode;

    private:
        /// @brief Invalidated assignment operator
        edge_opposite_direction_sorter& operator=(const edge_opposite_direction_sorter& s);

    };

    // ---------------------------

    /**
     * edge_similar_direction_sorter
     * Class to sort edges by their angle in relation to the given edge
     * The resulting list should have the edge in the most similar direction
     * to the given edge as her first entry
     */
    class edge_similar_direction_sorter {
    public:
        /// constructor
        explicit edge_similar_direction_sorter(const NBEdge* const e)
            : myAngle(e->getTotalAngle()) {}

        /// comparing operation
        int operator()(NBEdge* e1, NBEdge* e2) const {
            SUMOReal d1 = GeomHelper::getMinAngleDiff(e1->getTotalAngle(), myAngle);
            SUMOReal d2 = GeomHelper::getMinAngleDiff(e2->getTotalAngle(), myAngle);
            return d1 < d2;
        }

    private:
        /// the angle to find the edge with the opposite direction
        SUMOReal myAngle;
    };


    /**
     * @class node_with_incoming_finder
     */
    class node_with_incoming_finder {
    public:
        /// constructor
        node_with_incoming_finder(const NBEdge* const e);

        bool operator()(const NBNode* const n) const;

    private:
        const NBEdge* const myEdge;

    private:
        /// @brief invalidated assignment operator
        node_with_incoming_finder& operator=(const node_with_incoming_finder& s);

    };


    /**
     * @class node_with_outgoing_finder
     */
    class node_with_outgoing_finder {
    public:
        /// constructor
        node_with_outgoing_finder(const NBEdge* const e);

        bool operator()(const NBNode* const n) const;

    private:
        const NBEdge* const myEdge;

    private:
        /// @brief invalidated assignment operator
        node_with_outgoing_finder& operator=(const node_with_outgoing_finder& s);

    };




    class edge_with_destination_finder {
    public:
        /// constructor
        edge_with_destination_finder(NBNode* dest);

        bool operator()(NBEdge* e) const;

    private:
        NBNode* myDestinationNode;

    private:
        /// @brief invalidated assignment operator
        edge_with_destination_finder& operator=(const edge_with_destination_finder& s);

    };


    /** Tries to return the first edge within the given container which
        connects both given nodes */
    static NBEdge* findConnectingEdge(const EdgeVector& edges,
                                      NBNode* from, NBNode* to);


    /** returns the maximum speed allowed on the edges */
    static SUMOReal maxSpeed(const EdgeVector& ev);

    /**
     * same_connection_edge_sorter
     * This class is used to sort edges which connect the same nodes.
     * The edges are sorted in dependence to edges connecting them. The
     * rightmost will be the first in the list; the leftmost the last one.
     */
    class same_connection_edge_sorter {
    public:
        /// constructor
        explicit same_connection_edge_sorter() { }

        /// comparing operation
        int operator()(NBEdge* e1, NBEdge* e2) const {
            std::pair<SUMOReal, SUMOReal> mm1 = getMinMaxRelAngles(e1);
            std::pair<SUMOReal, SUMOReal> mm2 = getMinMaxRelAngles(e2);
            if (mm1.first == mm2.first && mm1.second == mm2.second) {
                // ok, let's simply sort them arbitrarily
                return e1->getID() < e2->getID();
            }

            assert(
                (mm1.first <= mm2.first && mm1.second <= mm2.second)
                ||
                (mm1.first >= mm2.first && mm1.second >= mm2.second));
            return (mm1.first >= mm2.first && mm1.second >= mm2.second);
        }

        /**
         *
         */
        std::pair<SUMOReal, SUMOReal> getMinMaxRelAngles(NBEdge* e) const {
            SUMOReal min = 360;
            SUMOReal max = 360;
            const EdgeVector& ev = e->getConnectedEdges();
            for (EdgeVector::const_iterator i = ev.begin(); i != ev.end(); ++i) {
                SUMOReal angle = NBHelpers::normRelAngle(
                                     e->getTotalAngle(), (*i)->getTotalAngle());
                if (min == 360 || min > angle) {
                    min = angle;
                }
                if (max == 360 || max < angle) {
                    max = angle;
                }
            }
            return std::pair<SUMOReal, SUMOReal>(min, max);
        }
    };


    friend std::ostream& operator<<(std::ostream& os, const EdgeVector& ev);

    class opposite_finder {
    public:
        /// constructor
        opposite_finder(NBEdge* edge)
            : myReferenceEdge(edge) { }

        bool operator()(NBEdge* e) const {
            return e->isTurningDirectionAt(myReferenceEdge) ||
                   myReferenceEdge->isTurningDirectionAt(e);
        }

    private:
        NBEdge* myReferenceEdge;

    };

    /**
     * edge_by_angle_to_nodeShapeCentroid_sorter
     * Class to sort edges by their angle in relation to the node shape
     * */
    class edge_by_angle_to_nodeShapeCentroid_sorter {
    public:
        /// constructor
        explicit edge_by_angle_to_nodeShapeCentroid_sorter(const NBNode* n) : myNode(n) {}

    public:
        /// comparing operation
        int operator()(const NBEdge* e1, const NBEdge* e2) const;

    private:
        /// the edge to compute the relative angle of
        const NBNode* myNode;
    };

};


#endif

/****************************************************************************/

