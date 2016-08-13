/****************************************************************************/
/// @file    NGRandomNetBuilder.cpp
/// @author  Markus Hartinger
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mar, 2003
/// @version $Id: NGRandomNetBuilder.cpp 21202 2016-07-19 13:40:35Z behrisch $
///
// Additional structures for building random nets
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

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include "NGRandomNetBuilder.h"
#include <utils/geom/GeomHelper.h>
#include <utils/common/RandHelper.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// TNeighbourDistribution-definitions
// ---------------------------------------------------------------------------
void
TNeighbourDistribution::add(int NumNeighbours, SUMOReal ratio) {
    myNeighbours[NumNeighbours] = ratio;
}


int
TNeighbourDistribution::num() {
    SUMOReal sum = 0, RandValue;
    std::map<int, SUMOReal>::iterator i;
    // total sum of ratios
    for (i = myNeighbours.begin(); i != myNeighbours.end(); ++i) {
        sum += (*i).second;
    }
    // RandValue = [0,sum]
    RandValue = RandHelper::rand(sum);
    // find selected item
    i = myNeighbours.begin();
    sum = (*i).second;
    while ((i != myNeighbours.end()) && (sum < RandValue)) {
        ++i;
        sum += (*i).second;
    }
    return (*i).first;
}


// ---------------------------------------------------------------------------
// NGRandomNetBuilder-definitions
// ---------------------------------------------------------------------------
NGRandomNetBuilder::NGRandomNetBuilder(NGNet& net, SUMOReal minAngle, SUMOReal minDistance,
                                       SUMOReal maxDistance, SUMOReal connectivity,
                                       int numTries, const TNeighbourDistribution& neighborDist)
    : myNet(net), myMinLinkAngle(minAngle), myMinDistance(minDistance),
      myMaxDistance(maxDistance), myConnectivity(connectivity), myNumTries(numTries),
      myNeighbourDistribution(neighborDist) {
}


void
NGRandomNetBuilder::removeOuterNode(NGNode* node) {
    for (NGNodeList::iterator ni = myOuterNodes.begin(); ni != myOuterNodes.end(); ++ni) {
        if (*ni == node) {
            myOuterNodes.erase(ni);
            return;
        }
    }
}


bool
NGRandomNetBuilder::checkAngles(NGNode* node) {
    bool check = true;

    if (node->LinkList.size() >  1) {
        // loop over all links
        NGEdgeList::iterator li;
        NGNode* ni;
        for (li = node->LinkList.begin(); li != node->LinkList.end(); ++li) {
            // calc vector of currentnode
            if ((*li)->getStartNode() == node) {
                ni = (*li)->getEndNode();
            } else {
                ni = (*li)->getStartNode();
            }
            Position v1(
                ni->getPosition().x() - node->getPosition().x(),
                ni->getPosition().y() - node->getPosition().y());
            // loop over all links
            NGEdgeList::iterator lj;
            for (lj = node->LinkList.begin(); lj != node->LinkList.end(); ++lj) {
                if (li != lj) {
                    if ((*lj)->getStartNode() == node) {
                        ni = (*lj)->getEndNode();
                    } else {
                        ni = (*lj)->getStartNode();
                    }
                    Position v2(
                        ni->getPosition().x() - node->getPosition().x(),
                        ni->getPosition().y() - node->getPosition().y());
                    if (fabs(GeomHelper::angle2D(v1, v2)) < myMinLinkAngle) {
                        check = false;
                    }
                }
            }
        }
    }
    return check;
}


bool
NGRandomNetBuilder::canConnect(NGNode* baseNode, NGNode* newNode) {
    bool connectable = true;
    const PositionVector n(baseNode->getPosition(), newNode->getPosition());

    // check for range between Basenode and Newnode
    if (connectable) {
        SUMOReal dist = n.length();
        if ((dist < myMinDistance) || (dist > myMaxDistance)) {
            connectable = false;
        }
    }

    // check for angle restrictions
    if (connectable) {
        connectable = checkAngles(baseNode);
    }
    if (connectable) {
        connectable = checkAngles(newNode);
    }

    // check for intersections and range restrictions with outer links
    if (connectable) {
        NGEdgeList::iterator li;
        li = myOuterLinks.begin();
        while (connectable && (li != myOuterLinks.end())) {
            // check intersection only if links don't share a node
            const NGNode* const start = (*li)->getStartNode();
            const NGNode* const end = (*li)->getEndNode();
            const Position& p1 = start->getPosition();
            const Position& p2 = end->getPosition();
            if ((baseNode != start) && (baseNode != end) && (newNode != start) && (newNode != end)) {
                connectable = !n.intersects(p1, p2);
            }
            // check NewNode-To-Links distance only, if NewNode isn't part of link
            if (connectable && (newNode != start) && (newNode != end)) {
                const SUMOReal offset = GeomHelper::nearest_offset_on_line_to_point2D(p1, p2, n[1]);
                if (offset != GeomHelper::INVALID_OFFSET) {
                    const Position p = PositionVector(p1, p2).positionAtOffset2D(offset);
                    const SUMOReal dist = p.distanceTo2D(n[1]);
                    if (dist < myMinDistance) {
                        connectable = false;
                    }
                }
            }
            ++li;
        }
    }
    return connectable;
}


void
NGRandomNetBuilder::findPossibleOuterNodes(NGNode* node) {
    myConNodes.clear();
    NGNodeList::iterator ni;
    for (ni = myOuterNodes.begin(); ni != myOuterNodes.end(); ++ni) {
        NGNode* on = *ni;
        if (!node->connected(on)) {
            if ((node->getMaxNeighbours() > node->LinkList.size()) &&
                    ((on)->getMaxNeighbours() > (on)->LinkList.size())) {
                if (canConnect(node, on)) {
                    myConNodes.push_back(on);
                }
            }
        }
    }
}


bool
NGRandomNetBuilder::createNewNode(NGNode* baseNode) {
    // calculate position of new node based on BaseNode
    SUMOReal dist = RandHelper::rand(myMinDistance, myMaxDistance);
    SUMOReal angle = RandHelper::rand((SUMOReal)(2 * M_PI));
    SUMOReal x = baseNode->getPosition().x() + dist * cos(angle);
    SUMOReal y = baseNode->getPosition().y() + dist * sin(angle);
    NGNode* newNode = new NGNode(myNet.getNextFreeID());
    newNode->setX(x);
    newNode->setY(y);
    newNode->setMaxNeighbours((SUMOReal) myNeighbourDistribution.num());
    NGEdge* newLink = new NGEdge(myNet.getNextFreeID(), baseNode, newNode);
    if (canConnect(baseNode, newNode)) {
        // add node
        myNet.add(newNode);
        myOuterNodes.push_front(newNode);
        // add link
        myNet.add(newLink);
        myOuterLinks.push_back(newLink);
        // check basenode for being outer node
        if (baseNode->LinkList.size() >= baseNode->getMaxNeighbours()) {
            removeOuterNode(baseNode);
        }
        return true;
    } else {
        delete newNode;
        return false;
    }
}


void
NGRandomNetBuilder::createNet(int numNodes) {
    myNumNodes = numNodes;

    NGNode* outerNode = new NGNode(myNet.getNextFreeID());
    outerNode->setX(0);
    outerNode->setY(0);
    outerNode->setMaxNeighbours(4);

    myNet.add(outerNode);
    myOuterNodes.push_back(outerNode);

    bool created = true;
    while ((myNet.nodeNo() < numNodes) && (myOuterNodes.size() > 0)) {
        // brings last element to front
        if (!created) {
            myOuterNodes.push_front(myOuterNodes.back());
            myOuterNodes.pop_back();
        }
        outerNode = myOuterNodes.back();
        findPossibleOuterNodes(outerNode);
        created = false;
        if ((myConNodes.size() > 0) && (RandHelper::rand() < myConnectivity)) {
            // create link
            NGEdge* newLink = new NGEdge(myNet.getNextFreeID(), outerNode, myConNodes.back());
            if (canConnect(outerNode, myConNodes.back())) {
                // add link
                myNet.add(newLink);
                myOuterLinks.push_back(newLink);
                // check nodes for being outer node
                if (outerNode->LinkList.size() >= outerNode->getMaxNeighbours()) {
                    removeOuterNode(outerNode);
                }
                if (myConNodes.back()->LinkList.size() >= myConNodes.back()->getMaxNeighbours()) {
                    removeOuterNode(myConNodes.back());
                }
                created = true;
            } else {
                delete newLink;
            }
        } else {
            int count = 0;
            do {
                created = createNewNode(outerNode);
                count++;
            } while ((count <= myNumTries) && !created);
            if (!created) {
                outerNode->setMaxNeighbours((SUMOReal) outerNode->LinkList.size());
                myOuterNodes.remove(outerNode);
            }
        }
    }
}


/****************************************************************************/

