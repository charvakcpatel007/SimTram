/****************************************************************************/
/// @file    NIVissimNodeDef_Edges.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: NIVissimNodeDef_Edges.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// -------------------
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



#include <string>
#include <map>
#include <algorithm>
#include <cassert>
#include <utils/geom/Boundary.h>
#include "NIVissimNodeParticipatingEdgeVector.h"
#include "NIVissimNodeDef.h"
#include "NIVissimEdge.h"
#include "NIVissimNodeDef_Edges.h"
#include "NIVissimDisturbance.h"
#include "NIVissimConnection.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimNodeDef_Edges::NIVissimNodeDef_Edges(int id,
        const std::string& name, const NIVissimNodeParticipatingEdgeVector& edges)
    : NIVissimNodeDef(id, name), myEdges(edges) {}


NIVissimNodeDef_Edges::~NIVissimNodeDef_Edges() {
    for (NIVissimNodeParticipatingEdgeVector::iterator i = myEdges.begin(); i != myEdges.end(); i++) {
        delete(*i);
    }
    myEdges.clear();
}


bool
NIVissimNodeDef_Edges::dictionary(int id, const std::string& name,
                                  const NIVissimNodeParticipatingEdgeVector& edges) {
    NIVissimNodeDef_Edges* o = new NIVissimNodeDef_Edges(id, name, edges);
    if (!NIVissimNodeDef::dictionary(id, o)) {
        delete o;
        return false;
    }
    return true;
}


/*
void
NIVissimNodeDef_Edges::searchAndSetConnections() {
    std::vector<int> connections;
    std::vector<int> edges;
    Boundary boundary;
    for (NIVissimNodeParticipatingEdgeVector::const_iterator i = myEdges.begin(); i != myEdges.end(); i++) {
        NIVissimNodeParticipatingEdge* edge = *i;
        NIVissimConnection* c =
            NIVissimConnection::dictionary(edge->getID());
        NIVissimEdge* e =
            NIVissimEdge::dictionary(edge->getID());
        if (c != 0) {
            connections.push_back(edge->getID());
            boundary.add(c->getFromGeomPosition());
            boundary.add(c->getToGeomPosition());
            c->setNodeCluster(myID);
        }
        if (e != 0) {
            edges.push_back(edge->getID());
            boundary.add(e->getGeomPosition(edge->getFromPos()));
            boundary.add(e->getGeomPosition(edge->getToPos()));
        }
    }
    NIVissimConnectionCluster* c =
        new NIVissimConnectionCluster(connections, boundary, myID, edges);
    for (std::vector<int>::iterator j = edges.begin(); j != edges.end(); j++) {
        NIVissimEdge* edge = NIVissimEdge::dictionary(*j);
        edge->myConnectionClusters.push_back(c);
    }
}
*/


SUMOReal
NIVissimNodeDef_Edges::getEdgePosition(int edgeid) const {
    for (NIVissimNodeParticipatingEdgeVector::const_iterator i = myEdges.begin(); i != myEdges.end(); i++) {
        NIVissimNodeParticipatingEdge* edge = *i;
        if (edge->getID() == edgeid) {
            return (edge->getFromPos() + edge->getToPos()) / (SUMOReal) 2.0;
        }
    }
    return -1;
}



/****************************************************************************/

