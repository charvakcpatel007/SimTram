/****************************************************************************/
/// @file    NIXMLConnectionsHandler.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Thu, 17 Oct 2002
/// @version $Id: NIXMLConnectionsHandler.cpp 21201 2016-07-19 11:57:22Z behrisch $
///
// Importer for edge connections stored in XML
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
#include <iostream>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/sax/AttributeList.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>
#include "NIXMLConnectionsHandler.h"
#include <netbuild/NBEdge.h>
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBNodeCont.h>
#include <netbuild/NBTrafficLightLogicCont.h>
#include <netbuild/NBNode.h>
#include <utils/common/StringTokenizer.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/common/ToString.h>
#include <utils/common/TplConvert.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/options/OptionsCont.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NIXMLConnectionsHandler::NIXMLConnectionsHandler(NBEdgeCont& ec, NBNodeCont& nc, NBTrafficLightLogicCont& tlc) :
    SUMOSAXHandler("xml-connection-description"),
    myEdgeCont(ec),
    myNodeCont(nc),
    myTLLogicCont(tlc),
    myHaveWarnedAboutDeprecatedLanes(false),
    myErrorMsgHandler(OptionsCont::getOptions().getBool("ignore-errors.connections") ?
                      MsgHandler::getWarningInstance() : MsgHandler::getErrorInstance()) {}


NIXMLConnectionsHandler::~NIXMLConnectionsHandler() {}


void
NIXMLConnectionsHandler::myStartElement(int element,
                                        const SUMOSAXAttributes& attrs) {
    if (element == SUMO_TAG_DELETE) {
        bool ok = true;
        std::string from = attrs.get<std::string>(SUMO_ATTR_FROM, 0, ok);
        std::string to = attrs.get<std::string>(SUMO_ATTR_TO, 0, ok);
        if (!ok) {
            return;
        }
        // these connections were removed when the edge was deleted
        if (myEdgeCont.wasRemoved(from) || myEdgeCont.wasRemoved(to)) {
            return;
        }
        NBEdge* fromEdge = myEdgeCont.retrieve(from);
        NBEdge* toEdge = myEdgeCont.retrieve(to);
        if (fromEdge == 0) {
            myErrorMsgHandler->inform("The connection-source edge '" + from + "' to reset is not known.");
            return;
        }
        if (toEdge == 0) {
            myErrorMsgHandler->inform("The connection-destination edge '" + to + "' to reset is not known.");
            return;
        }
        if (!fromEdge->isConnectedTo(toEdge) && fromEdge->getStep() >= NBEdge::EDGE2EDGES) {
            WRITE_WARNING("Target edge '" + toEdge->getID() + "' is not connected with '" + fromEdge->getID() + "'; the connection cannot be reset.");
            return;
        }
        int fromLane = -1; // Assume all lanes are to be reset.
        int toLane = -1;
        if (attrs.hasAttribute(SUMO_ATTR_LANE)
                || attrs.hasAttribute(SUMO_ATTR_FROM_LANE)
                || attrs.hasAttribute(SUMO_ATTR_TO_LANE)) {
            if (!parseLaneInfo(attrs, fromEdge, toEdge, &fromLane, &toLane)) {
                return;
            }
            // we could be trying to reset a connection loaded from a sumo net and which has become obsolete.
            // In this case it's ok to encounter invalid lance indices
            if (!fromEdge->hasConnectionTo(toEdge, toLane) && fromEdge->getStep() >= NBEdge::LANES2EDGES) {
                WRITE_WARNING("Edge '" + fromEdge->getID() + "' has no connection to lane " + toString(toLane) + " of edge '" + toEdge->getID() + "'; the connection cannot be reset.");
            }
        }
        fromEdge->removeFromConnections(toEdge, fromLane, toLane, true);
    }

    if (element == SUMO_TAG_CONNECTION) {
        bool ok = true;
        std::string from = attrs.get<std::string>(SUMO_ATTR_FROM, "connection", ok);
        std::string to = attrs.getOpt<std::string>(SUMO_ATTR_TO, "connection", ok, "");
        if (!ok || myEdgeCont.wasIgnored(from) || myEdgeCont.wasIgnored(to)) {
            return;
        }
        // extract edges
        NBEdge* fromEdge = myEdgeCont.retrieve(from);
        NBEdge* toEdge = to.length() != 0 ? myEdgeCont.retrieve(to) : 0;
        // check whether they are valid
        if (fromEdge == 0) {
            myErrorMsgHandler->inform("The connection-source edge '" + from + "' is not known.");
            return;
        }
        if (toEdge == 0 && to.length() != 0) {
            myErrorMsgHandler->inform("The connection-destination edge '" + to + "' is not known.");
            return;
        }
        // invalidate traffic light definition loaded from a SUMO network
        // XXX it would be preferable to reconstruct the phase definitions heuristically
        fromEdge->getToNode()->invalidateTLS(myTLLogicCont);
        // parse optional lane information
        if (attrs.hasAttribute(SUMO_ATTR_LANE) || attrs.hasAttribute(SUMO_ATTR_FROM_LANE) || attrs.hasAttribute(SUMO_ATTR_TO_LANE)) {
            parseLaneBound(attrs, fromEdge, toEdge);
        } else {
            fromEdge->addEdge2EdgeConnection(toEdge);
        }
    }
    if (element == SUMO_TAG_PROHIBITION) {
        bool ok = true;
        std::string prohibitor = attrs.getOpt<std::string>(SUMO_ATTR_PROHIBITOR, 0, ok, "");
        std::string prohibited = attrs.getOpt<std::string>(SUMO_ATTR_PROHIBITED, 0, ok, "");
        if (!ok) {
            return;
        }
        NBConnection prohibitorC = parseConnection("prohibitor", prohibitor);
        NBConnection prohibitedC = parseConnection("prohibited", prohibited);
        if (prohibitorC == NBConnection::InvalidConnection || prohibitedC == NBConnection::InvalidConnection) {
            // something failed
            return;
        }
        NBNode* n = prohibitorC.getFrom()->getToNode();
        n->addSortedLinkFoes(prohibitorC, prohibitedC);
    }
    if (element == SUMO_TAG_CROSSING) {
        addCrossing(attrs);
    }
    if (element == SUMO_TAG_CUSTOMSHAPE) {
        addCustomShape(attrs);
    }
}


NBConnection
NIXMLConnectionsHandler::parseConnection(const std::string& defRole, const std::string& def) {
    // split from/to
    const std::string::size_type div = def.find("->");
    if (div == std::string::npos) {
        myErrorMsgHandler->inform("Missing connection divider in " + defRole + " '" + def + "'");
        return NBConnection::InvalidConnection;
    }
    std::string fromDef = def.substr(0, div);
    std::string toDef = def.substr(div + 2);

    // retrieve the edges
    // check whether the definition includes a lane information (do not process it)
    if (fromDef.find('_') != std::string::npos) {
        fromDef = fromDef.substr(0, fromDef.find('_'));
    }
    if (toDef.find('_') != std::string::npos) {
        toDef = toDef.substr(0, toDef.find('_'));
    }
    // retrieve them now
    NBEdge* fromE = myEdgeCont.retrieve(fromDef);
    NBEdge* toE = myEdgeCont.retrieve(toDef);
    // check
    if (fromE == 0) {
        myErrorMsgHandler->inform("Could not find edge '" + fromDef + "' in " + defRole + " '" + def + "'");
        return NBConnection::InvalidConnection;
    }
    if (toE == 0) {
        myErrorMsgHandler->inform("Could not find edge '" + toDef + "' in " + defRole + " '" + def + "'");
        return NBConnection::InvalidConnection;
    }
    return NBConnection(fromE, toE);
}


void
NIXMLConnectionsHandler::parseLaneBound(const SUMOSAXAttributes& attrs, NBEdge* from, NBEdge* to) {
    if (to == 0) {
        // do nothing if it's a dead end
        return;
    }
    bool ok = true;
    const bool mayDefinitelyPass = attrs.getOpt<bool>(SUMO_ATTR_PASS, 0, ok, false);
    const bool keepClear = attrs.getOpt<bool>(SUMO_ATTR_KEEP_CLEAR, 0, ok, true);
    const SUMOReal contPos = attrs.getOpt<SUMOReal>(SUMO_ATTR_CONTPOS, 0, ok, NBEdge::UNSPECIFIED_CONTPOS);
    if (!ok) {
        return;
    }
    // get the begin and the end lane
    int fromLane;
    int toLane;
    try {
        if (!parseLaneInfo(attrs, from, to, &fromLane, &toLane)) {
            return;
        }
        if (fromLane < 0) {
            myErrorMsgHandler->inform("Invalid value '" + toString(fromLane) +
                                      "' for " + toString(SUMO_ATTR_FROM_LANE) + " in connection from '" +
                                      from->getID() + "' to '" + to->getID() + "'.");
            return;
        }
        if (toLane < 0) {
            myErrorMsgHandler->inform("Invalid value '" + toString(toLane) +
                                      "' for " + toString(SUMO_ATTR_TO_LANE) + " in connection from '" +
                                      from->getID() + "' to '" + to->getID() + "'.");
            return;
        }
        if (from->hasConnectionTo(to, toLane) && from->getToNode()->getType() != NODETYPE_ZIPPER) {
            WRITE_WARNING("Target lane '" + to->getLaneID(toLane) + "' is already connected from '" + from->getID() + "'.");
        }
        if (!from->addLane2LaneConnection(fromLane, to, toLane, NBEdge::L2L_USER, true, mayDefinitelyPass, keepClear, contPos)) {
            if (OptionsCont::getOptions().getBool("show-errors.connections-first-try")) {
                WRITE_WARNING("Could not set loaded connection from '" + from->getLaneID(fromLane) + "' to '" + to->getLaneID(toLane) + "'.");
            }
            // set as to be re-applied after network processing
            myEdgeCont.addPostProcessConnection(from->getID(), fromLane, to->getID(), toLane, mayDefinitelyPass, keepClear, contPos);
        }
    } catch (NumberFormatException&) {
        myErrorMsgHandler->inform("At least one of the defined lanes was not numeric");
    }
    //
    bool keepUncontrolled = attrs.getOpt<bool>(SUMO_ATTR_UNCONTROLLED, 0, ok, false);
    if (keepUncontrolled) {
        from->disableConnection4TLS(fromLane, to, toLane);
    }
}

bool
NIXMLConnectionsHandler::parseLaneInfo(const SUMOSAXAttributes& attributes, NBEdge* fromEdge, NBEdge* toEdge,
                                       int* fromLane, int* toLane) {
    if (attributes.hasAttribute(SUMO_ATTR_LANE)) {
        return parseDeprecatedLaneDefinition(attributes, fromEdge, toEdge, fromLane, toLane);
    } else {
        return parseLaneDefinition(attributes, fromLane, toLane);
    }
}


inline bool
NIXMLConnectionsHandler::parseDeprecatedLaneDefinition(const SUMOSAXAttributes& attributes,
        NBEdge* from, NBEdge* to,
        int* fromLane, int* toLane) {
    bool ok = true;
    if (!myHaveWarnedAboutDeprecatedLanes) {
        myHaveWarnedAboutDeprecatedLanes = true;
        WRITE_WARNING("'" + toString(SUMO_ATTR_LANE) + "' is deprecated, please use '" +
                      toString(SUMO_ATTR_FROM_LANE) + "' and '" + toString(SUMO_ATTR_TO_LANE) +
                      "' instead.");
    }

    std::string laneConn = attributes.get<std::string>(SUMO_ATTR_LANE, 0, ok);
    StringTokenizer st(laneConn, ':');
    if (!ok || st.size() != 2) {
        myErrorMsgHandler->inform("Invalid lane to lane connection from '" +
                                  from->getID() + "' to '" + to->getID() + "'.");
        return false; // There was an error.
    }

    *fromLane = TplConvert::_2intSec(st.next().c_str(), -1);
    *toLane = TplConvert::_2intSec(st.next().c_str(), -1);

    return true; // We succeeded.
}


inline bool
NIXMLConnectionsHandler::parseLaneDefinition(const SUMOSAXAttributes& attributes,
        int* fromLane,
        int* toLane) {
    bool ok = true;
    *fromLane = attributes.get<int>(SUMO_ATTR_FROM_LANE, 0, ok);
    *toLane = attributes.get<int>(SUMO_ATTR_TO_LANE, 0, ok);
    return ok;
}


void
NIXMLConnectionsHandler::addCrossing(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    NBNode* node = 0;
    EdgeVector edges;
    const std::string nodeID = attrs.get<std::string>(SUMO_ATTR_NODE, 0, ok);
    const SUMOReal width = attrs.getOpt<SUMOReal>(SUMO_ATTR_WIDTH, nodeID.c_str(), ok, NBNode::DEFAULT_CROSSING_WIDTH, true);
    const bool discard = attrs.getOpt<bool>(SUMO_ATTR_DISCARD, nodeID.c_str(), ok, false, true);
    std::vector<std::string> edgeIDs;
    if (!attrs.hasAttribute(SUMO_ATTR_EDGES)) {
        if (discard) {
            node = myNodeCont.retrieve(nodeID);
            if (node == 0) {
                WRITE_ERROR("Node '" + nodeID + "' in crossing is not known.");
                return;
            }
            node->discardAllCrossings();
            return;
        } else {
            WRITE_ERROR("No edges specified for crossing at node '" + nodeID + "'.");
            return;
        }
    }
    SUMOSAXAttributes::parseStringVector(attrs.get<std::string>(SUMO_ATTR_EDGES, 0, ok), edgeIDs);
    if (!ok) {
        return;
    }
    for (std::vector<std::string>::const_iterator it = edgeIDs.begin(); it != edgeIDs.end(); ++it) {
        NBEdge* edge = myEdgeCont.retrieve(*it);
        if (edge == 0) {
            WRITE_ERROR("Edge '" + (*it) + "' for crossing at node '" + nodeID + "' is not known.");
            return;
        }
        if (node == 0) {
            if (edge->getToNode()->getID() == nodeID) {
                node = edge->getToNode();
            } else if (edge->getFromNode()->getID() == nodeID) {
                node = edge->getFromNode();
            } else {
                WRITE_ERROR("Edge '" + (*it) + "' does not touch node '" + nodeID + "'.");
                return;
            }
        } else {
            if (edge->getToNode() != node && edge->getFromNode() != node) {
                WRITE_ERROR("Edge '" + (*it) + "' does not touch node '" + nodeID + "'.");
                return;
            }
        }
        edges.push_back(edge);
    }
    bool priority = attrs.getOpt<bool>(SUMO_ATTR_PRIORITY, nodeID.c_str(), ok, node->isTLControlled(), true);
    if (node->isTLControlled() && !priority) {
        // traffic_light nodes should always have priority crossings
        WRITE_WARNING("Crossing at controlled node '" + nodeID + "' must be prioritized");
        priority = true;
    }
    if (discard) {
        node->removeCrossing(edges);
    } else {
        node->addCrossing(edges, width, priority);
    }
}


void
NIXMLConnectionsHandler::addCustomShape(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, 0, ok);
    const PositionVector shape = attrs.get<PositionVector>(SUMO_ATTR_SHAPE, id.c_str(), ok);
    if (!ok) {
        return;
    }
    const std::string nodeID = NBNode::getNodeIDFromInternalLane(id);
    NBNode* node = myNodeCont.retrieve(nodeID);
    if (node == 0) {
        WRITE_ERROR("Node '" + nodeID + "' in customShape is not known.");
        return;
    }
    node->setCustomLaneShape(id, shape);
}



/****************************************************************************/

