/****************************************************************************/
/// @file    NWWriter_MATSim.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, 04.05.2011
/// @version $Id: NWWriter_MATSim.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// Exporter writing networks using the MATSim format
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
#include "NWWriter_MATSim.h"
#include <utils/common/MsgHandler.h>
#include <netbuild/NBEdge.h>
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBNode.h>
#include <netbuild/NBNodeCont.h>
#include <netbuild/NBNetBuilder.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS



// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static methods
// ---------------------------------------------------------------------------
void
NWWriter_MATSim::writeNetwork(const OptionsCont& oc, NBNetBuilder& nb) {
    // check whether a matsim-file shall be generated
    if (!oc.isSet("matsim-output")) {
        return;
    }
    OutputDevice& device = OutputDevice::getDevice(oc.getString("matsim-output"));
    device << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n";
    device << "<!DOCTYPE network SYSTEM \"http://www.matsim.org/files/dtd/network_v1.dtd\">\n\n";
    device << "<network name=\"NAME\">\n"; // !!! name
    // write nodes
    device << "   <nodes>\n";
    NBNodeCont& nc = nb.getNodeCont();
    for (std::map<std::string, NBNode*>::const_iterator i = nc.begin(); i != nc.end(); ++i) {
        device << "      <node id=\"" << (*i).first
               << "\" x=\"" << (*i).second->getPosition().x()
               << "\" y=\"" << (*i).second->getPosition().y()
               << "\"/>\n";
    }
    device << "   </nodes>\n";
    // write edges
    device << "   <links capperiod=\"01:00:00\">\n";
    NBEdgeCont& ec = nb.getEdgeCont();
    for (std::map<std::string, NBEdge*>::const_iterator i = ec.begin(); i != ec.end(); ++i) {
        device << "      <link id=\"" << (*i).first
               << "\" from=\"" << (*i).second->getFromNode()->getID()
               << "\" to=\"" << (*i).second->getToNode()->getID()
               << "\" length=\"" << (*i).second->getLoadedLength()
               << "\" capacity=\"" << (oc.getFloat("lanes-from-capacity.norm") * (*i).second->getNumLanes())
               << "\" freespeed=\"" << (*i).second->getSpeed()
               << "\" permlanes=\"" << (*i).second->getNumLanes()
               << "\"/>\n";
    }
    device << "   </links>\n";
    //
    device << "</network>\n"; // !!! name
    device.close();
}


/****************************************************************************/

