/****************************************************************************/
/// @file    GNEChange_TLS.cpp
/// @author  Jakob Erdmann
/// @date    July 2011
/// @version $Id: GNEChange_TLS.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// A network change in which a traffic light is created or deleted
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

#include <cassert>
#include <utils/options/OptionsCont.h>
#include <netbuild/NBTrafficLightDefinition.h>
#include <netbuild/NBOwnTLDef.h>
#include "GNEChange_TLS.h"
#include "GNEJunction.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif


// ===========================================================================
// FOX-declarations
// ===========================================================================
FXIMPLEMENT_ABSTRACT(GNEChange_TLS, GNEChange, NULL, 0)

// ===========================================================================
// member method definitions
// ===========================================================================


// Constructor for creating an edge
GNEChange_TLS::GNEChange_TLS(
    GNEJunction* junction, NBTrafficLightDefinition* tlDef, bool forward, bool forceInsert, const std::string tlID):
    GNEChange(0, forward),
    myJunction(junction),
    myTlDef(tlDef),
    myForceInsert(forceInsert) {
    myJunction->incRef("GNEChange_TLS");
    if (myTlDef == 0) {
        assert(forward);
        // potential memory leak if this change is never executed
        TrafficLightType type = SUMOXMLDefinitions::TrafficLightTypes.get(OptionsCont::getOptions().getString("tls.default-type"));
        myTlDef = new NBOwnTLDef(tlID == "" ? myJunction->getMicrosimID() : tlID, 0, type);
    }
}


GNEChange_TLS::~GNEChange_TLS() {
    assert(myJunction);
    myJunction->decRef("GNEChange_TLS");
    if (myJunction->unreferenced()) {
        delete myJunction;
    }
}


void GNEChange_TLS::undo() {
    if (myForward) {
        myJunction->removeTrafficLight(myTlDef);
    } else {
        myJunction->addTrafficLight(myTlDef, myForceInsert);
    }
}


void GNEChange_TLS::redo() {
    if (myForward) {
        myJunction->addTrafficLight(myTlDef, myForceInsert);
    } else {
        myJunction->removeTrafficLight(myTlDef);
    }
}


FXString GNEChange_TLS::undoName() const {
    if (myForward) {
        return ("Undo create traffic light");
    } else {
        return ("Undo delete traffic light");
    }
}


FXString GNEChange_TLS::redoName() const {
    if (myForward) {
        return ("Redo create traffic light");
    } else {
        return ("Redo delete traffic light");
    }
}
