/****************************************************************************/
/// @file    GNEChange_Junction.cpp
/// @author  Jakob Erdmann
/// @date    Mar 2011
/// @version $Id: GNEChange_Junction.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// A network change in which a single junction is created or deleted
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright (C) 2001-2014 DLR (http://www.dlr.de/) and contributors
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

#include "GNEChange_Junction.h"
#include "GNENet.h"
#include "GNEJunction.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif


// ===========================================================================
// FOX-declarations
// ===========================================================================
FXIMPLEMENT_ABSTRACT(GNEChange_Junction, GNEChange, NULL, 0)

// ===========================================================================
// member method definitions
// ===========================================================================


// Constructor for creating a junction
GNEChange_Junction::GNEChange_Junction(GNENet* net, GNEJunction* junction, bool forward):
    GNEChange(net, forward),
    myJunction(junction) {
    assert(myNet);
    junction->incRef("GNEChange_Junction");
}


GNEChange_Junction::~GNEChange_Junction() {
    assert(myJunction);
    myJunction->decRef("GNEChange_Junction");
    if (myJunction->unreferenced()) {
        delete myJunction;
    }
}


void GNEChange_Junction::undo() {
    if (myForward) {
        myNet->deleteSingleJunction(myJunction);
    } else {
        myNet->insertJunction(myJunction);
    }
}


void GNEChange_Junction::redo() {
    if (myForward) {
        myNet->insertJunction(myJunction);
    } else {
        myNet->deleteSingleJunction(myJunction);
    }
}


FXString GNEChange_Junction::undoName() const {
    if (myForward) {
        return ("Undo create junction");
    } else {
        return ("Undo delete junction");
    }
}


FXString GNEChange_Junction::redoName() const {
    if (myForward) {
        return ("Redo create junction");
    } else {
        return ("Redo delete junction");
    }
}
