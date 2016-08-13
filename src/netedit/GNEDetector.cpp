/****************************************************************************/
/// @file    GNEDetectorE1.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
/// @version $Id: GNEDetector.cpp 21182 2016-07-18 06:46:01Z behrisch $
///
///
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2001-2013 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 3 of the License, or
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
#include <utility>
#include <foreign/polyfonts/polyfonts.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/common/ToString.h>
#include <utils/geom/GeomHelper.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/xml/SUMOSAXHandler.h>

#include "GNEDetector.h"
#include "GNELane.h"
#include "GNEChange_Attribute.h"
#include "GNEUndoList.h"
#include "GNENet.h"
#include "GNEViewNet.h"


#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif

// ===========================================================================
// member method definitions
// ===========================================================================

GNEDetector::GNEDetector(const std::string& id, GNEViewNet* viewNet, SumoXMLTag tag, GNELane* lane, SUMOReal posOverLane, int freq, const std::string& filename, bool blocked, GNEAdditionalSet* parent) :
    GNEAdditional(id, viewNet, Position(posOverLane, 0), tag, parent, blocked),
    myLane(lane),
    myFreq(freq),
    myFilename(filename) {
    myLane->addAdditional(this);
}


GNEDetector::~GNEDetector() {
    if (myLane) {
        myLane->removeAdditional(this);
    }
}


void
GNEDetector::moveAdditional(SUMOReal posx, SUMOReal posy, GNEUndoList* undoList) {
    // Due a detector is placed over an lane ignore Warning of posy
    UNUSED_PARAMETER(posy);
    // if item isn't blocked
    if (myBlocked == false) {
        // Move to Right if posx is positive, to left if posx is negative
        if (((posx > 0) && ((myPosition.x() + posx) < myLane->getLaneShapeLenght())) || ((posx < 0) && ((myPosition.x() + posx) > 0))) {
            // change attribute
            undoList->p_add(new GNEChange_Attribute(this, SUMO_ATTR_POSITION, toString(myPosition.x() + posx)));
        }
    }
}


GNELane*
GNEDetector::getLane() const {
    return myLane;
}


void
GNEDetector::removeLaneReference() {
    myLane = NULL;
}


SUMOReal
GNEDetector::getPositionOverLane() const {
    return myPosition.x();
}


int
GNEDetector::getFrequency() const {
    return myFreq;
}


std::string
GNEDetector::getFilename() const {
    return myFilename;
}


void
GNEDetector::setPositionOverLane(SUMOReal pos) {
    if (pos < 0) {
        throw InvalidArgument("Position '" + toString(pos) + "' not allowed. Must be greather than 0");
    } else if (pos > myLane->getLaneShapeLenght()) {
        throw InvalidArgument("Position '" + toString(pos) + "' not allowed. Must be smaller than lane length");
    } else {
        myPosition = Position(pos, 0);
    }
}


void
GNEDetector::setFrequency(int freq) {
    if (freq >= 0) {
        myFreq = freq;
    } else {
        throw InvalidArgument("Frequency '" + toString(freq) + "' not allowed. Must be greather than 0");
    }
}


void
GNEDetector::setFilename(std::string filename) {
    myFilename = filename;
}


void
GNEDetector::changeLane(GNELane* newLane) {
    myLane->removeAdditional(this);
    myLane = newLane;
    myLane->addAdditional(this);
    updateGeometry();
    getViewNet()->update();
}


const std::string&
GNEDetector::getParentName() const {
    return myLane->getMicrosimID();
}


void
GNEDetector::drawDetectorIcon(const int GNELogoID, SUMOReal sizex, SUMOReal sizey) const {
    // Add a draw matrix
    glPushMatrix();
    // Traslate to center
    glTranslated(myShape.getLineCenter().x(), myShape.getLineCenter().y(), getType() + 0.1);
    // Set color
    glColor3d(1, 1, 1);
    // Set first rotation
    glRotated(myBlockIconRotation, 0, 0, -1);
    // Set second rotation
    glRotated(180, 0, 0, 1);
    //Traslate offset
    glTranslated(myDetectorLogoOffset.x(), myDetectorLogoOffset.y(), 0);
    // Draw detector logo
    GUITexturesHelper::drawTexturedBox(GNELogoID, sizex, sizey, (-1 * sizex), (-1 * sizey));
    // Pop detector logo matrix
    glPopMatrix();
}

/****************************************************************************/
