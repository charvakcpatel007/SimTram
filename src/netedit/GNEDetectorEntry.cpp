/****************************************************************************/
/// @file    GNEDetectorEntry.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
/// @version $Id: GNEDetectorEntry.cpp 21150 2016-07-12 12:28:35Z behrisch $
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
#include <utils/gui/images/GUITextureSubSys.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/xml/SUMOSAXHandler.h>

#include "GNEDetectorEntry.h"
#include "GNEDetectorE3.h"
#include "GNELane.h"
#include "GNEViewNet.h"
#include "GNEUndoList.h"
#include "GNENet.h"
#include "GNEChange_Attribute.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif

// ===========================================================================
// member method definitions
// ===========================================================================

GNEDetectorEntry::GNEDetectorEntry(const std::string& id, GNEViewNet* viewNet, GNELane* lane, SUMOReal pos, GNEDetectorE3* parent, bool blocked) :
    GNEDetector(id, viewNet, SUMO_TAG_DET_ENTRY, lane, pos, 0, "", blocked, parent) {
    // Update geometry;
    updateGeometry();
    // Set colors
    myBaseColor = RGBColor(0, 204, 0, 255);
    myBaseColorSelected = RGBColor(125, 204, 0, 255);
}


GNEDetectorEntry::~GNEDetectorEntry() {}


void
GNEDetectorEntry::updateGeometry() {
    // Clear all containers
    myShapeRotations.clear();
    myShapeLengths.clear();

    // clear Shape
    myShape.clear();

    // Get shape of lane parent
    myShape.push_back(myLane->getShape().positionAtOffset(myLane->getPositionRelativeToParametricLenght(myPosition.x())));

    // Save rotation (angle) of the vector constructed by points f and s
    myShapeRotations.push_back(myLane->getShape().rotationDegreeAtOffset(myLane->getPositionRelativeToParametricLenght(myPosition.x())) * -1);

    // Set block icon position
    myBlockIconPosition = myShape.getLineCenter();

    // Set offset of logo
    myDetectorLogoOffset = Position(-2, 0);

    // Set block icon rotation, and using their rotation for logo
    setBlockIconRotation(myLane);

    // Update parent geometry
    myAdditionalSetParent->updateGeometry();
}


Position
GNEDetectorEntry::getPositionInView() const {
    return myLane->getShape().positionAtOffset(myLane->getPositionRelativeToParametricLenght(myPosition.x()));
}


void
GNEDetectorEntry::writeAdditional(OutputDevice& device, const std::string&) {
    // Write parameters
    device.openTag(getTag());
    device.writeAttr(SUMO_ATTR_LANE, myLane->getID());
    device.writeAttr(SUMO_ATTR_POSITION, myPosition.x());
    // Close tag
    device.closeTag();
}


void
GNEDetectorEntry::drawGL(const GUIVisualizationSettings& s) const {
    // Start drawing adding gl identificator
    glPushName(getGlID());

    // Push detector matrix
    glPushMatrix();
    glTranslated(0, 0, getType());

    // Set initial values
    if (isAdditionalSelected()) {
        glColor3d(myBaseColorSelected.red(), myBaseColorSelected.green(), myBaseColorSelected.blue());
    } else {
        glColor3d(myBaseColor.red(), myBaseColor.green(), myBaseColor.blue());
    }
    const SUMOReal exaggeration = s.addSize.getExaggeration(s);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Push poligon matrix
    glPushMatrix();
    glScaled(exaggeration, exaggeration, 1);
    glTranslated(myShape[0].x(), myShape[0].y(), 0);
    glRotated(myShapeRotations[0], 0, 0, 1);

    // Draw poligon
    glBegin(GL_LINES);
    glVertex2d(1.7, 0);
    glVertex2d(-1.7, 0);
    glEnd();
    glBegin(GL_QUADS);
    glVertex2d(-1.7, .5);
    glVertex2d(-1.7, -.5);
    glVertex2d(1.7, -.5);
    glVertex2d(1.7, .5);
    glEnd();

    // first Arrow
    glTranslated(1.5, 0, 0);
    GLHelper::drawBoxLine(Position(0, 4), 0, 2, .05);
    GLHelper::drawTriangleAtEnd(Position(0, 4), Position(0, 1), (SUMOReal) 1, (SUMOReal) .25);

    // second Arrow
    glTranslated(-3, 0, 0);
    GLHelper::drawBoxLine(Position(0, 4), 0, 2, .05);
    GLHelper::drawTriangleAtEnd(Position(0, 4), Position(0, 1), (SUMOReal) 1, (SUMOReal) .25);

    // Pop poligon matrix
    glPopMatrix();

    // Pop detector matrix
    glPopMatrix();

    // Check if the distance is enought to draw details
    if (s.scale * exaggeration >= 10) {
        // Draw icon
        drawDetectorIcon(GUITextureSubSys::getGif(GNETEXTURE_ENTRY), 1.5, 1);

        // Show Lock icon depending of the Edit mode
        drawLockIcon(0.4);
    }
    // Draw name
    drawName(getCenteringBoundary().getCenter(), s.scale, s.addName);

    // pop gl identificator
    glPopName();
}


std::string
GNEDetectorEntry::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_LANE:
            return toString(myLane->getAttribute(SUMO_ATTR_ID));
        case SUMO_ATTR_POSITION:
            return toString(myPosition.x());
        default:
            throw InvalidArgument(toString(getType()) + " attribute '" + toString(key) + "' not allowed");
    }
}


void
GNEDetectorEntry::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
            setAdditionalID(value);
            break;
        case SUMO_ATTR_LANE:
            changeLane(myViewNet->getNet()->retrieveLane(value));
            break;
        case SUMO_ATTR_POSITION:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            updateGeometry();
            break;
        default:
            throw InvalidArgument(toString(getType()) + " attribute '" + toString(key) + "' not allowed");
    }
}


bool
GNEDetectorEntry::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            if (myViewNet->getNet()->getAdditional(getTag(), value) == NULL) {
                return true;
            } else {
                return false;
            }
        case SUMO_ATTR_LANE:
            if (myViewNet->getNet()->retrieveLane(value, false) != NULL) {
                return true;
            } else {
                return false;
            }
        case SUMO_ATTR_POSITION:
            return (canParse<SUMOReal>(value) && parse<SUMOReal>(value) >= 0 && parse<SUMOReal>(value) <= (myLane->getLaneParametricLenght()));
        default:
            throw InvalidArgument(toString(getType()) + " attribute '" + toString(key) + "' not allowed");
    }
}

void
GNEDetectorEntry::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            setAdditionalID(value);
            break;
        case SUMO_ATTR_LANE:
            changeLane(myViewNet->getNet()->retrieveLane(value));
            break;
        case SUMO_ATTR_POSITION:
            myPosition = Position(parse<SUMOReal>(value), 0);
            updateGeometry();
            getViewNet()->update();
            break;
        default:
            throw InvalidArgument(toString(getType()) + " attribute '" + toString(key) + "' not allowed");
    }
}

/****************************************************************************/
