/****************************************************************************/
/// @file    GNESelectorFrame.cpp
/// @author  Jakob Erdmann
/// @date    Mar 2011
/// @version $Id: GNESelectorFrame.cpp 21217 2016-07-22 10:57:44Z behrisch $
///
// The Widget for modifying selections of network-elements
// (some elements adapted from GUIDialog_GLChosenEditor)
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

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include <iostream>
#include <utils/foxtools/fxexdefs.h>
#include <utils/foxtools/MFXUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIIOGlobals.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include "GNESelectorFrame.h"
#include "GNEViewNet.h"
#include "GNEViewParent.h"
#include "GNENet.h"
#include "GNEJunction.h"
#include "GNEEdge.h"
#include "GNELane.h"
#include "GNEUndoList.h"
#include "GNEChange_Selection.h"
#include "GNEAttributeCarrier.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// FOX callback mapping
// ===========================================================================
FXDEFMAP(GNESelectorFrame) GNESelectorFrameMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_LOAD,       GNESelectorFrame::onCmdLoad),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_SAVE,       GNESelectorFrame::onCmdSave),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_INVERT,     GNESelectorFrame::onCmdInvert),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_CLEAR,      GNESelectorFrame::onCmdClear),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SELMB_TAG,      GNESelectorFrame::onCmdSelMBTag),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SELMB_STRING,   GNESelectorFrame::onCmdSelMBString),
    FXMAPFUNC(SEL_COMMAND,  MID_HELP,               GNESelectorFrame::onCmdHelp),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SELECT_SCALE,   GNESelectorFrame::onCmdScaleSelection)
};

// Object implementation
FXIMPLEMENT(GNESelectorFrame, FXScrollWindow, GNESelectorFrameMap, ARRAYNUMBER(GNESelectorFrameMap))

// ===========================================================================
// method definitions
// ===========================================================================
GNESelectorFrame::GNESelectorFrame(FXComposite* parent, GNEViewNet* viewNet):
    GNEFrame(parent, viewNet, getStats().c_str()),
    mySetOperation(SET_ADD),
    mySetOperationTarget(mySetOperation),
    ALL_VCLASS_NAMES_MATCH_STRING("all " + joinToString(SumoVehicleClassStrings.getStrings(), " ")) {
    // selection modification mode
    FXGroupBox* selBox = new FXGroupBox(myContentFrame, "Modification Mode",
                                        GROUPBOX_NORMAL | FRAME_GROOVE | LAYOUT_FILL_X, 2, 0, 0, 0, 4, 2, 2, 2);
    new FXRadioButton(selBox, "add\t\tSelected objects are added to the previous selection",
                      &mySetOperationTarget, FXDataTarget::ID_OPTION + SET_ADD);
    new FXRadioButton(selBox, "remove\t\tSelected objects are removed from the previous selection",
                      &mySetOperationTarget, FXDataTarget::ID_OPTION + SET_SUB);
    new FXRadioButton(selBox, "keep\t\tRestrict previous selection by the current selection",
                      &mySetOperationTarget, FXDataTarget::ID_OPTION + SET_RESTRICT);
    new FXRadioButton(selBox, "replace\t\tReplace previous selection by the current selection",
                      &mySetOperationTarget, FXDataTarget::ID_OPTION + SET_REPLACE);

    // selection by expression matching (match box)
    FXGroupBox* matchBox = new FXGroupBox(myContentFrame, "Match Attribute",
                                          GROUPBOX_NORMAL | FRAME_GROOVE | LAYOUT_FILL_X, 2, 0, 0, 0, 4, 2, 2, 2);
    myMatchTagBox = new FXListBox(matchBox, this, MID_GNE_SELMB_TAG);
    const std::vector<SumoXMLTag>& tags = GNEAttributeCarrier::allowedTags();
    for (std::vector<SumoXMLTag>::const_iterator it = tags.begin(); it != tags.end(); it++) {
        myMatchTagBox->appendItem(toString(*it).c_str());
    }
    myMatchTagBox->setCurrentItem(1); // edges
    myMatchTagBox->setNumVisible(myMatchTagBox->getNumItems());
    myMatchAttrBox = new FXListBox(matchBox);
    onCmdSelMBTag(0, 0, 0);
    myMatchAttrBox->setCurrentItem(3); // speed
    myMatchString = new FXTextField(matchBox, 12, this, MID_GNE_SELMB_STRING, TEXTFIELD_NORMAL, 0, 0, 0, 0, 4, 2, 0, 2);
    myMatchString->setText(">10.0");
    new FXButton(matchBox, "Help", 0, this, MID_HELP);

    FXGroupBox* selSizeBox = new FXGroupBox(myContentFrame, "Visual Scaling",
                                            GROUPBOX_NORMAL | FRAME_GROOVE | LAYOUT_FILL_X, 2, 0, 0, 0, 4, 2, 2, 2);
    mySelectionScaling =
        new FXRealSpinDial(selSizeBox, 7, this, MID_GNE_SELECT_SCALE,
                           LAYOUT_TOP | FRAME_SUNKEN | FRAME_THICK | LAYOUT_FILL_Y);
    mySelectionScaling->setNumberFormat(1);
    mySelectionScaling->setIncrements(0.1, .5, 1);
    mySelectionScaling->setRange(1, 100);
    mySelectionScaling->setValue(1);
    mySelectionScaling->setHelpText("Enlarge selected objects");

    // additional buttons
    // new FXHorizontalSeparator(this,SEPARATOR_GROOVE|LAYOUT_FILL_X);
    // "Clear List"
    new FXButton(myContentFrame, "Clear\t\t", 0, this, MID_CHOOSEN_CLEAR,
                 ICON_BEFORE_TEXT | LAYOUT_FILL_X | FRAME_THICK | FRAME_RAISED,
                 0, 0, 0, 0, 4, 4, 3, 3);
    // "Invert"
    new FXButton(myContentFrame, "Invert\t\t", 0, this, MID_CHOOSEN_INVERT,
                 ICON_BEFORE_TEXT | LAYOUT_FILL_X | FRAME_THICK | FRAME_RAISED,
                 0, 0, 0, 0, 4, 4, 3, 3);
    // "Save"
    new FXButton(myContentFrame, "Save\t\tSave ids of currently selected objects to a file.", 0, this, MID_CHOOSEN_SAVE,
                 ICON_BEFORE_TEXT | LAYOUT_FILL_X | FRAME_THICK | FRAME_RAISED,
                 0, 0, 0, 0, 4, 4, 3, 3);

    // "Load"
    new FXButton(myContentFrame, "Load\t\tLoad ids from a file according to the current modfication mode.", 0, this, MID_CHOOSEN_LOAD,
                 ICON_BEFORE_TEXT | LAYOUT_FILL_X | FRAME_THICK | FRAME_RAISED,
                 0, 0, 0, 0, 4, 4, 3, 3);


    // Selection Hint
    new FXLabel(myContentFrame, "Hold <SHIFT> for\nrectangle selection.\nPress <DEL> to\ndelete selected items.", 0, JUSTIFY_LEFT);
}


GNESelectorFrame::~GNESelectorFrame() {
    gSelected.remove2Update();
}

long
GNESelectorFrame::onCmdLoad(FXObject*, FXSelector, void*) {
    // get the new file name
    FXFileDialog opendialog(this, "Open List of Selected Items");
    opendialog.setIcon(GUIIconSubSys::getIcon(ICON_EMPTY));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList("Selection files (*.txt)\nAll files (*)");
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        std::string file = opendialog.getFilename().text();
        // @todo maybe rewrite so that mySetOperation also applies to loaded items?
        std::string errors;
        std::set<GUIGlID> ids = gSelected.loadIDs(file, errors);
        handleIDs(std::vector<GUIGlID>(ids.begin(), ids.end()), false);
        if (errors != "") {
            FXMessageBox::error(this, MBOX_OK, "Errors while loading Selection", "%s", errors.c_str());
        }
    }
    myViewNet->update();
    return 1;
}


long
GNESelectorFrame::onCmdSave(FXObject*, FXSelector, void*) {
    FXString file = MFXUtils::getFilename2Write(
                        this, "Save List of selected Items", ".txt", GUIIconSubSys::getIcon(ICON_EMPTY), gCurrentFolder);
    if (file == "") {
        return 1;
    }
    try {
        gSelected.save(file.text());
    } catch (IOError& e) {
        FXMessageBox::error(this, MBOX_OK, "Storing Selection failed", "%s", e.what());
    }
    return 1;
}


long
GNESelectorFrame::onCmdClear(FXObject*, FXSelector, void*) {
    myViewNet->getUndoList()->add(new GNEChange_Selection(std::set<GUIGlID>(), gSelected.getSelected(), true), true);
    myViewNet->update();
    return 1;
}


long
GNESelectorFrame::onCmdInvert(FXObject*, FXSelector, void*) {
    std::set<GUIGlID> ids = myViewNet->getNet()->getGlIDs(GLO_JUNCTION);
    for (std::set<GUIGlID>::const_iterator it = ids.begin(); it != ids.end(); it++) {
        gSelected.toggleSelection(*it);
    }
    ids = myViewNet->getNet()->getGlIDs(myViewNet->selectEdges() ? GLO_EDGE : GLO_LANE);
    for (std::set<GUIGlID>::const_iterator it = ids.begin(); it != ids.end(); it++) {
        gSelected.toggleSelection(*it);
    }
    ids = myViewNet->getNet()->getGlIDs(GLO_ADDITIONAL);
    for (std::set<GUIGlID>::const_iterator it = ids.begin(); it != ids.end(); it++) {
        gSelected.toggleSelection(*it);
    }
    myViewNet->update();
    return 1;
}


long
GNESelectorFrame::onCmdSelMBTag(FXObject*, FXSelector, void*) {
    const std::vector<SumoXMLTag>& tags = GNEAttributeCarrier::allowedTags();
    SumoXMLTag tag = tags[myMatchTagBox->getCurrentItem()];
    myMatchAttrBox->clearItems();
    const std::vector<std::pair <SumoXMLAttr, std::string> >& attrs = GNEAttributeCarrier::allowedAttributes(tag);
    for (std::vector<std::pair <SumoXMLAttr, std::string> >::const_iterator it = attrs.begin(); it != attrs.end(); it++) {
        myMatchAttrBox->appendItem(toString(it->first).c_str());
    }

    // @ToDo: Here can be placed a butto to set the default value
    myMatchAttrBox->setNumVisible(myMatchAttrBox->getNumItems());
    update();
    return 1;
}


long
GNESelectorFrame::onCmdSelMBString(FXObject*, FXSelector, void*) {
    const std::vector<SumoXMLTag>& tags = GNEAttributeCarrier::allowedTags();
    SumoXMLTag tag = tags[myMatchTagBox->getCurrentItem()];
    const std::vector<std::pair <SumoXMLAttr, std::string> >& attrs = GNEAttributeCarrier::allowedAttributes(tag);
    SumoXMLAttr attr = attrs.at(myMatchAttrBox->getCurrentItem()).first;
    std::string expr(myMatchString->getText().text());
    bool valid = true;

    if (expr == "") {
        // the empty expression matches all objects
        handleIDs(getMatches(tag, attr, '@', 0, expr), false);
    } else if (GNEAttributeCarrier::isNumerical(attr)) {
        // The expression must have the form
        //  <val matches if attr < val
        //  >val matches if attr > val
        //  =val matches if attr = val
        //  val matches if attr = val
        char compOp = expr[0];
        if (compOp == '<' || compOp == '>' || compOp == '=') {
            expr = expr.substr(1);
        } else {
            compOp = '=';
        }
        SUMOReal val;
        std::istringstream buf(expr);
        buf >> val;
        if (!buf.fail() && (int)buf.tellg() == (int)expr.size()) {
            handleIDs(getMatches(tag, attr, compOp, val, expr), false);
        } else {
            valid = false;
        }
    } else {
        // The expression must have the form
        //   =str: matches if <str> is an exact match
        //   !str: matches if <str> is not a substring
        //   ^str: matches if <str> is not an exact match
        //   str: matches if <str> is a substring (sends compOp '@')
        // Alternatively, if the expression is empty it matches all objects
        char compOp = expr[0];
        if (compOp == '=' || compOp == '!' || compOp == '^') {
            expr = expr.substr(1);
        } else {
            compOp = '@';
        }
        handleIDs(getMatches(tag, attr, compOp, 0, expr), false);
    }
    if (valid) {
        myMatchString->setTextColor(FXRGB(0, 0, 0));
        myMatchString->killFocus();
    } else {
        myMatchString->setTextColor(FXRGB(255, 0, 0));
    }

    return 1;
}


long
GNESelectorFrame::onCmdHelp(FXObject*, FXSelector, void*) {
    FXDialogBox* helpDialog = new FXDialogBox(this, "Match Attribute Help", DECOR_CLOSE | DECOR_TITLE);
    std::ostringstream help;
    help
            << "The 'Match Attribute' controls allow to specify a set of objects which are then applied to the current selection "
            << "according to the current 'Modification Mode'.\n"
            << "1. Select an object type from the first input box\n"
            << "2. Select an attribute from the second input box\n"
            << "3. Enter a 'match expression' in the third input box and press <return>\n"
            << "\n"
            << "The empty expression matches all objects\n"
            << "For numerical attributes the match expression must consist of a comparison operator ('<', '>', '=') and a number.\n"
            << "An object matches if the comparison between its attribute and the given number by the given operator evaluates to 'true'\n"
            << "\n"
            << "For string attributes the match expression must consist of a comparison operator ('', '=', '!', '^') and a string.\n"
            << "  '' (no operator) matches if string is a substring of that object'ts attribute.\n"
            << "  '=' matches if string is an exact match.\n"
            << "  '!' matches if string is not a substring.\n"
            << "  '^' matches if string is not an exact match.\n"
            << "\n"
            << "Examples:\n"
            << "junction; id; 'foo' -> match all junctions that have 'foo' in their id\n"
            << "junction; type; '=priority' -> match all junctions of type 'priority', but not of type 'priority_stop'\n"
            << "edge; speed; '>10' -> match all edges with a speed above 10\n";
    new FXLabel(helpDialog, help.str().c_str(), 0, JUSTIFY_LEFT);
    // "OK"
    new FXButton(helpDialog, "OK\t\tSave modifications", 0, helpDialog, FXDialogBox::ID_ACCEPT,
                 ICON_BEFORE_TEXT | LAYOUT_FILL_X | FRAME_THICK | FRAME_RAISED,
                 0, 0, 0, 0, 4, 4, 3, 3);
    helpDialog->create();
    helpDialog->show();
    return 1;
}


long
GNESelectorFrame::onCmdScaleSelection(FXObject*, FXSelector, void*) {
    myViewNet->setSelectionScaling(mySelectionScaling->getValue());
    myViewNet->update();
    return 1;
}


void
GNESelectorFrame::show() {
    gSelected.add2Update(this);
    selectionUpdated(); // selection may have changed due to deletions
    FXScrollWindow::show();
    // Show and Frame Area in which this GNEFrame is placed
    myViewNet->getViewParent()->showFramesArea();
}


void
GNESelectorFrame::hide() {
    gSelected.remove2Update();
    FXScrollWindow::hide();
    // Hide Frame Area in which this GNEFrame is placed
    myViewNet->getViewParent()->hideFramesArea();
}


std::string
GNESelectorFrame::getStats() const {
    return "Selection:\n" +
           toString(gSelected.getSelected(GLO_JUNCTION).size()) + " Junctions\n" +
           toString(gSelected.getSelected(GLO_EDGE).size()) + " Edges\n" +
           toString(gSelected.getSelected(GLO_LANE).size()) + " Lanes\n" +
           toString(gSelected.getSelected(GLO_ADDITIONAL).size()) + " Additionals\n";
}


void
GNESelectorFrame::selectionUpdated() {
    myFrameHeaderLabel->setText(getStats().c_str());
    update();
}


void
GNESelectorFrame::handleIDs(std::vector<GUIGlID> ids, bool selectEdges, SetOperation setop) {
    const SetOperation setOperation = (setop == SET_DEFAULT ? (SetOperation)mySetOperation : setop);
    std::set<GUIGlID> previousSelection;
    myViewNet->getUndoList()->p_begin("change selection");
    if (setOperation == SET_REPLACE) {
        myViewNet->getUndoList()->add(new GNEChange_Selection(std::set<GUIGlID>(), gSelected.getSelected(), true), true);
    } else if (setOperation == SET_RESTRICT) {
        previousSelection = gSelected.getSelected(); // have to make a copy
        myViewNet->getUndoList()->add(new GNEChange_Selection(std::set<GUIGlID>(), gSelected.getSelected(), true), true);
    }
    // handle ids
    GUIGlObject* object;
    GUIGlObjectType type;
    std::set<GUIGlID> idsSet(ids.begin(), ids.end());
    std::set<GUIGlID> selected;
    std::set<GUIGlID> deselected;
    if (myViewNet->autoSelectNodes()) {
        for (std::vector<GUIGlID>::const_iterator it = ids.begin(); it != ids.end(); it++) {
            GUIGlID id = *it;
            if (id > 0) { // net object?
                object = GUIGlObjectStorage::gIDStorage.getObjectBlocking(id);
                if (object->getType() == GLO_LANE && selectEdges) {
                    const GNEEdge& edge = (static_cast<GNELane*>(object))->getParentEdge();
                    idsSet.insert(edge.getSource()->getGlID());
                    idsSet.insert(edge.getDest()->getGlID());
                }
                GUIGlObjectStorage::gIDStorage.unblockObject(id);
            }
        }
    }
    for (std::set<GUIGlID>::const_iterator it = idsSet.begin(); it != idsSet.end(); it++) {
        GUIGlID id = *it;
        if (id > 0) { // net object?
            object = GUIGlObjectStorage::gIDStorage.getObjectBlocking(id);
            if (object == 0) {
                // in debug mode we would like to know about this.
                // It might be caused by a corrupted gl-name stack.
                // However, most cases of uninitizliaed values would go hidden since 0 is assumed to be the net object anyway
                assert(false);
                continue;
            }
            type = object->getType();
            GUIGlObjectStorage::gIDStorage.unblockObject(id);
            if (type == GLO_LANE && selectEdges) {
                // @note edge may be selected/deselected multiple times but this shouldn't
                // hurt unless we add SET_TOGGLE
                id = (static_cast<GNELane*>(object))->getParentEdge().getGlID();
            }
            // doing the switch outside the loop requires functional techniques. this was deemed to ugly
            switch (setOperation) {
                case GNESelectorFrame::SET_ADD:
                case GNESelectorFrame::SET_REPLACE:
                    selected.insert(id);
                    break;
                case GNESelectorFrame::SET_SUB:
                    deselected.insert(id);
                    break;
                case GNESelectorFrame::SET_RESTRICT:
                    if (previousSelection.count(id)) {
                        selected.insert(id);
                    }
                    break;
                default:
                    break;
            }
        }
    }
    myViewNet->getUndoList()->add(new GNEChange_Selection(selected, deselected, true), true);
    myViewNet->getUndoList()->p_end();
    myViewNet->update();
}


std::vector<GUIGlID>
GNESelectorFrame::getMatches(SumoXMLTag tag, SumoXMLAttr attr, char compOp, SUMOReal val, const std::string& expr) {
    GUIGlObject* object;
    GNEAttributeCarrier* ac;
    std::vector<GUIGlID> result;
    const std::set<GUIGlID> allIDs = myViewNet->getNet()->getGlIDs();
    const bool numerical = GNEAttributeCarrier::isNumerical(attr);
    for (std::set<GUIGlID>::const_iterator it = allIDs.begin(); it != allIDs.end(); it++) {
        GUIGlID id = *it;
        object = GUIGlObjectStorage::gIDStorage.getObjectBlocking(id);
        if (!object) {
            throw ProcessError("Unkown object passed to GNESelectorFrame::getMatches (id=" + toString(id) + ").");
        }
        ac = dynamic_cast<GNEAttributeCarrier*>(object);
        if (ac && ac->getTag() == tag) { // not all objects need to be attribute carriers
            if (expr == "") {
                result.push_back(id);
            } else if (numerical) {
                SUMOReal acVal;
                std::istringstream buf(ac->getAttribute(attr));
                buf >> acVal;
                switch (compOp) {
                    case '<':
                        if (acVal < val) {
                            result.push_back(id);
                        }
                        break;
                    case '>':
                        if (acVal > val) {
                            result.push_back(id);
                        }
                        break;
                    case '=':
                        if (acVal == val) {
                            result.push_back(id);
                        }
                        break;
                }
            } else {
                // string match
                std::string acVal = ac->getAttribute(attr);
                if ((attr == SUMO_ATTR_ALLOW || attr == SUMO_ATTR_DISALLOW) && acVal == "all") {
                    acVal = ALL_VCLASS_NAMES_MATCH_STRING;
                }
                switch (compOp) {
                    case '@':
                        if (acVal.find(expr) != std::string::npos) {
                            result.push_back(id);
                        }
                        break;
                    case '!':
                        if (acVal.find(expr) == std::string::npos) {
                            result.push_back(id);
                        }
                        break;
                    case '=':
                        if (acVal == expr) {
                            result.push_back(id);
                        }
                        break;
                    case '^':
                        if (acVal != expr) {
                            result.push_back(id);
                        }
                        break;
                }
            }
        }
        GUIGlObjectStorage::gIDStorage.unblockObject(id);
    }
    return result;
}

/****************************************************************************/
