/****************************************************************************/
/// @file    TraCIServerAPI_GUI.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    07.05.2009
/// @version $Id: TraCIServerAPI_GUI.cpp 21186 2016-07-18 12:04:16Z namdre $
///
// APIs for getting/setting GUI values via TraCI
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

#ifndef NO_TRACI

#include <fx.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/gui/windows/GUIGlChildWindow.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIPerspectiveChanger.h>
#include <utils/foxtools/MFXImageHelper.h>
#include <traci-server/TraCIConstants.h>
#include <guisim/GUINet.h>
#include <guisim/GUIVehicle.h>
#include <guisim/GUIBaseVehicle.h>
#include "GUIEvent_Screenshot.h"
#include "TraCIServerAPI_GUI.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
bool
TraCIServerAPI_GUI::processGet(TraCIServer& server, tcpip::Storage& inputStorage,
                               tcpip::Storage& outputStorage) {
    // variable & id
    int variable = inputStorage.readUnsignedByte();
    std::string id = inputStorage.readString();
    // check variable
    if (variable != ID_LIST && variable != VAR_VIEW_ZOOM && variable != VAR_VIEW_OFFSET
            && variable != VAR_VIEW_SCHEMA && variable != VAR_VIEW_BOUNDARY) {
        return server.writeErrorStatusCmd(CMD_GET_GUI_VARIABLE, "Get GUI Variable: unsupported variable " + toHex(variable, 2) + " specified", outputStorage);
    }
    // begin response building
    tcpip::Storage tempMsg;
    //  response-code, variableID, objectID
    tempMsg.writeUnsignedByte(RESPONSE_GET_GUI_VARIABLE);
    tempMsg.writeUnsignedByte(variable);
    tempMsg.writeString(id);
    // process request
    if (variable == ID_LIST) {
        std::vector<std::string> ids = GUIMainWindow::getInstance()->getViewIDs();
        tempMsg.writeUnsignedByte(TYPE_STRINGLIST);
        tempMsg.writeStringList(ids);
    } else {
        GUISUMOAbstractView* v = getNamedView(id);
        if (v == 0) {
            return server.writeErrorStatusCmd(CMD_GET_GUI_VARIABLE, "View '" + id + "' is not known", outputStorage);
        }
        switch (variable) {
            case VAR_VIEW_ZOOM:
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(v->getChanger().getZoom());
                break;
            case VAR_VIEW_OFFSET:
                tempMsg.writeUnsignedByte(POSITION_2D);
                tempMsg.writeDouble(v->getChanger().getXPos());
                tempMsg.writeDouble(v->getChanger().getYPos());
                break;
            case VAR_VIEW_SCHEMA: {
                tempMsg.writeUnsignedByte(TYPE_STRING);
                tempMsg.writeString(v->getVisualisationSettings()->name);
                break;
            }
            case VAR_VIEW_BOUNDARY: {
                tempMsg.writeUnsignedByte(TYPE_BOUNDINGBOX);
                Boundary b = v->getVisibleBoundary();
                tempMsg.writeDouble(b.xmin());
                tempMsg.writeDouble(b.ymin());
                tempMsg.writeDouble(b.xmax());
                tempMsg.writeDouble(b.ymax());
                break;
            }
            default:
                break;
        }
    }
    server.writeStatusCmd(CMD_GET_GUI_VARIABLE, RTYPE_OK, "", outputStorage);
    server.writeResponseWithLength(outputStorage, tempMsg);
    return true;
}


bool
TraCIServerAPI_GUI::processSet(TraCIServer& server, tcpip::Storage& inputStorage,
                               tcpip::Storage& outputStorage) {
    std::string warning = ""; // additional description for response
    // variable
    int variable = inputStorage.readUnsignedByte();
    if (variable != VAR_VIEW_ZOOM && variable != VAR_VIEW_OFFSET && variable != VAR_VIEW_SCHEMA && variable != VAR_VIEW_BOUNDARY
            && variable != VAR_SCREENSHOT && variable != VAR_TRACK_VEHICLE
       ) {
        return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Change GUI State: unsupported variable " + toHex(variable, 2) + " specified", outputStorage);
    }
    // id
    std::string id = inputStorage.readString();
    GUISUMOAbstractView* v = getNamedView(id);
    if (v == 0) {
        return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "View '" + id + "' is not known", outputStorage);
    }
    // process
    switch (variable) {
        case VAR_VIEW_ZOOM: {
            Position off, p;
            double zoom = 1;
            if (!server.readTypeCheckingDouble(inputStorage, zoom)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The zoom must be given as a double.", outputStorage);
            }
            off.set(v->getChanger().getXPos(), v->getChanger().getYPos(), v->getChanger().zoom2ZPos(zoom));
            p.set(off.x(), off.y(), 0);
            v->setViewportFromTo(off, p);
        }
        break;
        case VAR_VIEW_OFFSET: {
            Position off, p;
            if (!server.readTypeCheckingPosition2D(inputStorage, off)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The view port must be given as a position.", outputStorage);
            }
            off.set(off.x(), off.y(), v->getChanger().getZPos());
            p.set(off.x(), off.y(), 0);
            v->setViewportFromTo(off, p);
        }
        break;
        case VAR_VIEW_SCHEMA: {
            std::string schema;
            if (!server.readTypeCheckingString(inputStorage, schema)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The scheme must be specified by a string.", outputStorage);
            }
            if (!v->setColorScheme(schema)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The scheme is not known.", outputStorage);
            }
        }
        break;
        case VAR_VIEW_BOUNDARY: {
            Boundary b;
            if (!server.readTypeCheckingBoundary(inputStorage, b)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "The boundary must be specified by a bounding box.", outputStorage);
            }
            v->centerTo(b);
            break;
        }
        case VAR_SCREENSHOT: {
            std::string filename;
            if (!server.readTypeCheckingString(inputStorage, filename)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Making a snapshot requires a file name.", outputStorage);
            }
            GUIMainWindow::getInstance()->sendBlockingEvent(new GUIEvent_Screenshot(v, filename));
        }
        break;
        case VAR_TRACK_VEHICLE: {
            std::string id;
            if (!server.readTypeCheckingString(inputStorage, id)) {
                return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Tracking requires a string vehicle ID.", outputStorage);
            }
            if (id == "") {
                v->stopTrack();
            } else {
                SUMOVehicle* veh = MSNet::getInstance()->getVehicleControl().getVehicle(id);
                if (veh == 0) {
                    return server.writeErrorStatusCmd(CMD_SET_GUI_VARIABLE, "Could not find vehicle '" + id + "'.", outputStorage);
                }
                if (!static_cast<GUIVehicle*>(veh)->hasActiveAddVisualisation(v, GUIBaseVehicle::VO_TRACKED)) {
                    v->startTrack(static_cast<GUIVehicle*>(veh)->getGlID());
                    static_cast<GUIVehicle*>(veh)->addActiveAddVisualisation(v, GUIBaseVehicle::VO_TRACKED);
                }
            }
        }
        default:
            break;
    }
    server.writeStatusCmd(CMD_SET_GUI_VARIABLE, RTYPE_OK, warning, outputStorage);
    return true;
}


GUISUMOAbstractView*
TraCIServerAPI_GUI::getNamedView(const std::string& id) {
    GUIMainWindow* const mw = GUIMainWindow::getInstance();
    if (mw == 0) {
        return 0;
    }
    GUIGlChildWindow* const c = static_cast<GUIGlChildWindow*>(mw->getViewByID(id));
    if (c == 0) {
        return 0;
    }
    return c->getView();
}


#endif


/****************************************************************************/

