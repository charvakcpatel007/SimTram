/****************************************************************************/
/// @file    GUILoadThread.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: GUILoadThread.cpp 21010 2016-06-20 13:32:41Z behrisch $
///
// Class describing the thread that performs the loading of a simulation
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
#include <ctime>
#include <utils/common/RandHelper.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/MsgRetrievingFunction.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/options/OptionsIO.h>
#include <utils/foxtools/MFXEventQue.h>
#include <utils/gui/events/GUIEvent_Message.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/xml/XMLSubSys.h>
#include <guisim/GUINet.h>
#include <guisim/GUIEventControl.h>
#include <guisim/GUIVehicleControl.h>
#include <netload/NLBuilder.h>
#include <netload/NLHandler.h>
#include <netload/NLJunctionControlBuilder.h>
#include <guinetload/GUIEdgeControlBuilder.h>
#include <guinetload/GUIDetectorBuilder.h>
#include <guinetload/GUITriggerBuilder.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSFrame.h>
#include <microsim/MSRouteHandler.h>
#include "GUIApplicationWindow.h"
#include "GUILoadThread.h"
#include "GUIGlobals.h"
#include "GUIEvent_SimulationLoaded.h"

#include <mesogui/GUIMEVehicleControl.h>

#ifndef NO_TRACI
#include <traci-server/TraCIServer.h>
#include "TraCIServerAPI_GUI.h"
#endif

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
GUILoadThread::GUILoadThread(FXApp* app, GUIApplicationWindow* mw,
                             MFXEventQue<GUIEvent*>& eq, FXEX::FXThreadEvent& ev)
    : FXSingleEventThread(app, mw), myParent(mw), myEventQue(eq),
      myEventThrow(ev) {
    myErrorRetriever = new MsgRetrievingFunction<GUILoadThread>(this, &GUILoadThread::retrieveMessage, MsgHandler::MT_ERROR);
    myMessageRetriever = new MsgRetrievingFunction<GUILoadThread>(this, &GUILoadThread::retrieveMessage, MsgHandler::MT_MESSAGE);
    myWarningRetriever = new MsgRetrievingFunction<GUILoadThread>(this, &GUILoadThread::retrieveMessage, MsgHandler::MT_WARNING);
    MsgHandler::getErrorInstance()->addRetriever(myErrorRetriever);
}


GUILoadThread::~GUILoadThread() {
    delete myErrorRetriever;
    delete myMessageRetriever;
    delete myWarningRetriever;
}


FXint
GUILoadThread::run() {
    // register message callbacks
    MsgHandler::getMessageInstance()->addRetriever(myMessageRetriever);
    MsgHandler::getErrorInstance()->addRetriever(myErrorRetriever);
    MsgHandler::getWarningInstance()->addRetriever(myWarningRetriever);

    // try to load the given configuration
    OptionsCont& oc = OptionsCont::getOptions();
    try {
        oc.clear();
        MSFrame::fillOptions();
        if (myFile != "") {
            // triggered by menu option or reload
            if (myLoadNet) {
                oc.set("net-file", myFile);
            } else {
                oc.set("configuration-file", myFile);
            }
            oc.resetWritable(); // there may be command line options
            OptionsIO::getOptions();
        } else {
            // triggered at application start
            OptionsIO::getOptions();
            if (oc.isSet("configuration-file")) {
                myFile = oc.getString("configuration-file");
            } else if (oc.isSet("net-file")) {
                myFile = oc.getString("net-file");
                myLoadNet = true;
            }
            myEventQue.add(new GUIEvent_Message("Loading '" + myFile + "'."));
            myEventThrow.signal();
            myParent->addRecentFile(FXPath::absolute(myFile.c_str()), myLoadNet);
        }
        myTitle = myFile;
        // within gui-based applications, nothing is reported to the console
        MsgHandler::getMessageInstance()->removeRetriever(&OutputDevice::getDevice("stdout"));
        MsgHandler::getWarningInstance()->removeRetriever(&OutputDevice::getDevice("stderr"));
        MsgHandler::getErrorInstance()->removeRetriever(&OutputDevice::getDevice("stderr"));
        // do this once again to get parsed options
        if (oc.getBool("duration-log.statistics") && oc.isDefault("verbose")) {
            // must be done before calling initOutputOptions (which checks option "verbose")
            // but initOutputOptions must come before checkOptions (so that warnings are printed)
            oc.set("verbose", "true");
        }
        MsgHandler::initOutputOptions();
        if (!MSFrame::checkOptions()) {
            throw ProcessError();
        }
        XMLSubSys::setValidation(oc.getString("xml-validation"), oc.getString("xml-validation.net"));
        GUIGlobals::gRunAfterLoad = oc.getBool("start");
        GUIGlobals::gQuitOnEnd = oc.getBool("quit-on-end");
        GUIGlobals::gDemoAutoReload = oc.getBool("demo");
    } catch (ProcessError& e) {
        if (std::string(e.what()) != std::string("Process Error") && std::string(e.what()) != std::string("")) {
            WRITE_ERROR(e.what());
        }
        // the options are not valid but maybe we want to quit
        GUIGlobals::gQuitOnEnd = oc.getBool("quit-on-end");
        MsgHandler::getErrorInstance()->inform("Quitting (on error).", false);
        submitEndAndCleanup(0, 0, 0);
        return 0;
    }

    // initialise global settings
    RandHelper::initRandGlobal();
    RandHelper::initRandGlobal(MSRouteHandler::getParsingRNG());
    MSFrame::setMSGlobals(oc);
    GUITexturesHelper::allowTextures(!oc.getBool("disable-textures"));
    if (oc.getBool("game")) {
        myParent->onCmdGaming(0, 0, 0);
    }
    MSVehicleControl* vehControl = 0;
    GUIVisualizationSettings::UseMesoSim = MSGlobals::gUseMesoSim;
    if (MSGlobals::gUseMesoSim) {
        vehControl = new GUIMEVehicleControl();
    } else {
        vehControl = new GUIVehicleControl();
    }

    GUINet* net = 0;
    int simStartTime = 0;
    int simEndTime = 0;
    std::vector<std::string> guiSettingsFiles;
    bool osgView = false;
    GUIEdgeControlBuilder* eb = 0;
    try {
        net = new GUINet(
            vehControl,
            new GUIEventControl(),
            new GUIEventControl(),
            new GUIEventControl());
#ifndef NO_TRACI
        // need to init TraCI-Server before loading routes to catch VEHICLE_STATE_BUILT
        std::map<int, TraCIServer::CmdExecutor> execs;
        execs[CMD_GET_GUI_VARIABLE] = &TraCIServerAPI_GUI::processGet;
        execs[CMD_SET_GUI_VARIABLE] = &TraCIServerAPI_GUI::processSet;
        TraCIServer::openSocket(execs);
#endif

        eb = new GUIEdgeControlBuilder();
        GUIDetectorBuilder db(*net);
        NLJunctionControlBuilder jb(*net, db);
        GUITriggerBuilder tb;
        NLHandler handler("", *net, db, tb, *eb, jb);
        tb.setHandler(&handler);
        NLBuilder builder(oc, *net, *eb, jb, db, handler);
        MsgHandler::getErrorInstance()->clear();
        MsgHandler::getWarningInstance()->clear();
        MsgHandler::getMessageInstance()->clear();
        if (!builder.build()) {
            throw ProcessError();
        } else {
            net->initGUIStructures();
            simStartTime = string2time(oc.getString("begin"));
            simEndTime = string2time(oc.getString("end"));
            guiSettingsFiles = oc.getStringVector("gui-settings-file");
#ifdef HAVE_OSG
            osgView = oc.getBool("osg-view");
#endif
        }
    } catch (ProcessError& e) {
        if (std::string(e.what()) != std::string("Process Error") && std::string(e.what()) != std::string("")) {
            WRITE_ERROR(e.what());
        }
        MsgHandler::getErrorInstance()->inform("Quitting (on error).", false);
        delete net;
        net = 0;
#ifndef _DEBUG
    } catch (std::exception& e) {
        WRITE_ERROR(e.what());
        delete net;
        net = 0;
#endif
    }
    if (net == 0) {
        MSNet::clearAll();
    }
    delete eb;
    submitEndAndCleanup(net, simStartTime, simEndTime, guiSettingsFiles, osgView);
    return 0;
}


void
GUILoadThread::submitEndAndCleanup(GUINet* net,
                                   const SUMOTime simStartTime,
                                   const SUMOTime simEndTime,
                                   const std::vector<std::string>& guiSettingsFiles,
                                   const bool osgView) {
    // remove message callbacks
    MsgHandler::getErrorInstance()->removeRetriever(myErrorRetriever);
    MsgHandler::getWarningInstance()->removeRetriever(myWarningRetriever);
    MsgHandler::getMessageInstance()->removeRetriever(myMessageRetriever);
    // inform parent about the process
    GUIEvent* e = new GUIEvent_SimulationLoaded(net, simStartTime, simEndTime, myTitle, guiSettingsFiles, osgView);
    myEventQue.add(e);
    myEventThrow.signal();
}


void
GUILoadThread::loadConfigOrNet(const std::string& file, bool isNet) {
    myFile = file;
    myLoadNet = isNet;
    if (myFile != "") {
        OptionsIO::setArgs(0, 0);
    }
    start();
}


void
GUILoadThread::retrieveMessage(const MsgHandler::MsgType type, const std::string& msg) {
    GUIEvent* e = new GUIEvent_Message(type, msg);
    myEventQue.add(e);
    myEventThrow.signal();
}


const std::string&
GUILoadThread::getFileName() const {
    return myFile;
}


/****************************************************************************/
