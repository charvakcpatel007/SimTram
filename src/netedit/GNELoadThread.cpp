/****************************************************************************/
/// @file    GNELoadThread.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
/// @version $Id: GNELoadThread.cpp 21239 2016-07-26 09:33:31Z namdre $
///
// The thread that performs the loading of a Netedit-net (adapted from
// GUILoadThread)
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
#include <utils/xml/XMLSubSys.h>
#include <utils/gui/events/GUIEvent_Message.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/common/RandHelper.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/MsgRetrievingFunction.h>
#include <utils/common/SystemFrame.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/options/OptionsIO.h>
#include <utils/geom/GeoConvHelper.h>
#include <netbuild/NBFrame.h>
#include <netimport/NILoader.h>
#include <netimport/NIFrame.h>
#include <netwrite/NWFrame.h>
#include <netbuild/NBFrame.h>

#include "GNELoadThread.h"
#include "GNENet.h"
#include "GNEEvent_NetworkLoaded.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
GNELoadThread::GNELoadThread(FXApp* app, MFXInterThreadEventClient* mw, MFXEventQue<GUIEvent*>& eq, FXEX::FXThreadEvent& ev) :
    FXSingleEventThread(app, mw), myParent(mw), myEventQue(eq),
    myEventThrow(ev) {
    myErrorRetriever = new MsgRetrievingFunction<GNELoadThread>(this, &GNELoadThread::retrieveMessage, MsgHandler::MT_ERROR);
    myMessageRetriever = new MsgRetrievingFunction<GNELoadThread>(this, &GNELoadThread::retrieveMessage, MsgHandler::MT_MESSAGE);
    myWarningRetriever = new MsgRetrievingFunction<GNELoadThread>(this, &GNELoadThread::retrieveMessage, MsgHandler::MT_WARNING);
    MsgHandler::getErrorInstance()->addRetriever(myErrorRetriever);
}


GNELoadThread::~GNELoadThread() {
    delete myErrorRetriever;
    delete myMessageRetriever;
    delete myWarningRetriever;
}


FXint
GNELoadThread::run() {
    // register message callbacks
    MsgHandler::getMessageInstance()->addRetriever(myMessageRetriever);
    MsgHandler::getErrorInstance()->addRetriever(myErrorRetriever);
    MsgHandler::getWarningInstance()->addRetriever(myWarningRetriever);

    GNENet* net = 0;

    // try to load the given configuration
    OptionsCont& oc = OptionsCont::getOptions();
    oc.clear();
    if (!initOptions()) {
        submitEndAndCleanup(net);
        return 0;
    }
    MsgHandler::initOutputOptions();
    if (!(NIFrame::checkOptions() &&
            NBFrame::checkOptions() &&
            NWFrame::checkOptions())) {
        // options are not valid
        WRITE_ERROR("Invalid Options. Nothing loaded");
        submitEndAndCleanup(net);
        return 0;
    }
    MsgHandler::getErrorInstance()->clear();
    MsgHandler::getWarningInstance()->clear();
    MsgHandler::getMessageInstance()->clear();

    RandHelper::initRandGlobal();
    if (!GeoConvHelper::init(oc)) {
        WRITE_ERROR("Could not build projection!");
        submitEndAndCleanup(net);
        return 0;
    }
    XMLSubSys::setValidation(oc.getString("xml-validation"), oc.getString("xml-validation.net"));
    // this netbuilder instance becomes the responsibility of the GNENet
    NBNetBuilder* netBuilder = new NBNetBuilder();

    netBuilder->applyOptions(oc);

    if (myNewNet) {
        // create new network
        net = new GNENet(netBuilder);
    } else {
        NILoader nl(*netBuilder);
        try {
            nl.load(oc);

            if (!myLoadNet) {
                WRITE_MESSAGE("Performing initial computation ...\n");
                // perform one-time processing (i.e. edge removal)
                netBuilder->compute(oc);
                // @todo remove one-time processing options!
            } else {
                // make coordinate conversion usable before first netBuilder->compute()
                GeoConvHelper::computeFinal();
            }

            if (oc.getBool("ignore-errors")) {
                MsgHandler::getErrorInstance()->clear();
            }
            // check whether any errors occured
            if (MsgHandler::getErrorInstance()->wasInformed()) {
                throw ProcessError();
            } else {
                net = new GNENet(netBuilder);
            }
        } catch (ProcessError& e) {
            if (std::string(e.what()) != std::string("Process Error") && std::string(e.what()) != std::string("")) {
                WRITE_ERROR(e.what());
            }
            WRITE_ERROR("Failed to build network.");
            delete net;
            delete netBuilder;
            net = 0;
#ifndef _DEBUG
        } catch (std::exception& e) {
            WRITE_ERROR(e.what());
            delete net;
            delete netBuilder;
            net = 0;
#endif
        }
    }
    // only a single setting file is supported
    submitEndAndCleanup(net, oc.getString("gui-settings-file"), oc.getBool("registry-viewport"));
    return 0;
}



void
GNELoadThread::submitEndAndCleanup(GNENet* net, const std::string& guiSettingsFile, const bool viewportFromRegistry) {
    // remove message callbacks
    MsgHandler::getErrorInstance()->removeRetriever(myErrorRetriever);
    MsgHandler::getWarningInstance()->removeRetriever(myWarningRetriever);
    MsgHandler::getMessageInstance()->removeRetriever(myMessageRetriever);
    // inform parent about the process
    GUIEvent* e = new GNEEvent_NetworkLoaded(net, myFile, guiSettingsFile, viewportFromRegistry);
    myEventQue.add(e);
    myEventThrow.signal();
}


void
GNELoadThread::fillOptions(OptionsCont& oc) {
    oc.clear();
    oc.addCallExample("", "start plain GUI with empty net");
    oc.addCallExample("-c <CONFIGURATION>", "edit net with options read from file");

    SystemFrame::addConfigurationOptions(oc); // this subtopic is filled here, too
    oc.addOptionSubTopic("Input");
    oc.addOptionSubTopic("Output");
    GeoConvHelper::addProjectionOptions(oc);
    oc.addOptionSubTopic("TLS Building");
    oc.addOptionSubTopic("Ramp Guessing");
    oc.addOptionSubTopic("Edge Removal");
    oc.addOptionSubTopic("Unregulated Nodes");
    oc.addOptionSubTopic("Processing");
    oc.addOptionSubTopic("Building Defaults");
    oc.addOptionSubTopic("Visualisation");

    oc.doRegister("disable-textures", 'T', new Option_Bool(false)); // !!!
    oc.addDescription("disable-textures", "Visualisation", "");

    oc.doRegister("gui-settings-file", new Option_FileName());
    oc.addDescription("gui-settings-file", "Visualisation", "Load visualisation settings from FILE");

    oc.doRegister("registry-viewport", new Option_Bool(false));
    oc.addDescription("registry-viewport", "Visualisation", "Load current viewport from registry");

    SystemFrame::addReportOptions(oc); // this subtopic is filled here, too

    NIFrame::fillOptions();
    NBFrame::fillOptions(false);
    NWFrame::fillOptions(false);
    RandHelper::insertRandOptions();
}


void
GNELoadThread::setDefaultOptions(OptionsCont& oc) {
    oc.set("offset.disable-normalization", "true"); // preserve the given network as far as possible
    oc.set("no-turnarounds", "true"); // otherwise it is impossible to manually removed turn-arounds
}


bool
GNELoadThread::initOptions() {
    OptionsCont& oc = OptionsCont::getOptions();
    fillOptions(oc);
    if (myFile != "") {
        if (myLoadNet) {
            oc.set("sumo-net-file", myFile);
        } else {
            oc.set("configuration-file", myFile);
        }
    }
    setDefaultOptions(oc);
    try {
        OptionsIO::getOptions();
        if (!oc.isSet("output-file")) {
            oc.set("output-file", oc.getString("sumo-net-file"));
        }
        return true;
    } catch (ProcessError& e) {
        if (std::string(e.what()) != std::string("Process Error") && std::string(e.what()) != std::string("")) {
            WRITE_ERROR(e.what());
        }
        WRITE_ERROR("Failed to parse options.");
    }
    return false;
}


void
GNELoadThread::loadConfigOrNet(const std::string& file, bool isNet, bool optionsReady, bool newNet) {
    myFile = file;
    myLoadNet = isNet;
    if (myFile != "") {
        OptionsIO::setArgs(0, 0);
    }
    myOptionsReady = optionsReady;
    myNewNet = newNet;
    start();
}


void
GNELoadThread::retrieveMessage(const MsgHandler::MsgType type, const std::string& msg) {
    GUIEvent* e = new GUIEvent_Message(type, msg);
    myEventQue.add(e);
    myEventThrow.signal();
}

/****************************************************************************/

