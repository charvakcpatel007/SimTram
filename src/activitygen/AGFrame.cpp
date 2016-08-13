/****************************************************************************/
/// @file    AGFrame.cpp
/// @author  Walter Bamberger
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mo, 13 Sept 2010
/// @version $Id: AGFrame.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// Configuration of the options of ActivityGen
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2001-2016 DLR (http://www.dlr.de/) and contributors
// activitygen module
// Copyright 2010 TUM (Technische Universitaet Muenchen, http://www.tum.de/)
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

#include "AGFrame.h"
#include <utils/common/StdDefs.h>
#include <router/ROFrame.h>
#include <duarouter/RODUAFrame.h>
#include <utils/common/SystemFrame.h>
#include <utils/common/RandHelper.h>
#include <utils/options/OptionsCont.h>


// ===========================================================================
// method definitions
// ===========================================================================
void AGFrame::fillOptions() {
    OptionsCont& oc = OptionsCont::getOptions();
    // Options handling
    oc.addCallExample("--net-file <INPUT>.net.xml --stat-file <INPUT>.stat.xml --output <OUTPUT>.rou.xml --rand",
                      "generate a trips file from a stats file on a given net using arbitrary random seed");
    oc.addCallExample("--net-file <INPUT>.net.xml --stat-file <INPUT>.stat.xml --output <OUTPUT>.rou.xml --duration-d <NBR_OF_DAYS>",
                      "generate a trips file from a stats file on a given net for numerous days (with fixed random seed)");

    // Add categories and insert the standard options
    SystemFrame::addConfigurationOptions(oc);
    oc.addOptionSubTopic("Input");
    oc.addOptionSubTopic("Output");
    oc.addOptionSubTopic("Processing");
    oc.addOptionSubTopic("Time");
    SystemFrame::addReportOptions(oc);
    RandHelper::insertRandOptions();

    // Insert options
    oc.doRegister("net-file", 'n', new Option_FileName());
    oc.addSynonyme("net-file", "net");
    oc.addDescription("net-file", "Input", "Use FILE as SUMO-network to create trips for");

    oc.doRegister("stat-file", 's', new Option_FileName());
    oc.addDescription("stat-file", "Input", "Loads the SUMO-statistics FILE");

    oc.doRegister("output-file", 'o', new Option_FileName());
    oc.addSynonyme("output-file", "output", true);
    oc.addDescription("output-file", "Output", "Write generated trips to FILE");

    oc.doRegister("debug", new Option_Bool(false));
    oc.addDescription("debug", "Report",
                      "Detailed messages about every single step");

    // TODO: What time options are consistent with other parts of SUMO and
    // useful for the user?
    oc.doRegister("begin", 'b', new Option_Integer());
    oc.addDescription("begin", "Time", "Sets the time of beginning of the simulation during the first day (in seconds)");

    oc.doRegister("end", 'e', new Option_Integer());
    oc.addDescription("end", "Time", "Sets the time of ending of the simulation during the last day (in seconds)");

    oc.doRegister("duration-d", new Option_Integer());
    oc.addDescription("duration-d", "Time", "Sets the duration of the simulation in days");
}


bool AGFrame::checkOptions() {
    return true;
}
