/****************************************************************************/
/// @file    TrajectoriesHandler.cpp
/// @author  Michael Behrisch
/// @date    14.03.2014
/// @version $Id: TrajectoriesHandler.cpp 21131 2016-07-08 07:59:22Z behrisch $
///
// An XML-Handler for amitran and netstate trajectories
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2014-2016 DLR (http://www.dlr.de/) and contributors
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

#include <string>
#include <utility>
#include <iostream>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/geom/GeomHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include "TrajectoriesHandler.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
TrajectoriesHandler::TrajectoriesHandler(const bool computeA, const bool computeAForward,
        const bool accelZeroCorrection, const SUMOEmissionClass defaultClass,
        const SUMOReal defaultSlope, std::ostream* stdOut, OutputDevice* xmlOut)
    : SUMOSAXHandler(""), myComputeA(computeA), myComputeAForward(computeAForward), myAccelZeroCorrection(accelZeroCorrection), myDefaultClass(defaultClass),
      myDefaultSlope(defaultSlope), myStdOut(stdOut), myXMLOut(xmlOut), myCurrentTime(-1), myStepSize(TS) {}


TrajectoriesHandler::~TrajectoriesHandler() {}


void
TrajectoriesHandler::myStartElement(int element,
                                    const SUMOSAXAttributes& attrs) {
    bool ok = true;
    switch (element) {
        case SUMO_TAG_TRAJECTORIES:
            myStepSize = attrs.getFloat("timeStepSize") / 1000.;
            break;
        case SUMO_TAG_TIMESTEP:
            myCurrentTime = attrs.getSUMOTimeReporting(SUMO_ATTR_TIME, 0, ok);
            break;
        case SUMO_TAG_VEHICLE:
            if (attrs.hasAttribute(SUMO_ATTR_SPEED)) {
                SUMOReal v = attrs.getFloat(SUMO_ATTR_SPEED);
                SUMOReal a = INVALID_VALUE;
                SUMOReal s = INVALID_VALUE;
                writeEmissions(std::cout, attrs.getString(SUMO_ATTR_ID), myDefaultClass, STEPS2TIME(myCurrentTime), v, a, s);
            } else {
                const std::string acId = attrs.getString(SUMO_ATTR_ACTORCONFIG);
                const std::string id = attrs.getString(SUMO_ATTR_ID);
                if (myEmissionClassByType.count(acId) == 0) {
                    WRITE_WARNING("Unknown actor configuration '" + acId + "' for vehicle '" + id + "'!");
                } else {
                    myEmissionClassByVehicle[id] = myEmissionClassByType.count(acId) > 0 ? myEmissionClassByType[acId] : myDefaultClass;
                }
            }
            break;
        case SUMO_TAG_ACTORCONFIG: {
            const std::string id = attrs.getString(SUMO_ATTR_ID);
            const std::string vClass = attrs.getString(SUMO_ATTR_VEHICLECLASS);
            const std::string fuel = attrs.getString(SUMO_ATTR_FUEL);
            const std::string eClass = attrs.getString(SUMO_ATTR_EMISSIONCLASS);
            const SUMOReal weight = attrs.getOpt<SUMOReal>(SUMO_ATTR_WEIGHT, id.c_str(), ok, 0.) * 10.;
            myEmissionClassByType[id] = PollutantsInterface::getClass(myDefaultClass, vClass, fuel, eClass, weight);
            break;
        }
        case SUMO_TAG_MOTIONSTATE: {
            const std::string id = attrs.getString(SUMO_ATTR_VEHICLE);
            if (myEmissionClassByVehicle.count(id) == 0) {
                WRITE_WARNING("Motion state for unknown vehicle '" + id + "'!");
                myEmissionClassByVehicle[id] = myDefaultClass;
            }
            const SUMOEmissionClass c = myEmissionClassByVehicle[id];
            SUMOReal v = attrs.getFloat(SUMO_ATTR_SPEED) / 100.;
            SUMOReal a = attrs.hasAttribute(SUMO_ATTR_ACCELERATION) ? attrs.get<SUMOReal>(SUMO_ATTR_ACCELERATION, id.c_str(), ok) / 1000. : INVALID_VALUE;
            SUMOReal s = attrs.hasAttribute(SUMO_ATTR_SLOPE) ? RAD2DEG(asin(attrs.get<SUMOReal>(SUMO_ATTR_SLOPE, id.c_str(), ok) / 10000.)) : INVALID_VALUE;
            const SUMOTime time = attrs.getOpt<int>(SUMO_ATTR_TIME, id.c_str(), ok, INVALID_VALUE);
            if (myXMLOut != 0) {
                writeXMLEmissions(id, c, time, v, a, s);
            }
            if (myStdOut != 0) {
                writeEmissions(*myStdOut, id, c, STEPS2TIME(time), v, a, s);
            }
            break;
        }
        default:
            break;
    }
}


const PollutantsInterface::Emissions
TrajectoriesHandler::computeEmissions(const std::string id, const SUMOEmissionClass c,
                                      SUMOReal& v, SUMOReal& a, SUMOReal& s) {

    if (myComputeA) {
        if (myLastV.count(id) == 0) {
            a = 0.;
        } else {
            a = v - myLastV[id];
        }
        myLastV[id] = v;
        if (myComputeAForward) {
            v -= a;
        }
    }
    if (myAccelZeroCorrection && v == 0.) {
        a = 0.;
    }
    if (a == INVALID_VALUE) {
        throw ProcessError("Acceleration information is missing; try running with --compute-a.");
    }
    if (s == INVALID_VALUE) {
        s = myDefaultSlope;
    }
    const PollutantsInterface::Emissions result = PollutantsInterface::computeAll(c, v, a, s);
    mySums[id].addScaled(result, myStepSize);
    if (id != "") {
        mySums[""].addScaled(result, myStepSize);
    }
    return result;
}


bool
TrajectoriesHandler::writeEmissions(std::ostream& o, const std::string id,
                                    const SUMOEmissionClass c,
                                    SUMOReal t, SUMOReal& v,
                                    SUMOReal& a, SUMOReal& s) {
    if (myComputeA && myLastV.count(id) == 0) {
        myLastV[id] = v;
        return false;
    }
    if (myComputeAForward) {
        t -= TS;
    }
    const PollutantsInterface::Emissions e = computeEmissions(id, c, v, a, s);
    o << t << ";" << v << ";" << a << ";" << s << ";"
      << e.CO << ";" << e.CO2 << ";" << e.HC << ";" << e.PMx << ";"
      << e.NOx << ";" << e.fuel << ";" << e.electricity << std::endl;
    return true;
}


bool
TrajectoriesHandler::writeXMLEmissions(const std::string id,
                                       const SUMOEmissionClass c,
                                       SUMOTime t, SUMOReal& v,
                                       SUMOReal a, SUMOReal s) {
    if (myComputeA && myLastV.count(id) == 0) {
        myLastV[id] = v;
        return false;
    }
    if (myCurrentTime != t) {
        if (myCurrentTime != -1) {
            myXMLOut->closeTag();
        }
        myCurrentTime = t;
        myXMLOut->openTag(SUMO_TAG_TIMESTEP).writeAttr(SUMO_ATTR_TIME, time2string(t));
    }
    const PollutantsInterface::Emissions e = computeEmissions(id, c, v, a, s);
    myXMLOut->openTag("vehicle").writeAttr("id", id).writeAttr("eclass", PollutantsInterface::getName(c));
    myXMLOut->writeAttr("CO2", e.CO2).writeAttr("CO", e.CO).writeAttr("HC", e.HC).writeAttr("NOx", e.NOx);
    myXMLOut->writeAttr("PMx", e.PMx).writeAttr("fuel", e.fuel).writeAttr("electricity", e.electricity);
    myXMLOut->writeAttr("speed", v).closeTag();
    return true;
}


void
TrajectoriesHandler::writeSums(std::ostream& o, const std::string id) {
    o << "CO:" << mySums[id].CO << std::endl
      << "CO2:" << mySums[id].CO2 << std::endl
      << "HC:" << mySums[id].HC << std::endl
      << "NOx:" << mySums[id].NOx << std::endl
      << "PMx:" << mySums[id].PMx << std::endl
      << "fuel:" << mySums[id].fuel << std::endl
      << "electricity:" << mySums[id].electricity << std::endl;
}


void
TrajectoriesHandler::writeNormedSums(std::ostream& o, const std::string id, const SUMOReal factor) {
    o << mySums[id].fuel / factor << ","
      << mySums[id].electricity / factor << ","
      << mySums[id].CO2 / factor << ","
      << mySums[id].NOx / factor << ","
      << mySums[id].CO / factor << ","
      << mySums[id].HC / factor << ","
      << mySums[id].PMx / factor << std::endl;
}


/****************************************************************************/
