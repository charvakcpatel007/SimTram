/****************************************************************************/
/// @file    NINavTeqHelper.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Jul 2006
/// @version $Id: NINavTeqHelper.cpp 21182 2016-07-18 06:46:01Z behrisch $
///
// Some parser methods shared around several formats containing NavTeq-Nets
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

#include "NINavTeqHelper.h"
#include <utils/common/TplConvert.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/UtilExceptions.h>
#include <netbuild/NBEdge.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
SUMOReal
NINavTeqHelper::getSpeed(const std::string& id, const std::string& speedClassS) {
    try {
        int speedClass = TplConvert::_2int(speedClassS.c_str());
        switch (speedClass) {
            case -1:
                return (SUMOReal) 1.0 / (SUMOReal) 3.6;
            case 1:
                return (SUMOReal) 200 / (SUMOReal) 3.6; //> 130 KPH / > 80 MPH
            case 2:
                return (SUMOReal) 120 / (SUMOReal) 3.6; //101-130 KPH / 65-80 MPH
            case 3:
                return (SUMOReal) 100 / (SUMOReal) 3.6; // 91-100 KPH / 55-64 MPH
            case 4:
                return (SUMOReal) 80 / (SUMOReal) 3.6; // 71-90 KPH / 41-54 MPH
            case 5:
                return (SUMOReal) 70 / (SUMOReal) 3.6; // 51-70 KPH / 31-40 MPH
            case 6:
                return (SUMOReal) 50 / (SUMOReal) 3.6; // 31-50 KPH / 21-30 MPH
            case 7:
                return (SUMOReal) 30 / (SUMOReal) 3.6; // 11-30 KPH / 6-20 MPH
            case 8:
                return (SUMOReal) 5 / (SUMOReal) 3.6; //< 11 KPH / < 6 MPH
            default:
                throw ProcessError("Invalid speed code (edge '" + id + "').");
        }
    } catch (NumberFormatException&) {
        throw ProcessError("Non-numerical value for an edge's speed type occured (edge '" + id + "').");
    }
}


int
NINavTeqHelper::getLaneNumber(const std::string& id, const std::string& laneNoS, SUMOReal speed) {
    try {
        int nolanes = TplConvert::_2int(laneNoS.c_str());
        if (nolanes < 0) {
            return 1;
        } else if (nolanes / 10 > 0) {
            return nolanes / 10;
        } else {
            switch (nolanes % 10) {
                case 1:
                    return 1;
                case 2:
                    nolanes = 2;
                    if (speed > 78.0 / 3.6) {
                        nolanes = 3;
                    }
                    return nolanes;
                case 3:
                    return 4;
                default:
                    throw ProcessError("Invalid lane number (edge '" + id + "').");
            }
        }
    } catch (NumberFormatException&) {
        throw ProcessError("Non-numerical value for an edge's lane number occured (edge '" + id + "'.");
    }
}


void
NINavTeqHelper::addVehicleClasses(NBEdge& e, const std::string& oclassS) {
    std::string classS = "0000000000" + oclassS;
    classS = classS.substr(classS.length() - 10);
    // 0: allow all vehicle types
    if (classS[0] == '1') {
        e.setPermissions(SVCAll);
        return;
    }
    // we have some restrictions. disallow all and then add classes indiviually
    e.setPermissions(0);
    // Passenger cars -- becomes SVC_PASSENGER
    if (classS[1] == '1') {
        e.allowVehicleClass(-1, SVC_PASSENGER);
    }
    // High Occupancy Vehicle -- becomes SVC_PASSENGER|SVC_HOV
    if (classS[2] == '1') {
        e.allowVehicleClass(-1, SVC_HOV);
        e.allowVehicleClass(-1, SVC_PASSENGER);
    }
    // Emergency Vehicle -- becomes SVC_PUBLIC_EMERGENCY
    if (classS[3] == '1') {
        e.allowVehicleClass(-1, SVC_EMERGENCY);
    }
    // Taxi -- becomes SVC_TAXI
    if (classS[4] == '1') {
        e.allowVehicleClass(-1, SVC_TAXI);
    }
    // Public Bus -- becomes SVC_BUS|SVC_COACH
    if (classS[5] == '1') {
        e.allowVehicleClass(-1, SVC_BUS);
        e.allowVehicleClass(-1, SVC_COACH);
    }
    // Delivery Truck -- becomes SVC_DELIVERY
    if (classS[6] == '1') {
        e.allowVehicleClass(-1, SVC_DELIVERY);
    }
    // Transport Truck -- becomes SVC_TRUCK|SVC_TRAILER
    if (classS[7] == '1') {
        e.allowVehicleClass(-1, SVC_TRUCK);
        e.allowVehicleClass(-1, SVC_TRAILER);
    }
    // Bicycle -- becomes SVC_BICYCLE
    if (classS[8] == '1') {
        e.allowVehicleClass(-1, SVC_BICYCLE);
    }
    // Pedestrian -- becomes SVC_PEDESTRIAN
    if (classS[9] == '1') {
        e.allowVehicleClass(-1, SVC_PEDESTRIAN);
    }
}


void
NINavTeqHelper::addVehicleClassesV6(NBEdge& e, const std::string& oclassS) {
    std::string classS = "0000000000" + oclassS;
    classS = classS.substr(classS.length() - 12);
    // 0: allow all vehicle types
    if (classS[0] == '1') {
        e.setPermissions(SVCAll);
        return;
    }
    // we have some restrictions. disallow all and then add classes indiviually
    e.setPermissions(0);
    // Passenger cars -- becomes SVC_PASSENGER
    if (classS[1] == '1') {
        e.allowVehicleClass(-1, SVC_PASSENGER);
    }
    // Residential Vehicle -- becomes SVC_PASSENGER
    if (classS[2] == '1') {
        e.allowVehicleClass(-1, SVC_PASSENGER);
    }
    // High Occupancy Vehicle -- becomes SVC_PASSENGER|SVC_HOV
    if (classS[3] == '1') {
        e.allowVehicleClass(-1, SVC_HOV);
        e.allowVehicleClass(-1, SVC_PASSENGER);
    }
    // Emergency Vehicle -- becomes SVC_PUBLIC_EMERGENCY
    if (classS[4] == '1') {
        e.allowVehicleClass(-1, SVC_EMERGENCY);
    }
    // Taxi -- becomes SVC_TAXI
    if (classS[5] == '1') {
        e.allowVehicleClass(-1, SVC_TAXI);
    }
    // Public Bus -- becomes SVC_BUS|SVC_COACH
    if (classS[6] == '1') {
        e.allowVehicleClass(-1, SVC_BUS);
        e.allowVehicleClass(-1, SVC_COACH);
    }
    // Delivery Truck -- becomes SVC_DELIVERY
    if (classS[7] == '1') {
        e.allowVehicleClass(-1, SVC_DELIVERY);
    }
    // Transport Truck -- becomes SVC_TRUCK|SVC_TRAILER
    if (classS[8] == '1') {
        e.allowVehicleClass(-1, SVC_TRUCK);
        e.allowVehicleClass(-1, SVC_TRAILER);
    }
    // Motorcycle -- becomes SVC_MOTORCYCLE
    if (classS[9] == '1') {
        e.allowVehicleClass(-1, SVC_MOTORCYCLE);
    }
    // Bicycle -- becomes SVC_BICYCLE
    if (classS[10] == '1') {
        e.allowVehicleClass(-1, SVC_BICYCLE);
    }
    // Pedestrian -- becomes SVC_PEDESTRIAN
    if (classS[11] == '1') {
        e.allowVehicleClass(-1, SVC_PEDESTRIAN);
    }
}
/****************************************************************************/

