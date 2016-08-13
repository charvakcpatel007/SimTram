/****************************************************************************/
/// @file    NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 18 Dec 2002
/// @version $Id: NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
//
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
#include <utils/geom/Position.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/TplConvert.h>
#include <utils/common/ToString.h>
#include <netbuild/NBDistribution.h>
#include <utils/distribution/Distribution_Points.h>
#include "../NIImporter_Vissim.h"
#include "NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition::NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition::~NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition() {}


bool
NIVissimSingleTypeParser_Geschwindigkeitsverteilungsdefinition::parse(std::istream& from) {
    // id
    std::string id;
    from >> id;
    // list of points
    PositionVector points;
    std::string tag;
    do {
        tag = readEndSecure(from);
        if (tag == "name") {
            readName(from);
            tag = readEndSecure(from);
        }
        if (tag != "DATAEND") {
            SUMOReal p1 = TplConvert::_2SUMOReal(tag.c_str());
            from >> tag;
            SUMOReal p2 = TplConvert::_2SUMOReal(tag.c_str());
            points.push_back(Position(p1, p2));
        }
    } while (tag != "DATAEND");
    NBDistribution::dictionary("speed",
                               id, new Distribution_Points(id, points));
    return true;
}



/****************************************************************************/

