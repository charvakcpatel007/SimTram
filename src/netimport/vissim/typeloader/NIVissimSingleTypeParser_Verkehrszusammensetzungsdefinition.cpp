/****************************************************************************/
/// @file    NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 18 Dec 2002
/// @version $Id: NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition.cpp 20433 2016-04-13 08:00:14Z behrisch $
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
#include <utils/common/TplConvert.h>
#include "../NIImporter_Vissim.h"
#include "../tempstructs/NIVissimVehicleClass.h"
#include "../tempstructs/NIVissimVehicleClassVector.h"
#include "../tempstructs/NIVissimTrafficDescription.h"
#include "NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition::NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition::~NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition() {}


bool
NIVissimSingleTypeParser_Verkehrszusammensetzungsdefinition::parse(std::istream& from) {
    // id
    int id;
    from >> id; // type-checking is missing!
    // name
    std::string tag;
    from >> tag;
    std::string name = readName(from);
    // assigned vehicle classes
    tag = myRead(from);
    if (tag == "temperatur") {
        tag = myRead(from);
        tag = myRead(from);
    }
    NIVissimVehicleClassVector assignedTypes;
    while (tag != "DATAEND") {
        int type;
        from >> type;
        SUMOReal percentage;
        from >> tag;
        from >> percentage;
        int vwish;
        from >> tag;
        from >> vwish;
        assignedTypes.push_back(new NIVissimVehicleClass(vwish));
        tag = readEndSecure(from, "Fahrzeugtyp");
    }
    //
    return NIVissimTrafficDescription::dictionary(id, name, assignedTypes);
}



/****************************************************************************/

