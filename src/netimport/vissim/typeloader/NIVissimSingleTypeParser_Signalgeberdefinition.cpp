/****************************************************************************/
/// @file    NIVissimSingleTypeParser_Signalgeberdefinition.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 18 Dec 2002
/// @version $Id: NIVissimSingleTypeParser_Signalgeberdefinition.cpp 20433 2016-04-13 08:00:14Z behrisch $
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

#include <cassert>
#include <iostream>
#include <utils/common/TplConvert.h>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/VectorHelper.h>
#include "../NIImporter_Vissim.h"
#include "../tempstructs/NIVissimTL.h"
#include "NIVissimSingleTypeParser_Signalgeberdefinition.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Signalgeberdefinition::NIVissimSingleTypeParser_Signalgeberdefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Signalgeberdefinition::~NIVissimSingleTypeParser_Signalgeberdefinition() {}


bool
NIVissimSingleTypeParser_Signalgeberdefinition::parse(std::istream& from) {
    //
    int id;
    from >> id;
    //
    std::string tag, name;
    tag = myRead(from);
    if (tag == "name") {
        name = readName(from);
        tag = myRead(from);
    }
    // skip optional "Beschriftung"
    tag = overrideOptionalLabel(from, tag);
    //
    int lsaid;
    std::vector<int> groupids;
    if (tag == "lsa") {
        int groupid;
        from >> lsaid; // type-checking is missing!
        from >> tag; // "Gruppe"
        do {
            from >> groupid;
            groupids.push_back(groupid);
            tag = myRead(from);
        } while (tag == "oder");
        //
    } else {
        from >> tag; // strecke
        WRITE_WARNING("Omitting unknown traffic light.");
        return true;
    }
    if (tag == "typ") {
        from >> tag; // typ-value
        from >> tag; // "ort"
    }

    //
    from >> tag;
    int edgeid;
    from >> edgeid;

    from >> tag;
    int laneno;
    from >> laneno;

    from >> tag;
    SUMOReal position;
    from >> position;
    //
    while (tag != "fahrzeugklassen") {
        tag = myRead(from);
    }
    std::vector<int> assignedVehicleTypes = parseAssignedVehicleTypes(from, "N/A");
    //
    NIVissimTL::dictionary(lsaid); // !!! check whether someting is really done here
    NIVissimTL::NIVissimTLSignal* signal =
        new NIVissimTL::NIVissimTLSignal(id, name, groupids, edgeid,
                                         laneno, position, assignedVehicleTypes);
    if (!NIVissimTL::NIVissimTLSignal::dictionary(lsaid, id, signal)) {
        throw 1; // !!!
    }
    return true;
}



/****************************************************************************/

