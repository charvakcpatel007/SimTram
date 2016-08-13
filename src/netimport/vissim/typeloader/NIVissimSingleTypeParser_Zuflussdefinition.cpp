/****************************************************************************/
/// @file    NIVissimSingleTypeParser_Zuflussdefinition.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 18 Dec 2002
/// @version $Id: NIVissimSingleTypeParser_Zuflussdefinition.cpp 20433 2016-04-13 08:00:14Z behrisch $
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
#include "../tempstructs/NIVissimSource.h"
#include "NIVissimSingleTypeParser_Zuflussdefinition.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_Zuflussdefinition::NIVissimSingleTypeParser_Zuflussdefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_Zuflussdefinition::~NIVissimSingleTypeParser_Zuflussdefinition() {}


bool
NIVissimSingleTypeParser_Zuflussdefinition::parse(std::istream& from) {
    std::string id, edgeid;
    from >> id; // type-checking is missing!
    std::string tag, name;
    // override some optional values till q
    while (tag != "q") {
        tag = overrideOptionalLabel(from);
        if (tag == "name") {
            name = readName(from);
        } else if (tag == "strecke") {
            from >> edgeid; // type-checking is missing!
        }
    }
    // read q
    // bool exact = false;
    tag = myRead(from);
    if (tag == "exakt") {
        // exact = true;
        tag = myRead(from);
    }
    // SUMOReal q = TplConvert::_2SUMOReal(tag.c_str());
    // read the vehicle types
    from >> tag;
    int vehicle_combination;
    from >> vehicle_combination;
    // check whether optional time information is available
    tag = readEndSecure(from);
    SUMOReal beg, end;
    beg = -1;
    end = -1;
    if (tag == "zeit") {
        from >> tag;
        from >> beg;
        from >> tag;
        from >> end;
    }
    return NIVissimSource::dictionary(id, name, edgeid);
}



/****************************************************************************/

