/****************************************************************************/
/// @file    NIVissimSingleTypeParser_VWunschentscheidungsdefinition.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 18 Dec 2002
/// @version $Id: NIVissimSingleTypeParser_VWunschentscheidungsdefinition.cpp 20433 2016-04-13 08:00:14Z behrisch $
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
#include <vector>
#include <cassert>
#include <utils/common/TplConvert.h>
#include "../NIImporter_Vissim.h"
#include "../tempstructs/NIVissimEdge.h"
#include "../tempstructs/NIVissimConnection.h"
#include "NIVissimSingleTypeParser_VWunschentscheidungsdefinition.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
NIVissimSingleTypeParser_VWunschentscheidungsdefinition::NIVissimSingleTypeParser_VWunschentscheidungsdefinition(NIImporter_Vissim& parent)
    : NIImporter_Vissim::VissimSingleTypeParser(parent) {}


NIVissimSingleTypeParser_VWunschentscheidungsdefinition::~NIVissimSingleTypeParser_VWunschentscheidungsdefinition() {}


bool
NIVissimSingleTypeParser_VWunschentscheidungsdefinition::parse(std::istream& from) {
    std::string tag;
    from >> tag; // id
    from >> tag; // name
    tag = readName(from);
    tag = overrideOptionalLabel(from);
    from >> tag; // strecke
    std::string edgeid;
    from >> edgeid;
    from >> tag; // spur
    std::string lane;
    from >> lane;
    from >> tag; // bei
    std::string pos;
    from >> pos;
    from >> tag; // fahrzeugklasse
    from >> tag; // <fahrzeugklasse>
    from >> tag; // vwunsch
    std::string vwunsch;
    from >> vwunsch; // vwunsch
    std::vector<std::string> tmp;
    tmp.push_back("zeit");
    tmp.push_back("fahrzeugklasse");
    tag = readEndSecure(from, tmp);
    while (tag != "DATAEND" && tag != "zeit") {
        from >> tag;
        from >> tag;
        from >> tag;
        tag = myRead(from);
    }
    if (tag == "zeit") {
        from >> tag;
        from >> tag;
        from >> tag;
        from >> tag;
    }
    int numid = TplConvert::_2int(edgeid.c_str());
    int numlane = TplConvert::_2int(lane.c_str()) - 1;
    int numv = TplConvert::_2int(vwunsch.c_str());
    NIVissimEdge* e = NIVissimEdge::dictionary(numid);
    if (e == 0) {
        NIVissimConnection* c = NIVissimConnection::dictionary(numid);
        const std::vector<int>& lanes = c->getToLanes();
        e = NIVissimEdge::dictionary(c->getToEdgeID());
        for (std::vector<int>::const_iterator j = lanes.begin(); j != lanes.end(); j++) {
            e->setSpeed((*j), numv);
        }
        assert(e != 0);
    } else {
        e->setSpeed(numlane, numv);
    }
    return true;
}



/****************************************************************************/

