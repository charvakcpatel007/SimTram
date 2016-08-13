/****************************************************************************/
/// @file    NBDistribution.cpp
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id: NBDistribution.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// The base class for statistical distribution descriptions
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

#include "NBDistribution.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// static member variables
// ===========================================================================
NBDistribution::TypedDistDict NBDistribution::myDict;


// ===========================================================================
// method definitions
// ===========================================================================
bool
NBDistribution::dictionary(const std::string& type, const std::string& id,
                           Distribution* d) {
    TypedDistDict::iterator i = myDict.find(type);

    if (i == myDict.end()) {
        myDict[type][id] = d;
        return true;
    }
    DistDict& dict = (*i).second;
    DistDict::iterator j = dict.find(id);
    if (j == dict.end()) {
        myDict[type][id] = d;
        return true;
    }
    return false;
}


Distribution*
NBDistribution::dictionary(const std::string& type,
                           const std::string& id) {
    TypedDistDict::iterator i = myDict.find(type);
    if (i == myDict.end()) {
        return 0;
    }
    DistDict& dict = (*i).second;
    DistDict::iterator j = dict.find(id);
    if (j == dict.end()) {
        return 0;
    }
    return (*j).second;
}


void
NBDistribution::clear() {
    for (TypedDistDict::iterator i = myDict.begin(); i != myDict.end(); i++) {
        DistDict& dict = (*i).second;
        for (DistDict::iterator j = dict.begin(); j != dict.end(); j++) {
            delete(*j).second;
        }
    }
}



/****************************************************************************/

