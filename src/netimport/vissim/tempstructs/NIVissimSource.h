/****************************************************************************/
/// @file    NIVissimSource.h
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id: NIVissimSource.h 20433 2016-04-13 08:00:14Z behrisch $
///
// -------------------
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
#ifndef NIVissimSource_h
#define NIVissimSource_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif


#include <string>
#include <map>

// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 */
class NIVissimSource {
public:
    NIVissimSource(const std::string& id, const std::string& name,
                   const std::string& edgeid);
    ~NIVissimSource();
    static bool dictionary(const std::string& id, const std::string& name,
                           const std::string& edgeid);
    static bool dictionary(const std::string& id, NIVissimSource* o);
    static NIVissimSource* dictionary(const std::string& id);
    static void clearDict();
private:
    std::string myID;
    std::string myName;
    std::string myEdgeID;

private:
    typedef std::map<std::string, NIVissimSource*> DictType;
    static DictType myDict;
};


#endif

/****************************************************************************/

