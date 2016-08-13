/****************************************************************************/
/// @file    NIVissimClosures.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: NIVissimClosures.h 20433 2016-04-13 08:00:14Z behrisch $
///
// -------------------
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2002-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef NIVissimClosures_h
#define NIVissimClosures_h


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
class NIVissimClosures {
public:
    NIVissimClosures(const std::string& id,
                     int from_node, int to_node,
                     std::vector<int>& overEdges);
    ~NIVissimClosures();
    static bool dictionary(const std::string& id,
                           int from_node, int to_node, std::vector<int>& overEdges);
    static bool dictionary(const std::string& name, NIVissimClosures* o);
    static NIVissimClosures* dictionary(const std::string& name);
    static void clearDict();
private:
    typedef std::map<std::string, NIVissimClosures*> DictType;
    static DictType myDict;
    const std::string myID;
    int myFromNode, myToNode;
    std::vector<int> myOverEdges;

private:
    /// @brief invalidated copy constructor
    NIVissimClosures(const NIVissimClosures& s);

    /// @brief invalidated assignment operator
    NIVissimClosures& operator=(const NIVissimClosures& s);


};


#endif

/****************************************************************************/

