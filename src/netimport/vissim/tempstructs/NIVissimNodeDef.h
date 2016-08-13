/****************************************************************************/
/// @file    NIVissimNodeDef.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: NIVissimNodeDef.h 21182 2016-07-18 06:46:01Z behrisch $
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
#ifndef NIVissimNodeDef_h
#define NIVissimNodeDef_h


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
#include <utils/geom/Boundary.h>
#include "NIVissimExtendedEdgePointVector.h"
#include "NIVissimNodeCluster.h"


class NIVissimNodeDef {
public:
    NIVissimNodeDef(int id, const std::string& name);
    virtual ~NIVissimNodeDef();
    int buildNodeCluster();
//    virtual void computeBounding() = 0;
//    bool partialWithin(const AbstractPoly &p, SUMOReal off=0.0) const;
//    virtual void searchAndSetConnections() = 0;
    virtual SUMOReal getEdgePosition(int edgeid) const = 0;

public:
    static bool dictionary(int id, NIVissimNodeDef* o);
    static NIVissimNodeDef* dictionary(int id);
//    static std::vector<int> getWithin(const AbstractPoly &p, SUMOReal off=0.0);
//    static void buildNodeClusters();
//    static void dict_assignConnectionsToNodes();
    static int dictSize();
    static void clearDict();
    static int getMaxID();
protected:
    int myID;
    std::string myName;

private:
    typedef std::map<int, NIVissimNodeDef*> DictType;
    static DictType myDict;
    static int myMaxID;
};


#endif

/****************************************************************************/

