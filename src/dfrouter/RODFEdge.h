/****************************************************************************/
/// @file    RODFEdge.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Yun-Pang Floetteroed
/// @date    Thu, 16.03.2006
/// @version $Id: RODFEdge.h 21182 2016-07-18 06:46:01Z behrisch $
///
// An edge within the DFROUTER
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2006-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef RODFEdge_h
#define RODFEdge_h


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
#include <vector>
#include <router/ROEdge.h>
#include <utils/geom/Position.h>
#include "RODFDetectorFlow.h"


// ===========================================================================
// class declarations
// ===========================================================================
class ROLane;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class RODFEdge
 */
class RODFEdge : public ROEdge {
public:
    /** @brief Constructor
     *
     * @param[in] id The id of the edge
     * @param[in] from The node the edge begins at
     * @param[in] to The node the edge ends at
     * @param[in] index The numeric id of the edge
     */
    RODFEdge(const std::string& id, RONode* from, RONode* to, int index, const int priority);


    /// @brief Destructor
    ~RODFEdge();

    void setFlows(const std::vector<FlowDef>& flows);

    const std::vector<FlowDef>& getFlows() const;


private:
    std::vector<FlowDef> myFlows;

private:
    /// @brief Invalidated copy constructor
    RODFEdge(const RODFEdge& src);

    /// @brief Invalidated assignment operator
    RODFEdge& operator=(const RODFEdge& src);

};


#endif

/****************************************************************************/

