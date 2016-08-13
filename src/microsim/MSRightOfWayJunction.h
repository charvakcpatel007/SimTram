/****************************************************************************/
/// @file    MSRightOfWayJunction.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 12 Dez 2001
/// @version $Id: MSRightOfWayJunction.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A junction with right-of-way - rules
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
#ifndef MSRightOfWayJunction_h
#define MSRightOfWayJunction_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSLogicJunction.h"
#include <bitset>
#include <vector>
#include <string>


// ===========================================================================
// class declarations
// ===========================================================================
class MSLane;
class MSJunctionLogic;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSRightOfWayJunction
 * @brief A junction with right-of-way - rules
 *
 * A class which realises junctions that do regard any kind of a right-of-way.
 * The rules for the right-of-way themselves are stored within the associated
 * "MSJunctionLogic" - structure.
 */
class MSRightOfWayJunction : public MSLogicJunction {
public:
    /** @brief Constructor
     * @param[in] id The id of the junction
     * @param[in] position The position of the junction
     * @param[in] shape The shape of the junction
     * @param[in] incoming The incoming lanes
     * @param[in] internal The internal lanes
     * @param[in] logic The logic of this junction
     */
    MSRightOfWayJunction(const std::string& id, SumoXMLNodeType type, const Position& position,
                         const PositionVector& shape,
                         std::vector<MSLane*> incoming,
#ifdef HAVE_INTERNAL_LANES
                         std::vector<MSLane*> internal,
#endif
                         MSJunctionLogic* logic);

    /// Destructor.
    virtual ~MSRightOfWayJunction();

    void postloadInit();

    const std::vector<MSLink*>& getFoeLinks(const MSLink* const srcLink) const {
        return myLinkFoeLinks.find(srcLink)->second;
    }

    const std::vector<MSLane*>& getFoeInternalLanes(const MSLink* const srcLink) const {
        return myLinkFoeInternalLanes.find(srcLink)->second;
    }

    // @brief return the underlying right-of-way and conflict matrix
    const MSJunctionLogic* getLogic() const {
        return myLogic;
    }

protected:
    /** the type of the junction (its logic) */
    MSJunctionLogic* myLogic;

    std::map<const MSLink*, std::vector<MSLink*> > myLinkFoeLinks;
    std::map<const MSLink*, std::vector<MSLane*> > myLinkFoeInternalLanes;


private:
    /// @brief Invalidated copy constructor.
    MSRightOfWayJunction(const MSRightOfWayJunction&);

    /// Invalidated assignment operator.
    MSRightOfWayJunction& operator=(const MSRightOfWayJunction&);

};


#endif

/****************************************************************************/

