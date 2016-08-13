/****************************************************************************/
/// @file    MSJunction.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Wed, 12 Dez 2001
/// @version $Id: MSJunction.h 20433 2016-04-13 08:00:14Z behrisch $
///
// The base class for an intersection
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
#ifndef MSJunction_h
#define MSJunction_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <map>
#include <utils/geom/Position.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/Named.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/UtilExceptions.h>
#include <utils/xml/SUMOXMLDefinitions.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSVehicle;
class MSLink;
class MSLane;
class MSEdge;

typedef std::vector<const MSEdge*> ConstMSEdgeVector;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSJunction
 * @brief The base class for an intersection
 */
class MSJunction : public Named {
public:
    /** @brief Constructor
     * @param[in] id The id of the junction
     * @param[in] position The position of the junction
     * @param[in] shape The shape of the junction
     */
    MSJunction(const std::string& id,
               SumoXMLNodeType type,
               const Position& position,
               const PositionVector& shape);


    /// @brief Destructor.
    virtual ~MSJunction();


    /** performs some initialisation after the loading
        (e.g., link map computation) */
    virtual void postloadInit();


    /** returns the junction's position */
    const Position& getPosition() const;


    /** @brief Returns this junction's shape
     * @return The shape of this junction
     */
    const PositionVector& getShape() const {
        return myShape;
    }

    virtual const std::vector<MSLink*>& getFoeLinks(const MSLink* const /*srcLink*/) const {
        return myEmptyLinks;
    }

    virtual const std::vector<MSLane*>& getFoeInternalLanes(const MSLink* const /*srcLink*/) const {
        return myEmptyLanes;
    }

    inline const ConstMSEdgeVector& getIncoming() const {
        return myIncoming;
    }

    inline const ConstMSEdgeVector& getOutgoing() const {
        return myOutgoing;
    }

    void addIncoming(MSEdge* edge) {
        myIncoming.push_back(edge);
    }

    void addOutgoing(MSEdge* edge) {
        myOutgoing.push_back(edge);
    }

    /// @brief return the type of this Junction
    SumoXMLNodeType getType() const {
        return myType;
    }

    /// @brief erase vehicle from myLinkLeaders
    void passedJunction(const MSVehicle* vehicle);

    /* @brief @return whether the foe vehicle is a leader for ego
     * @note vehicles are added to myLinkLeaders when first seen as a foe */
    bool isLeader(const MSVehicle* ego, const MSVehicle* foe);

protected:
    /// @brief Tye type of this junction
    SumoXMLNodeType myType;

    /// @brief The position of the junction
    Position myPosition;

    /// @brief The shape of the junction
    PositionVector myShape;

    std::vector<MSLink*> myEmptyLinks;
    std::vector<MSLane*> myEmptyLanes;


    /// @brief incoming edges
    ConstMSEdgeVector myIncoming;
    /// @brief outgoing edges
    ConstMSEdgeVector myOutgoing;

    /// @brief definition of the static dictionary type
    typedef std::map<std::string, MSJunction* > DictType;

    /// @brief map from leader vehicle to follower vehicles
    typedef std::map<const MSVehicle*, std::set<const MSVehicle*> > LeaderMap;
    LeaderMap myLinkLeaders;


private:
    /// @brief Invalidated copy constructor.
    MSJunction(const MSJunction&);

    /// @brief Invalidated assignment operator.
    MSJunction& operator=(const MSJunction&);

};


#endif

/****************************************************************************/

