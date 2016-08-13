/****************************************************************************/
/// @file    GUIEvent.h
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id: GUIEvent.h 20995 2016-06-17 14:06:28Z behrisch $
///
// Definition of an own event class
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
#ifndef GUIEvent_h
#define GUIEvent_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/foxtools/FXThreadEvent.h>
#include <utils/foxtools/FXBaseObject.h>
#include <fx.h>


/**
 * As events are distinguished by their number, here is the enumeration
 * of our custom events
 */
enum GUIEventType {
    /// send when a simulation has been loaded
    EVENT_SIMULATION_LOADED,

    /// send when a simulation step has been performed
    EVENT_SIMULATION_STEP,

    /// send when a message occured
    EVENT_MESSAGE_OCCURED,

    /// send when a warning occured
    EVENT_WARNING_OCCURED,

    /// send when a error occured
    EVENT_ERROR_OCCURED,

    /// send when a status change occured
    EVENT_STATUS_OCCURED,

    /** @brief Send when the simulation is over;
        The reason and the time step are stored within the event */
    EVENT_SIMULATION_ENDED,

    /** @brief Send when a screenshot is requested;
        View and file name are stored within the event */
    EVENT_SCREENSHOT,

    /// End of events list; use this to define new
    EVENT_END
};


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * GUIEvent
 *
 */
class GUIEvent {
public:
    /// returns the event type
    GUIEventType getOwnType() const {
        return myType;
    }

    /// destructor
    virtual ~GUIEvent() { }

protected:
    /// constructor
    GUIEvent(GUIEventType ownType)
        : myType(ownType) { }


protected:
    /// the type of the event
    GUIEventType myType;

};


#endif

/****************************************************************************/

