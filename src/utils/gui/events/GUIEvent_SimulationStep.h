/****************************************************************************/
/// @file    GUIEvent_SimulationStep.h
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id: GUIEvent_SimulationStep.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Event send when the a simulation step has been performed by GUIRunThread
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
#ifndef GUIEvent_SimulationStep_h
#define GUIEvent_SimulationStep_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GUIEvent.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * GUIEvent_SimulationStep
 * Throw from GUIRunThread to GUIApplicationWindow and then further to all
 * displays after a step has been performed
 */
class GUIEvent_SimulationStep : public GUIEvent {
public:
    /// constructor
    GUIEvent_SimulationStep()
        : GUIEvent(EVENT_SIMULATION_STEP) { }

    /// destructor
    ~GUIEvent_SimulationStep() { }

};


#endif

/****************************************************************************/

