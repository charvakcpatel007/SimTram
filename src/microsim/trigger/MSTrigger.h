/****************************************************************************/
/// @file    MSTrigger.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    Sept 2002
/// @version $Id: MSTrigger.h 20433 2016-04-13 08:00:14Z behrisch $
///
// An abstract device that changes the state of the micro simulation
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
#ifndef MSTrigger_h
#define MSTrigger_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <set>
#include <utils/common/Named.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSTrigger
 * @brief An abstract device that changes the state of the micro simulation
 *
 * We name most of the additional microsim-structures "trigger" in order to
 *  allow some common operation on them.
 */
class MSTrigger : public Named {
public:
    /** @brief Constructor
     *
     * @param[in] id The id of the trigger
     */
    MSTrigger(const std::string& id);

    /// @brief Destructor
    virtual ~MSTrigger();

    /// @brief properly deletes all trigger instances
    static void cleanup();

private:
    /// @brief Invalidated copy constructor.
    MSTrigger(const MSTrigger&);

    /// @brief Invalidated assignment operator.
    MSTrigger& operator=(const MSTrigger&);

    static std::set<MSTrigger*> myInstances;

};


#endif

/****************************************************************************/

