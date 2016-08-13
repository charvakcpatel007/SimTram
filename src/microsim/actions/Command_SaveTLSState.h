/****************************************************************************/
/// @file    Command_SaveTLSState.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    15 Feb 2004
/// @version $Id: Command_SaveTLSState.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Writes the state of the tls to a file (in each second)
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
#ifndef Command_SaveTLSState_h
#define Command_SaveTLSState_h


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
#include <utils/common/Command.h>
#include <microsim/traffic_lights/MSTLLogicControl.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSTrafficLightLogic;
class OutputDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Command_SaveTLSState
 * @brief Writes the state of the tls to a file (in each second)
 */
class Command_SaveTLSState : public Command {
public:
    /** @brief Constructor
     *
     * @param[in] tlls The logic to write state of
     * @param[in] od The output device to write the state into
     */
    Command_SaveTLSState(const MSTLLogicControl::TLSLogicVariants& logics,
                         OutputDevice& od);


    /// @brief Destructor
    ~Command_SaveTLSState();


    /// @name Derived from Command
    /// @{

    /** @brief Writes the current state of the tls
     *
     * Returns always 1.
     *
     * @param[in] currentTime The current simulation time (unused)
     * @return Always DELTA_T (will be executed in next time step)
     * @see Command
     */
    SUMOTime execute(SUMOTime currentTime);
    /// @}


private:
    /// @brief The device to write to
    OutputDevice& myOutputDevice;

    /// @brief The traffic light logic to use
    const MSTLLogicControl::TLSLogicVariants& myLogics;


private:
    /// @brief Invalidated copy constructor.
    Command_SaveTLSState(const Command_SaveTLSState&);

    /// @brief Invalidated assignment operator.
    Command_SaveTLSState& operator=(const Command_SaveTLSState&);

};


#endif

/****************************************************************************/

