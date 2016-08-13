/****************************************************************************/
/// @file    AGFrame.cpp
/// @author  Walter Bamberger
/// @author  Daniel Krajzewicz
/// @date    Mon, 13 Sept 2010
/// @version $Id: AGFrame.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Configuration of the options of ActivityGen
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2001-2016 DLR (http://www.dlr.de/) and contributors
// activitygen module
// Copyright 2010 TUM (Technische Universitaet Muenchen, http://www.tum.de/)
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef AGFRAME_H
#define AGFRAME_H


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class AGFrame
 * @brief Sets and checks options for ActivityGen
 */
class AGFrame {
public:
    /** @brief Inserts options used by ActivityGen into the OptionsCont singleton
     */
    static void fillOptions();


    /** @brief Checks set options from the OptionsCont singleton for being valid
     * for usage within ActivityGen
     *
     * @return Whether all needed options are set
     * @todo Not implemented yet
     */
    static bool checkOptions();

};

#endif /* AGFRAME_H */
