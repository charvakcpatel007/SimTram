/****************************************************************************/
/// @file    ROMAFrame.h
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: ROMAFrame.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Sets and checks options for ma-routing
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
#ifndef ROMAFrame_h
#define ROMAFrame_h


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
 * @class ROMAFrame
 * @brief Sets and checks options for dua-routing
 */
class ROMAFrame {
public:
    /** @brief Inserts options used by duarouter into the OptionsCont-singleton
     *
     * As duarouter shares several options with other routing appplications, the
     *  insertion of these is done via a call to ROFrame::fillOptions.
     *
     * duarouter-specific options are added afterwards via calls to
     *  "addImportOptions" and "addDUAOptions".
     */
    static void fillOptions();


    /** @brief Checks set options from the OptionsCont-singleton for being valid for usage within duarouter
     *
     * Currently, this is done via a call to "ROFrame::checkOptions".
     *
     * @return Whether all needed options are set
     * @todo probably, more things should be checked...
     */
    static bool checkOptions();


protected:
    /** @brief Inserts import options used by duarouter into the OptionsCont-singleton
     */
    static void addImportOptions();


    /** @brief Inserts dua options used by duarouter into the OptionsCont-singleton
     */
    static void addAssignmentOptions();


};


#endif

/****************************************************************************/

