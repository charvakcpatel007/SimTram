/****************************************************************************/
/// @file    GUIIOGlobals.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    2004-11-23
/// @version $Id: GUIIOGlobals.h 20433 2016-04-13 08:00:14Z behrisch $
///
// The folder used as last
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
#ifndef GUIIOGlobals_h
#define GUIIOGlobals_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <fx.h>


// ===========================================================================
// global variable declarations
// ===========================================================================
/** @brief The folder used as last
 *
 * This value is loaded and stored within the registry on startup/shutdown
 *  of the application. It is changed after a file was loaded/saved.
 */
extern FXString gCurrentFolder;


#endif

/****************************************************************************/

