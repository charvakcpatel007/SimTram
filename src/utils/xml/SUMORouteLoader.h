/****************************************************************************/
/// @file    SUMORouteLoader.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Wed, 6 Nov 2002
/// @version $Id: SUMORouteLoader.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A class that performs the loading of routes
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2002-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef SUMORouteLoader_h
#define SUMORouteLoader_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMORouteHandler;
class SUMOSAXReader;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SUMORouteLoader
 */
class SUMORouteLoader {
public:
    /// constructor
    SUMORouteLoader(SUMORouteHandler* handler);

    /// destructor
    ~SUMORouteLoader();

    /** loads vehicles until a vehicle is read that starts after
        the specified time */
    SUMOTime loadUntil(SUMOTime time);

    /// returns the information whether new data is available
    bool moreAvailable() const;

    /// returns the first departure time that was ever read
    SUMOTime getFirstDepart() const;

private:
    /// the used SAXReader
    SUMOSAXReader* myParser;

    /// information whether more vehicles should be available
    bool myMoreAvailable;

    /// the used Handler
    SUMORouteHandler* myHandler;

};


#endif

/****************************************************************************/

