/****************************************************************************/
/// @file    AGBus.h
/// @author  Piotr Woznica
/// @author  Daniel Krajzewicz
/// @author  Walter Bamberger
/// @date    July 2010
/// @version $Id: AGBus.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A bus driving in the city
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
#ifndef AGBUS_H
#define AGBUS_H

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <string>


// ===========================================================================
// class definitions
// ===========================================================================
class AGBus {
public:
    AGBus(std::string name, int depTime) :
        name(name),
        departureTime(depTime) {};
    AGBus(int depTime) :
        departureTime(depTime) {};
    void setName(std::string name);
    int getDeparture();
    std::string getName();
    void print() const;

private:
    std::string name;
    int departureTime;
};

#endif

/****************************************************************************/
