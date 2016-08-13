/****************************************************************************/
/// @file    NIVissimSingleTypeParser_Fahrzeugklassendefinition.h
/// @author  Daniel Krajzewicz
/// @date    Wed, 18 Dec 2002
/// @version $Id: NIVissimSingleTypeParser_Fahrzeugklassendefinition.h 20433 2016-04-13 08:00:14Z behrisch $
///
//
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
#ifndef NIVissimSingleTypeParser_Fahrzeugklassendefinition_h
#define NIVissimSingleTypeParser_Fahrzeugklassendefinition_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include "../NIImporter_Vissim.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NIVissimSingleTypeParser_Fahrzeugklassendefinition
 *
 */
class NIVissimSingleTypeParser_Fahrzeugklassendefinition :
    public NIImporter_Vissim::VissimSingleTypeParser {
public:
    /// Constructor
    NIVissimSingleTypeParser_Fahrzeugklassendefinition(NIImporter_Vissim& parent,
            NIImporter_Vissim::ColorMap& colorMap);

    /// Destructor
    ~NIVissimSingleTypeParser_Fahrzeugklassendefinition();

    /// Parses the data type from the given stream
    bool parse(std::istream& from);

private:
    /// color definitions
    NIImporter_Vissim::ColorMap& myColorMap;

};


#endif

/****************************************************************************/

