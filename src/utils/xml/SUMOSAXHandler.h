/****************************************************************************/
/// @file    SUMOSAXHandler.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: SUMOSAXHandler.h 20433 2016-04-13 08:00:14Z behrisch $
///
// SAX-handler base for SUMO-files
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
#ifndef SUMOSAXHandler_h
#define SUMOSAXHandler_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/xml/GenericSAXHandler.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SUMOSAXHandler
 * @brief SAX-handler base for SUMO-files
 *
 * This class is a GenericSAXHandler which knows all tags SUMO uses, so all
 *  SUMO-XML - loading classes should be derived from it.
 */
class SUMOSAXHandler : public GenericSAXHandler {
public:
    /**
     * @brief Constructor
     *
     * @param[in] file The name of the processed file
     */
    SUMOSAXHandler(const std::string& file = "");


    /// Destructor
    virtual ~SUMOSAXHandler();


private:
    /// @brief invalidated copy constructor
    SUMOSAXHandler(const SUMOSAXHandler& s);

    /// @brief invalidated assignment operator
    const SUMOSAXHandler& operator=(const SUMOSAXHandler& s);

};


#endif

/****************************************************************************/

