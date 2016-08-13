/****************************************************************************/
/// @file    PCTypeDefHandler.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Thu, 16.03.2006
/// @version $Id: PCTypeDefHandler.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A handler for loading polygon type maps
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
#ifndef PCTypeDefHandler_h
#define PCTypeDefHandler_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/options/OptionsCont.h>
#include "PCTypeDefHandler.h"


// ===========================================================================
// class declarations
// ===========================================================================
class PCTypeToDef;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class PCTypeDefHandler
 * @brief A handler for loading polygon type maps
 */
class PCTypeDefHandler : public SUMOSAXHandler {
public:
    /** @brief Constructor
     * @param[in] oc The options used while loading the type map
     * @param[out] con The container to fill
     */
    PCTypeDefHandler(OptionsCont& oc, PCTypeMap& con);


    /// @brief Destructor
    virtual ~PCTypeDefHandler();


protected:
    /// @name inherited from GenericSAXHandler
    //@{

    /** @brief Called on the opening of a tag;
     *
     * @param[in] element ID of the currently opened element
     * @param[in] attrs Attributes within the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myStartElement
     * @todo Completely unsecure currently (invalid values may force abortion with no error message)
     */
    void myStartElement(int element,
                        const SUMOSAXAttributes& attrs);
    //@}


protected:
    /// @brief The options (program settings)
    OptionsCont& myOptions;

    /// @brief The type map to fill
    PCTypeMap& myContainer;


private:
    /// @brief Invalidated copy constructor
    PCTypeDefHandler(const PCTypeDefHandler& src);

    /// @brief Invalidated assignment operator
    PCTypeDefHandler& operator=(const PCTypeDefHandler& src);

};


#endif

/****************************************************************************/

