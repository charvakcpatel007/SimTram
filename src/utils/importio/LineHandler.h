/****************************************************************************/
/// @file    LineHandler.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Fri, 19 Jul 2002
/// @version $Id: LineHandler.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Interface definition for a class which retrieves lines from a LineHandler
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
#ifndef LineHandler_h
#define LineHandler_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <string>
#include <utils/common/UtilExceptions.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class LineHandler
 * @brief Interface definition for a class which retrieves lines from a LineHandler
 *
 * The LineHandler is an interface for a class which retrieves information
 *  from a file linewise. The lines are suppoted to this class using the
 *  "report"-method. The LineHandler is used together with the LineReader
 *  which reads the lines from a file.
 * @see LineReader
 */
class LineHandler {
public:
    /// @brief constructor
    LineHandler() { }


    /// @brief (virtual) destructor
    virtual ~LineHandler() { }


    /** @brief Method that obatins a line read by the LineReader
     *
     * Real interface method, used by a LineReader, which retrieves lines from a file
     *
     * @param[in] result The read line
     * @return Whether the caller shall continue with reading
     */
    virtual bool report(const std::string& result) = 0;

};


#endif

/****************************************************************************/

