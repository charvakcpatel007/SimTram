/****************************************************************************/
/// @file    OutputDevice_File.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    2004
/// @version $Id: OutputDevice_File.cpp 20433 2016-04-13 08:00:14Z behrisch $
///
// An output device that encapsulates an ofstream
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2004-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <cstring>
#include <cerrno>
#include <utils/common/UtilExceptions.h>
#include "OutputDevice_File.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
OutputDevice_File::OutputDevice_File(const std::string& fullName, const bool binary)
    : OutputDevice(binary), myFileStream(0) {
#ifdef WIN32
    if (fullName == "/dev/null") {
        myFileStream = new std::ofstream("NUL");
    } else
#endif
        myFileStream = new std::ofstream(fullName.c_str(), binary ? std::ios::binary : std::ios_base::out);
    if (!myFileStream->good()) {
        delete myFileStream;
        throw IOError("Could not build output file '" + fullName + "' (" + std::strerror(errno) + ").");
    }
}


OutputDevice_File::~OutputDevice_File() {
    myFileStream->close();
    delete myFileStream;
}


std::ostream&
OutputDevice_File::getOStream() {
    return *myFileStream;
}


/****************************************************************************/

