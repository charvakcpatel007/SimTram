/****************************************************************************/
/// @file    FileHelpers.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mon, 17 Dec 2001
/// @version $Id: FileHelpers.cpp 21201 2016-07-19 11:57:22Z behrisch $
///
// Functions for an easier usage of files
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


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#ifdef _MSC_VER
// this is how fox does it in xincs.h
#include <io.h>
#define access _access
#define R_OK    4       /* Test for read permission.  */
#else
#include <unistd.h>
#endif
#include <fstream>
#include "FileHelpers.h"
#include "StringTokenizer.h"
#include "MsgHandler.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// file access functions
// ---------------------------------------------------------------------------
bool
FileHelpers::isReadable(std::string path) {
    if (path.length() == 0) {
        return false;
    }
    while (path[path.length() - 1] == '/' || path[path.length() - 1] == '\\') {
        path.erase(path.end() - 1);
    }
    if (path.length() == 0) {
        return false;
    }
    return access(path.c_str(), R_OK) == 0;
}


// ---------------------------------------------------------------------------
// file path evaluating functions
// ---------------------------------------------------------------------------
std::string
FileHelpers::getFilePath(const std::string& path) {
    const std::string::size_type beg = path.find_last_of("\\/");
    if (beg == std::string::npos || beg == 0) {
        return "";
    }
    return path.substr(0, beg + 1);
}


std::string
FileHelpers::getConfigurationRelative(const std::string& configPath,
                                      const std::string& path) {
    std::string retPath = getFilePath(configPath);
    return retPath + path;
}


bool
FileHelpers::isSocket(const std::string& name) {
    const std::string::size_type colonPos = name.find(":");
    return (colonPos != std::string::npos) && (colonPos > 1);
}


bool
FileHelpers::isAbsolute(const std::string& path) {
    if (isSocket(path)) {
        return true;
    }
    // check UNIX - absolute paths
    if (path.length() > 0 && path[0] == '/') {
        return true;
    }
    // check Windows - absolute paths
    if (path.length() > 0 && path[0] == '\\') {
        return true;
    }
    if (path.length() > 1 && path[1] == ':') {
        return true;
    }
    if (path == "nul" || path == "NUL") {
        return true;
    }
    return false;
}


std::string
FileHelpers::checkForRelativity(const std::string& filename,
                                const std::string& basePath) {
    if (filename == "stdout" || filename == "STDOUT" || filename == "-") {
        return "stdout";
    }
    if (filename == "stderr" || filename == "STDERR") {
        return "stderr";
    }
    if (filename == "nul" || filename == "NUL") {
        return "/dev/null";
    }
    if (!isSocket(filename) && !isAbsolute(filename)) {
        return getConfigurationRelative(basePath, filename);
    }
    return filename;
}


std::string
FileHelpers::prependToLastPathComponent(const std::string& prefix, const std::string& path) {
    const std::string::size_type sep_index = path.find_last_of("\\/");
    if (sep_index == std::string::npos) {
        return prefix + path;
    } else {
        return path.substr(0, sep_index + 1) + prefix + path.substr(sep_index + 1);
    }
}

// ---------------------------------------------------------------------------
// binary reading/writing functions
// ---------------------------------------------------------------------------
std::ostream&
FileHelpers::writeInt(std::ostream& strm, int value) {
    strm.write((char*) &value, sizeof(int));
    return strm;
}


std::ostream&
FileHelpers::writeUInt(std::ostream& strm, int value) {
    strm.write((char*) &value, sizeof(int));
    return strm;
}


std::ostream&
FileHelpers::writeFloat(std::ostream& strm, SUMOReal value) {
    strm.write((char*) &value, sizeof(SUMOReal));
    return strm;
}


std::ostream&
FileHelpers::writeByte(std::ostream& strm, unsigned char value) {
    strm.write((char*) &value, sizeof(char));
    return strm;
}


std::ostream&
FileHelpers::writeString(std::ostream& strm, const std::string& value) {
    int size = (int)value.length();
    const char* cstr = value.c_str();
    writeUInt(strm, (int) size);
    strm.write((char*) cstr, (std::streamsize)(sizeof(char)*size));
    return strm;
}


std::ostream&
FileHelpers::writeTime(std::ostream& strm, SUMOTime value) {
    strm.write((char*) &value, sizeof(SUMOTime));
    return strm;
}


/****************************************************************************/

