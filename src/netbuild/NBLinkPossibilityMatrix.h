/****************************************************************************/
/// @file    NBLinkPossibilityMatrix.h
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: NBLinkPossibilityMatrix.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A matric to describe whether two links are foes to each other
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
#ifndef NBLinkPossibilityMatrix_h
#define NBLinkPossibilityMatrix_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <bitset>
#include <vector>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NBLinkPossibilityMatrix
 * Storing the information which links may be used simultanously, this matrix
 * is simply made by a vector of bitsets
 */
typedef std::vector<std::bitset<SUMO_MAX_CONNECTIONS> > NBLinkPossibilityMatrix;


#endif

/****************************************************************************/
