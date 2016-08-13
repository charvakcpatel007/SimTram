/****************************************************************************/
/// @file    MSSOTLPolicyDesirability.cpp
/// @author  Riccardo Belletti
/// @date    2014-03-20
/// @version $Id: MSSOTLPolicyDesirability.cpp 20113 2016-03-01 13:22:42Z martintaraz $
///
// The class for Swarm-based low-level policy
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright 2001-2013 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#include "MSSOTLPolicyDesirability.h"

MSSOTLPolicyDesirability::MSSOTLPolicyDesirability(
    std::string keyPrefix,
    const std::map<std::string, std::string>& parameters) :
    Parameterised(parameters), myKeyPrefix(keyPrefix) {
}

MSSOTLPolicyDesirability::~MSSOTLPolicyDesirability() {
}
