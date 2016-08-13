/****************************************************************************/
/// @file    MSSOTLHiLevelTrafficLightLogic.cpp
/// @author  Alessio Bonfietti
/// @date    Jun 2013
/// @version $Id: MSSOTLHiLevelTrafficLightLogic.cpp 21217 2016-07-22 10:57:44Z behrisch $
///
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
#include "MSSOTLHiLevelTrafficLightLogic.h"

MSSOTLHiLevelTrafficLightLogic::MSSOTLHiLevelTrafficLightLogic(MSTLLogicControl& tlcontrol,
        const std::string& id, const std::string& subid, const Phases& phases,
        int step, SUMOTime delay,
        const std::map<std::string, std::string>& parameters) :
    MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay,
                            parameters) {
    // Setting default values

}

MSSOTLHiLevelTrafficLightLogic::MSSOTLHiLevelTrafficLightLogic(MSTLLogicControl& tlcontrol,
        const std::string& id, const std::string& subid, const Phases& phases,
        int step, SUMOTime delay,
        const std::map<std::string, std::string>& parameters,
        MSSOTLSensors* sensors) :
    MSSOTLTrafficLightLogic(tlcontrol, id, subid, phases, step, delay,
                            parameters, sensors) {
    // Setting default values

}

MSSOTLHiLevelTrafficLightLogic::~MSSOTLHiLevelTrafficLightLogic() {
    for (int i = 0; i < (int)policies.size(); i++) {
        delete(policies[i]);
    }
}

void MSSOTLHiLevelTrafficLightLogic::addPolicy(MSSOTLPolicy* policy) {
    policies.push_back(policy);
}

void MSSOTLHiLevelTrafficLightLogic::init(NLDetectorBuilder& nb) throw(ProcessError) {
    MSSOTLTrafficLightLogic::init(nb);
}

void MSSOTLHiLevelTrafficLightLogic::activate(MSSOTLPolicy* policy) {
    currentPolicy = policy;
}
