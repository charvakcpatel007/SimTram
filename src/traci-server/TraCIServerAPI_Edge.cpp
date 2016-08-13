/****************************************************************************/
/// @file    TraCIServerAPI_Edge.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Jerome Haerri
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @author  Mario Krumnow
/// @date    Sept 2002
/// @version $Id: TraCIServerAPI_Edge.cpp 21106 2016-07-04 12:52:52Z behrisch $
///
// APIs for getting/setting edge values via TraCI
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


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#ifndef NO_TRACI

#include <utils/common/StdDefs.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/pedestrians/MSPerson.h>
#include "TraCIConstants.h"
#include "TraCIServerAPI_Edge.h"
#include <microsim/MSEdgeWeightsStorage.h>
#include <utils/emissions/HelpersHarmonoise.h>

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// method definitions
// ===========================================================================
bool
TraCIServerAPI_Edge::processGet(TraCIServer& server, tcpip::Storage& inputStorage,
                                tcpip::Storage& outputStorage) {
    // variable & id
    int variable = inputStorage.readUnsignedByte();
    std::string id = inputStorage.readString();
    // check variable
    if (variable != ID_LIST && variable != VAR_EDGE_TRAVELTIME && variable != VAR_EDGE_EFFORT && variable != VAR_CURRENT_TRAVELTIME
            && variable != VAR_CO2EMISSION && variable != VAR_COEMISSION && variable != VAR_HCEMISSION && variable != VAR_PMXEMISSION
            && variable != VAR_NOXEMISSION && variable != VAR_FUELCONSUMPTION && variable != VAR_NOISEEMISSION
            && variable != VAR_ELECTRICITYCONSUMPTION && variable != VAR_WAITING_TIME
            && variable != LAST_STEP_VEHICLE_NUMBER && variable != LAST_STEP_MEAN_SPEED && variable != LAST_STEP_OCCUPANCY
            && variable != LAST_STEP_VEHICLE_HALTING_NUMBER && variable != LAST_STEP_LENGTH
            && variable != LAST_STEP_PERSON_ID_LIST
            && variable != LAST_STEP_VEHICLE_ID_LIST && variable != ID_COUNT && variable != VAR_PARAMETER) {
        return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "Get Edge Variable: unsupported variable " + toHex(variable, 2) + " specified", outputStorage);
    }
    // begin response building
    tcpip::Storage tempMsg;
    //  response-code, variableID, objectID
    tempMsg.writeUnsignedByte(RESPONSE_GET_EDGE_VARIABLE);
    tempMsg.writeUnsignedByte(variable);
    tempMsg.writeString(id);
    // process request
    if (variable == ID_LIST) {
        std::vector<std::string> ids;
        MSEdge::insertIDs(ids);
        tempMsg.writeUnsignedByte(TYPE_STRINGLIST);
        tempMsg.writeStringList(ids);
    } else if (variable == ID_COUNT) {
        std::vector<std::string> ids;
        MSEdge::insertIDs(ids);
        tempMsg.writeUnsignedByte(TYPE_INTEGER);
        tempMsg.writeInt((int) ids.size());
    } else {
        MSEdge* e = MSEdge::dictionary(id);
        if (e == 0) {
            return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "Edge '" + id + "' is not known", outputStorage);
        }
        switch (variable) {
            case VAR_EDGE_TRAVELTIME: {
                // time
                int time = 0;
                if (!server.readTypeCheckingInt(inputStorage, time)) {
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "The message must contain the time definition.", outputStorage);
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                SUMOReal value;
                if (!MSNet::getInstance()->getWeightsStorage().retrieveExistingTravelTime(e, time, value)) {
                    tempMsg.writeDouble(-1);
                } else {
                    tempMsg.writeDouble(value);
                }
            }
            break;
            case VAR_EDGE_EFFORT: {
                // time
                int time = 0;
                if (!server.readTypeCheckingInt(inputStorage, time)) {
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "The message must contain the time definition.", outputStorage);
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                SUMOReal value;
                if (!MSNet::getInstance()->getWeightsStorage().retrieveExistingEffort(e, time, value)) {
                    tempMsg.writeDouble(-1);
                } else {
                    tempMsg.writeDouble(value);
                }
            }
            break;
            case VAR_CURRENT_TRAVELTIME:
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(e->getCurrentTravelTime());
                break;
            case VAR_WAITING_TIME: {
                SUMOReal wtime = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    wtime += (*i)->getWaitingSeconds();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(wtime);
            }
            break;
            case LAST_STEP_PERSON_ID_LIST: {
                std::vector<std::string> personIDs;
                std::vector<MSTransportable*> persons = e->getSortedPersons(MSNet::getInstance()->getCurrentTimeStep());
                for (std::vector<MSTransportable*>::iterator it = persons.begin(); it != persons.end(); ++it) {
                    personIDs.push_back((*it)->getID());
                }
                tempMsg.writeUnsignedByte(TYPE_STRINGLIST);
                tempMsg.writeStringList(personIDs);
            }
            break;
            case LAST_STEP_VEHICLE_ID_LIST: {
                std::vector<std::string> vehIDs;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    const MSLane::VehCont& vehs = (*i)->getVehiclesSecure();
                    for (MSLane::VehCont::const_iterator j = vehs.begin(); j != vehs.end(); ++j) {
                        vehIDs.push_back((*j)->getID());
                    }
                    (*i)->releaseVehicles();
                }
                tempMsg.writeUnsignedByte(TYPE_STRINGLIST);
                tempMsg.writeStringList(vehIDs);
            }
            break;
            case VAR_CO2EMISSION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getCO2Emissions();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum);
            }
            break;
            case VAR_COEMISSION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getCOEmissions();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum);
            }
            break;
            case VAR_HCEMISSION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getHCEmissions();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum);
            }
            break;
            case VAR_PMXEMISSION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getPMxEmissions();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum);
            }
            break;
            case VAR_NOXEMISSION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getNOxEmissions();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum);
            }
            break;
            case VAR_FUELCONSUMPTION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getFuelConsumption();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum);
            }
            break;
            case VAR_NOISEEMISSION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (SUMOReal) pow(10., ((*i)->getHarmonoise_NoiseEmissions() / 10.));
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                if (sum != 0) {
                    tempMsg.writeDouble(HelpersHarmonoise::sum(sum));
                } else {
                    tempMsg.writeDouble(0);
                }
            }
            break;
            case VAR_ELECTRICITYCONSUMPTION: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getElectricityConsumption();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum);
            }
            break;
            case LAST_STEP_VEHICLE_NUMBER: {
                int sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getVehicleNumber();
                }
                tempMsg.writeUnsignedByte(TYPE_INTEGER);
                tempMsg.writeInt(sum);
            }
            break;
            case LAST_STEP_MEAN_SPEED: {
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(e->getMeanSpeed());
            }
            break;
            case LAST_STEP_OCCUPANCY: {
                SUMOReal sum = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    sum += (*i)->getNettoOccupancy();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                tempMsg.writeDouble(sum / (SUMOReal) lanes.size());
            }
            break;
            case LAST_STEP_VEHICLE_HALTING_NUMBER: {
                int halting = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    const MSLane::VehCont& vehs = (*i)->getVehiclesSecure();
                    for (MSLane::VehCont::const_iterator j = vehs.begin(); j != vehs.end(); ++j) {
                        if ((*j)->getSpeed() < SUMO_const_haltingSpeed) {
                            ++halting;
                        }
                    }
                    (*i)->releaseVehicles();
                }
                tempMsg.writeUnsignedByte(TYPE_INTEGER);
                tempMsg.writeInt(halting);
            }
            break;
            case LAST_STEP_LENGTH: {
                SUMOReal lengthSum = 0;
                int noVehicles = 0;
                const std::vector<MSLane*>& lanes = e->getLanes();
                for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                    const MSLane::VehCont& vehs = (*i)->getVehiclesSecure();
                    for (MSLane::VehCont::const_iterator j = vehs.begin(); j != vehs.end(); ++j) {
                        lengthSum += (*j)->getVehicleType().getLength();
                    }
                    noVehicles += (int) vehs.size();
                    (*i)->releaseVehicles();
                }
                tempMsg.writeUnsignedByte(TYPE_DOUBLE);
                if (noVehicles == 0) {
                    tempMsg.writeDouble(0);
                } else {
                    tempMsg.writeDouble(lengthSum / (SUMOReal) noVehicles);
                }
            }
            break;
            case VAR_PARAMETER: {
                std::string paramName = "";
                if (!server.readTypeCheckingString(inputStorage, paramName)) {
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "Retrieval of a parameter requires its name.", outputStorage);
                }
                tempMsg.writeUnsignedByte(TYPE_STRING);
                tempMsg.writeString(e->getParameter(paramName, ""));
            }
            break;
            default:
                break;
        }
    }
    server.writeStatusCmd(CMD_GET_EDGE_VARIABLE, RTYPE_OK, "", outputStorage);
    server.writeResponseWithLength(outputStorage, tempMsg);
    return true;
}


bool
TraCIServerAPI_Edge::processSet(TraCIServer& server, tcpip::Storage& inputStorage,
                                tcpip::Storage& outputStorage) {
    std::string warning = ""; // additional description for response
    // variable
    int variable = inputStorage.readUnsignedByte();
    if (variable != VAR_EDGE_TRAVELTIME && variable != VAR_EDGE_EFFORT && variable != VAR_MAXSPEED && variable != VAR_PARAMETER) {
        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "Change Edge State: unsupported variable " + toHex(variable, 2) + " specified", outputStorage);
    }
    // id
    std::string id = inputStorage.readString();
    MSEdge* e = MSEdge::dictionary(id);
    if (e == 0) {
        return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "Edge '" + id + "' is not known", outputStorage);
    }
    // process
    switch (variable) {
        case LANE_ALLOWED: {
            // read and set allowed vehicle classes
            std::vector<std::string> classes;
            if (!server.readTypeCheckingStringList(inputStorage, classes)) {
                return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "Allowed vehicle classes must be given as a list of strings.", outputStorage);
            }
            SVCPermissions permissions = parseVehicleClasses(classes);
            const std::vector<MSLane*>& lanes = e->getLanes();
            for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                (*i)->setPermissions(permissions);
            }
            e->rebuildAllowedLanes();
        }
        break;
        case LANE_DISALLOWED: {
            // read and set disallowed vehicle classes
            std::vector<std::string> classes;
            if (!server.readTypeCheckingStringList(inputStorage, classes)) {
                return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "Not allowed vehicle classes must be given as a list of strings.", outputStorage);
            }
            SVCPermissions permissions = ~parseVehicleClasses(classes); // negation yields allowed
            const std::vector<MSLane*>& lanes = e->getLanes();
            for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                (*i)->setPermissions(permissions);
            }
            e->rebuildAllowedLanes();
        }
        break;
        case VAR_EDGE_TRAVELTIME: {
            // read and set travel time
            if (inputStorage.readUnsignedByte() != TYPE_COMPOUND) {
                return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "Setting travel time requires a compound object.", outputStorage);
            }
            int parameterCount = inputStorage.readInt();
            if (parameterCount == 3) {
                // bound by time
                int begTime = 0, endTime = 0;
                double value = 0;
                if (!server.readTypeCheckingInt(inputStorage, begTime)) {
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "The first variable must be the begin time given as int.", outputStorage);
                }
                if (!server.readTypeCheckingInt(inputStorage, endTime)) {
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "The second variable must be the end time given as int.", outputStorage);
                }
                if (!server.readTypeCheckingDouble(inputStorage, value)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The third variable must be the value given as double", outputStorage);
                }
                MSNet::getInstance()->getWeightsStorage().addTravelTime(e, begTime, endTime, value);
            } else if (parameterCount == 1) {
                // unbound
                double value = 0;
                if (!server.readTypeCheckingDouble(inputStorage, value)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The variable must be the value given as double", outputStorage);
                }
                MSNet::getInstance()->getWeightsStorage().addTravelTime(e, SUMOReal(0), SUMOReal(SUMOTime_MAX), value);
            } else {
                return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "Setting travel time requires either begin time, end time, and value, or only value as parameter.", outputStorage);
            }
        }
        break;
        case VAR_EDGE_EFFORT: {
            // read and set effort
            if (inputStorage.readUnsignedByte() != TYPE_COMPOUND) {
                return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "Setting effort requires a compound object.", outputStorage);
            }
            int parameterCount = inputStorage.readInt();
            if (parameterCount == 3) {
                // bound by time
                int begTime = 0, endTime = 0;
                double value = 0;
                if (!server.readTypeCheckingInt(inputStorage, begTime)) {
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "The first variable must be the begin time given as int.", outputStorage);
                }
                if (!server.readTypeCheckingInt(inputStorage, endTime)) {
                    return server.writeErrorStatusCmd(CMD_GET_EDGE_VARIABLE, "The second variable must be the end time given as int.", outputStorage);
                }
                if (!server.readTypeCheckingDouble(inputStorage, value)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The third variable must be the value given as double", outputStorage);
                }
                MSNet::getInstance()->getWeightsStorage().addEffort(e, begTime, endTime, value);
            } else if (parameterCount == 1) {
                // unbound
                double value = 0;
                if (!server.readTypeCheckingDouble(inputStorage, value)) {
                    return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The variable must be the value given as double", outputStorage);
                }
                MSNet::getInstance()->getWeightsStorage().addEffort(e, SUMOReal(0), SUMOReal(SUMOTime_MAX), value);
            } else {
                return server.writeErrorStatusCmd(CMD_SET_VEHICLE_VARIABLE, "Setting effort requires either begin time, end time, and value, or only value as parameter.", outputStorage);
            }
        }
        break;
        case VAR_MAXSPEED: {
            // read and set max. speed
            double value = 0;
            if (!server.readTypeCheckingDouble(inputStorage, value)) {
                return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The speed must be given as a double.", outputStorage);
            }
            const std::vector<MSLane*>& lanes = e->getLanes();
            for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
                (*i)->setMaxSpeed(value);
            }
        }
        break;
        case VAR_PARAMETER: {
            if (inputStorage.readUnsignedByte() != TYPE_COMPOUND) {
                return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "A compound object is needed for setting a parameter.", outputStorage);
            }
            //readt itemNo
            inputStorage.readInt();
            std::string name;
            if (!server.readTypeCheckingString(inputStorage, name)) {
                return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The name of the parameter must be given as a string.", outputStorage);
            }
            std::string value;
            if (!server.readTypeCheckingString(inputStorage, value)) {
                return server.writeErrorStatusCmd(CMD_SET_EDGE_VARIABLE, "The value of the parameter must be given as a string.", outputStorage);
            }
            e->addParameter(name, value);
        }
        break;
        default:
            break;
    }
    server.writeStatusCmd(CMD_SET_EDGE_VARIABLE, RTYPE_OK, warning, outputStorage);
    return true;
}


bool
TraCIServerAPI_Edge::getShape(const std::string& id, PositionVector& shape) {
    MSEdge* e = MSEdge::dictionary(id);
    if (e == 0) {
        return false;
    }
    const std::vector<MSLane*>& lanes = e->getLanes();
    shape = lanes.front()->getShape();
    if (lanes.size() > 1) {
        copy(lanes.back()->getShape().begin(), lanes.back()->getShape().end(), back_inserter(shape));
    }
    return true;
}

#endif


/****************************************************************************/

