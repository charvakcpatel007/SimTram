/****************************************************************************/
/// @file    SUMOVTypeParameter.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    10.09.2009
/// @version $Id: SUMOVTypeParameter.h 21182 2016-07-18 06:46:01Z behrisch $
///
// Structure representing possible vehicle parameter
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
#ifndef SUMOVTypeParameter_h
#define SUMOVTypeParameter_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <map>
#include <utils/common/Parameterised.h>
#include <utils/common/RGBColor.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/SUMOVehicleClass.h>

// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class OptionsCont;


// ===========================================================================
// value definitions
// ===========================================================================
const int VTYPEPARS_LENGTH_SET = 1;
const int VTYPEPARS_MINGAP_SET = 2;
const int VTYPEPARS_MAXSPEED_SET = 2 << 2;
const int VTYPEPARS_PROBABILITY_SET = 2 << 3;
const int VTYPEPARS_SPEEDFACTOR_SET = 2 << 4;
const int VTYPEPARS_SPEEDDEVIATION_SET = 2 << 5;
const int VTYPEPARS_EMISSIONCLASS_SET = 2 << 6;
const int VTYPEPARS_COLOR_SET = 2 << 7;
const int VTYPEPARS_VEHICLECLASS_SET = 2 << 8;
const int VTYPEPARS_WIDTH_SET = 2 << 9;
const int VTYPEPARS_HEIGHT_SET = 2 << 10;
const int VTYPEPARS_SHAPE_SET = 2 << 11;
const int VTYPEPARS_OSGFILE_SET = 2 << 12;
const int VTYPEPARS_IMGFILE_SET = 2 << 13;
const int VTYPEPARS_IMPATIENCE_SET = 2 << 14;
const int VTYPEPARS_LANE_CHANGE_MODEL_SET = 2 << 15;
const int VTYPEPARS_PERSON_CAPACITY = 2 << 16;
const int VTYPEPARS_BOARDING_DURATION = 2 << 17;
const int VTYPEPARS_CONTAINER_CAPACITY = 2 << 18;
const int VTYPEPARS_LOADING_DURATION = 2 << 19;
const int VTYPEPARS_CAR_FOLLOW_MODEL = 2 << 20;
const int VTYPEPARS_MAXSPEED_LAT_SET = 2 << 21;
const int VTYPEPARS_LATALIGNMENT_SET = 2 << 22;
const int VTYPEPARS_MINGAP_LAT_SET = 2 << 23;


// ===========================================================================
// struct definitions
// ===========================================================================
/**
 * @class SUMOVTypeParameter
 * @brief Structure representing possible vehicle parameter
 */
class SUMOVTypeParameter : public Parameterised {
public:
    /** @brief Constructor
     *
     * Initialises the structure with default values
     */
    SUMOVTypeParameter(const std::string& vtid, const SUMOVehicleClass vc = SVC_IGNORING);


    /** @brief Returns whether the given parameter was set
     * @param[in] what The parameter which one asks for
     * @return Whether the given parameter was set
     */
    bool wasSet(int what) const {
        return (setParameter & what) != 0;
    }


    /** @brief Writes the vtype
     *
     * @param[in, out] dev The device to write into
     * @exception IOError not yet implemented
     */
    void write(OutputDevice& dev) const;

    /** @brief Validates stored car-following parameter
     */
    void validateCFParameter() const;


    /** @brief Returns the named value from the map, or the default if it is not contained there
     * @param[in] attr The corresponding xml attribute
     * @param[in] defaultValue The value to return if the given map does not contain the named variable
     * @return The named value from the map or the default if it does not exist there
     */
    SUMOReal getCFParam(const SumoXMLAttr attr, const SUMOReal defaultValue) const;

    /** @brief Returns the named value from the map, or the default if it is not contained there
     * @param[in] attr The corresponding xml attribute
     * @param[in] defaultValue The value to return if the given map does not contain the named variable
     * @return The named value from the map or the default if it does not exist there
     */
    SUMOReal getLCParam(const SumoXMLAttr attr, const SUMOReal defaultValue) const;


    /// @brief The vehicle type's id
    std::string id;

    /// @brief The physical vehicle length
    SUMOReal length;
    /// @brief This class' free space in front of the vehicle itself
    SUMOReal minGap;
    /// @brief The vehicle type's maximum speed [m/s]
    SUMOReal maxSpeed;
    /// @brief The probability when being added to a distribution without an explicit probability
    SUMOReal defaultProbability;
    /// @brief The factor by which the maximum speed may deviate from the allowed max speed on the street
    SUMOReal speedFactor;
    /// @brief The standard deviation for speed variations
    SUMOReal speedDev;
    /// @brief The emission class of this vehicle
    SUMOEmissionClass emissionClass;
    /// @brief The color
    RGBColor color;
    /// @brief The vehicle's class
    SUMOVehicleClass vehicleClass;
    /// @brief The vehicle's impatience (willingness to obstruct others)
    SUMOReal impatience;
    /// @brief The person capacity of the vehicle
    int personCapacity;
    /// @brief The container capacity of the vehicle
    int containerCapacity;
    /// @brief The time a person needs to board the vehicle
    SUMOTime boardingDuration;
    /// @brief The time a container needs to get loaded on the vehicle
    SUMOTime loadingDuration;

    /// @name Values for drawing this class' vehicles
    /// @{

    /// @brief This class' width
    SUMOReal width;

    /// @brief This class' height
    SUMOReal height;

    /// @brief This class' shape
    SUMOVehicleShape shape;

    /// @brief 3D model file for this class
    std::string osgFile;

    /// @brief Image file for this class
    std::string imgFile;
    /// @}


    /// @brief The enum-representation of the car-following model to use
    SumoXMLTag cfModel;

    /// @brief sub-model parameters
    typedef std::map<SumoXMLAttr, SUMOReal> SubParams;
    /// @brief Car-following parameter
    SubParams cfParameter;
    /// @brief Lane-changing parameter
    SubParams lcParameter;

    /// @brief The lane-change model to use
    LaneChangeModel lcModel;

    /// @brief The vehicle type's maximum lateral speed [m/s]
    SUMOReal maxSpeedLat;
    /// @brief The vehicles desired lateral alignment
    LateralAlignment latAlignment;
    /// @brief The vehicle type's minimum lateral gap [m]
    SUMOReal minGapLat;

    /// @brief Information for the router which parameter were set
    int setParameter;


    /// @brief Information whether this type was already saved (needed by routers)
    mutable bool saved;

    /// @brief Information whether this is a type-stub, being only referenced but not defined (needed by routers)
    mutable bool onlyReferenced;

    /** @brief Returns the default acceleration for the given vehicle class
     * This needs to be a function because the actual value is stored in the car following model
     * @param[in] vc the vehicle class
     * @return the acceleration in m/s^2
     */
    static SUMOReal getDefaultAccel(const SUMOVehicleClass vc = SVC_IGNORING);

    /** @brief Returns the default deceleration for the given vehicle class
     * This needs to be a function because the actual value is stored in the car following model
     * @param[in] vc the vehicle class
     * @return the deceleration in m/s^2
     */
    static SUMOReal getDefaultDecel(const SUMOVehicleClass vc = SVC_IGNORING);

    /** @brief Returns the default driver's imperfection (sigma or epsilon in Krauss' model) for the given vehicle class
     * This needs to be a function because the actual value is stored in the car following model
     * @param[in] vc the vehicle class
     * @return the imperfection as a value between 0 and 1
     */
    static SUMOReal getDefaultImperfection(const SUMOVehicleClass vc = SVC_IGNORING);

    /// @brief return the default parameters, this is a function due to the http://www.parashift.com/c++-faq/static-init-order.html
    static const SUMOVTypeParameter& getDefault();

};

#endif

/****************************************************************************/

