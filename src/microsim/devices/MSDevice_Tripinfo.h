/****************************************************************************/
/// @file    MSDevice_Tripinfo.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Fri, 30.01.2009
/// @version $Id: MSDevice_Tripinfo.h 21182 2016-07-18 06:46:01Z behrisch $
///
// A device which collects info on the vehicle trip
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2009-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSDevice_Tripinfo_h
#define MSDevice_Tripinfo_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSDevice.h"
#include <utils/common/SUMOTime.h>

// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVehicle;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_Tripinfo
 * @brief A device which collects info on the vehicle trip (mainly on departure and arrival)
 *
 * Each device collects departure time, lane and speed and the same for arrival.
 *
 * @see MSDevice
 */
class MSDevice_Tripinfo : public MSDevice {
public:
    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a tripinfo-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into);

    /// @brief update tripinfo statistics
    void updateStatistics() const;

    /// @brief generate output for vehicles which are still in the network
    static void generateOutputForUnfinished();

    /// @brief get statistics for printing to stdout
    static std::string printStatistics();

    /// @brief accessors for GUINet-Parameters
    static SUMOReal getAvgRouteLength();
    static SUMOReal getAvgDuration();
    static SUMOReal getAvgWaitingTime();
    static SUMOReal getAvgTimeLoss();
    static SUMOReal getAvgDepartDelay();

public:
    /// @brief Destructor.
    ~MSDevice_Tripinfo();



    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Checks for waiting steps when the vehicle moves
     *
     * @param[in] veh Vehicle that asks this reminder.
     * @param[in] oldPos Position before move.
     * @param[in] newPos Position after move with newSpeed.
     * @param[in] newSpeed Moving speed.
     * @return True (always).
     */
    bool notifyMove(SUMOVehicle& veh, SUMOReal oldPos, SUMOReal newPos, SUMOReal newSpeed);


    /** @brief Saves departure info on insertion
     *
     * @param[in] veh The entering vehicle.
     * @param[in] reason how the vehicle enters the lane
     * @return Always true
     * @see MSMoveReminder::notifyEnter
     * @see MSMoveReminder::Notification
     */
    bool notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason);


    /** @brief Saves arrival info
     *
     * @param[in] veh The leaving vehicle.
     * @param[in] lastPos Position on the lane when leaving.
     * @param[in] isArrival whether the vehicle arrived at its destination
     * @param[in] isLaneChange whether the vehicle changed from the lane
     * @return True if it did not leave the net.
     */
    bool notifyLeave(SUMOVehicle& veh, SUMOReal lastPos, MSMoveReminder::Notification reason);
    /// @}



    /** @brief Called on writing tripinfo output
     *
     * @param[in] os The stream to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::generateOutput
     */
    void generateOutput() const;



private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_Tripinfo(SUMOVehicle& holder, const std::string& id);


    /// @brief dummy constructor
    MSDevice_Tripinfo();


    /* @brief compute trip length and duration (depending on whether the
       vehicle arrived or not */
    void computeLengthAndDuration(SUMOReal& routeLength, SUMOTime& duration) const;

protected:
    /** @brief Internal notification about the vehicle moves
     *
     * @param[in] veh Vehicle that asks this reminder.
     * @param[in] timeOnLane time the vehicle spent on the lane.
     * @param[in] speed Moving speed.
     */
    void notifyMoveInternal(SUMOVehicle& veh, SUMOReal timeOnLane,
                            SUMOReal speed);

private:
    /// @brief The lane the vehicle departed at
    std::string myDepartLane;

    /// @brief The speed on departure
    SUMOReal myDepartSpeed;

    /// @brief The overall number of waiting steps
    int myWaitingSteps;

    /// @brief The vehicle's arrival time
    SUMOTime myArrivalTime;

    /// @brief The lane the vehicle arrived at
    std::string myArrivalLane;

    /// @brief The position on the lane the vehicle arrived at
    SUMOReal myArrivalPos;

    /// @brief The speed when arriving
    SUMOReal myArrivalSpeed;

    /// @brief The time loss when compared to the desired and allowed speed
    SUMOTime myTimeLoss;

    /// @brief devices which may still need to produce output
    typedef std::set<const MSDevice_Tripinfo*, Named::NamedLikeComparatorIdLess<MSDevice_Tripinfo> > DeviceSet;

    static DeviceSet myPendingOutput;

    /// @brief global tripinfo statistics
    static SUMOReal myVehicleCount;
    static SUMOReal myTotalRouteLength;
    static SUMOTime myTotalDuration;
    static SUMOTime myTotalWaitingTime;
    static SUMOTime myTotalTimeLoss;
    static SUMOTime myTotalDepartDelay;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_Tripinfo(const MSDevice_Tripinfo&);

    /// @brief Invalidated assignment operator.
    MSDevice_Tripinfo& operator=(const MSDevice_Tripinfo&);


};


#endif

/****************************************************************************/

