/****************************************************************************/
/// @file    MSInductLoop.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    2004-11-23
/// @version $Id: MSInductLoop.h 21201 2016-07-19 11:57:22Z behrisch $
///
// An unextended detector measuring at a fixed position on a fixed lane.
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
#ifndef MSInductLoop_h
#define MSInductLoop_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <deque>
#include <map>
#include <functional>
#include <microsim/MSMoveReminder.h>
#include <microsim/output/MSDetectorFileOutput.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSLane;
class MSVehicle;
class OutputDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSInductLoop
 * @brief An unextended detector measuring at a fixed position on a fixed lane.
 *
 * Only vehicles that passed the entire detector are counted. We
 *  ignore vehicles that are emitted onto the detector and vehicles
 *  that change their lane while they are on the detector, because we
 *  cannot determine a meaningful enter/leave-times.
 *
 * This detector uses the MSMoveReminder mechanism, i.e. the vehicles
 *  call the detector if they pass it.
 *
 * @see MSMoveReminder
 * @see MSDetectorFileOutput
 */
class MSInductLoop
    : public MSMoveReminder, public MSDetectorFileOutput {
public:
    /**
     * @brief Constructor.
     *
     * Adds reminder to MSLane.
     *
     * @param[in] id Unique id
     * @param[in] lane Lane where detector woks on
     * @param[in] position Position of the detector within the lane
     * @param[in] splitByType Whether additional information split by vehicle classes shall be generated
     */
    MSInductLoop(const std::string& id, MSLane* const lane,
                 SUMOReal positionInMeters, bool splitByType);


    /// @brief Destructor
    ~MSInductLoop();


    /** @brief Resets all generated values to allow computation of next interval
     */
    virtual void reset();


    /** @brief Returns the position of the detector on the lane
     * @return The detector's position in meters
     */
    SUMOReal getPosition() const {
        return myPosition;
    }


    /// @name Methods inherited from MSMoveReminder
    /// @{
    /** @brief Checks whether the reminder is activated by a vehicle entering the lane
     *
     * Lane change means in this case that the vehicle changes to the lane
     *  the reminder is placed at.
     *
     * @param[in] veh The entering vehicle.
     * @param[in] reason how the vehicle enters the lane
     * @return True if vehicle enters the induction loop
     * @see Notification
     */
    bool notifyEnter(SUMOVehicle& veh, Notification reason);

    /** @brief Checks whether the vehicle shall be counted and/or shall still touch this MSMoveReminder
     *
     * As soon a vehicle enters the detector, its entry time is computed and stored
     *  in myVehiclesOnDet via enterDetectorByMove. If it passes the detector, the
     *  according leaving time is computed and stored, too, using leaveDetectorByMove.
     *
     * @param[in] veh Vehicle that asks this remider.
     * @param[in] oldPos Position before move.
     * @param[in] newPos Position after move with newSpeed.
     * @param[in] newSpeed Moving speed.
     * @return True if vehicle hasn't passed the detector completely.
     * @see MSMoveReminder
     * @see MSMoveReminder::notifyMove
     * @see enterDetectorByMove
     * @see leaveDetectorByMove
     */
    bool notifyMove(SUMOVehicle& veh, SUMOReal oldPos, SUMOReal newPos, SUMOReal newSpeed);


    /** @brief Dismisses the vehicle if it is on the detector due to a lane change
     *
     * If the vehicle is on the detector, it will be dismissed by incrementing
     *  myDismissedVehicleNumber and removing this vehicle's entering time from
     *  myVehiclesOnDet.
     *
     * @param[in] veh The leaving vehicle.
     * @param[in] lastPos Position on the lane when leaving.
     * @param[in] isArrival whether the vehicle arrived at its destination
     * @param[in] isLaneChange whether the vehicle changed from the lane
     * @see leaveDetectorByLaneChange
     * @see MSMoveReminder
     * @see MSMoveReminder::notifyLeave
     */
    bool notifyLeave(SUMOVehicle& veh, SUMOReal lastPos, MSMoveReminder::Notification reason);


    //@}



    /// @name Methods returning current values
    /// @{

    /** @brief Returns the speed of the vehicle on the detector
     *
     * If no vehicle is on the detector, -1 is returned, otherwise
     *  this vehicle's current speed.
     *
     * @return The speed [m/s] of the vehicle if one is on the detector, -1 otherwise
     */
    SUMOReal getCurrentSpeed() const;


    /** @brief Returns the length of the vehicle on the detector
     *
     * If no vehicle is on the detector, -1 is returned, otherwise
     *  this vehicle's length.
     *
     * @return The length [m] of the vehicle if one is on the detector, -1 otherwise
     */
    SUMOReal getCurrentLength() const;


    /** @brief Returns the current occupancy
     *
     * If a vehicle is on the detector, 1 is returned. If a vehicle has passed the detector
     *  in this timestep, its occupancy value is returned. If no vehicle has passed,
     *  0 is returned.
     *
     * @return This detector's current occupancy
     * @todo recheck (especially if more than one vehicle has passed)
     */
    SUMOReal getCurrentOccupancy() const;


    /** @brief Returns the number of vehicles that have passed the detector
     *
     * If a vehicle is on the detector, 1 is returned. If a vehicle has passed the detector
     *  in this timestep, 1 is returned. If no vehicle has passed,
     *  0 is returned.
     *
     * @return The number of vehicles that have passed the detector
     * @todo recheck (especially if more than one vehicle has passed)
     */
    int getCurrentPassedNumber() const;


    /** @brief Returns the ids of vehicles that have passed the detector
     *
     * @return The ids of vehicles that have passed the detector
     * @todo recheck (especially if more than one vehicle has passed)
     */
    std::vector<std::string> getCurrentVehicleIDs() const;


    /** @brief Returns the time since the last vehicle left the detector
     *
     * @return Timesteps from last leaving (detection) of the detector
     */
    SUMOReal getTimestepsSinceLastDetection() const;
    //@}



    /// @name Methods inherited from MSDetectorFileOutput.
    /// @{

    /** @brief Writes collected values into the given stream
     *
     * @param[in] dev The output device to write the data into
     * @param[in] startTime First time step the data were gathered
     * @param[in] stopTime Last time step the data were gathered
     * @see MSDetectorFileOutput::writeXMLOutput
     * @exception IOError If an error on writing occurs (!!! not yet implemented)
     */
    void writeXMLOutput(OutputDevice& dev, SUMOTime startTime, SUMOTime stopTime);


    /** @brief Opens the XML-output using "detector" as root element
     *
     * @param[in] dev The output device to write the root into
     * @see MSDetectorFileOutput::writeXMLDetectorProlog
     * @exception IOError If an error on writing occurs (!!! not yet implemented)
     */
    void writeXMLDetectorProlog(OutputDevice& dev) const;
    /// @}



    /** @brief Struct to store the data of the counted vehicle internally.
     *
     * These data is fed into a container.
     *
     * @see myVehicleDataCont
     */
    struct VehicleData {
        /** @brief Constructor
         *
         * Used if the vehicle has passed the induct loop completely
         *
         * @param[in] vehLength The length of the vehicle
         * @param[in] entryTimestep The time at which the vehicle entered the detector
         * @param[in] leaveTimestep The time at which the vehicle left the detector
         */
        VehicleData(const std::string& id, SUMOReal vehLength, SUMOReal entryTimestep, SUMOReal leaveTimestep,
                    const std::string& typeID)
            : idM(id), lengthM(vehLength), entryTimeM(entryTimestep), leaveTimeM(leaveTimestep),
              speedM(vehLength / MAX2(leaveTimestep - entryTimestep, NUMERICAL_EPS)), typeIDM(typeID) {}

        /// @brief The id of the vehicle
        std::string idM;
        /// @brief Length of the vehicle
        SUMOReal lengthM;
        /// @brief Entry-time of the vehicle in [s]
        SUMOReal entryTimeM;
        /// @brief Leave-time of the vehicle in [s]
        SUMOReal leaveTimeM;
        /// @brief Speed of the vehicle in [m/s]
        SUMOReal speedM;
        /// @brief Type of the vehicle in
        std::string typeIDM;
    };


    /** @brief Returns vehicle data for vehicles that have been on the detector starting at the given time
     *
     * @param[in] t The time from which vehicles shall be counted
     * @param[in] leaveTime Whether entryTime or leaveTime shall be compared against t
     *            (the latter gives a more complete picture but may include vehicles in multiple steps even if they did not stay on the detector)
     * @return The list of vehicles
     */
    virtual std::vector<VehicleData> collectVehiclesOnDet(SUMOTime t, bool leaveTime = false) const;


protected:
    /// @name Methods that add and remove vehicles from internal container
    /// @{

    /** @brief Introduces a vehicle to the detector's map myVehiclesOnDet.
     * @param veh The entering vehicle.
     * @param entryTimestep Timestep (not necessary integer) of entrance.
     */
    virtual void enterDetectorByMove(SUMOVehicle& veh, SUMOReal entryTimestep);


    /** @brief Processes a vehicle that leaves the detector
     *
     * Removes a vehicle from the detector's map myVehiclesOnDet and
     * adds the vehicle data to the internal myVehicleDataCont.
     *
     * @param veh The leaving vehicle.
     * @param leaveTimestep Timestep (not necessary integer) of leaving.
     */
    virtual void leaveDetectorByMove(SUMOVehicle& veh, SUMOReal leaveTimestep);


    /** @brief Removes a vehicle from the detector's map myVehiclesOnDet.
     * @param veh The leaving vehicle.
     * @param lastPos The last position of the leaving vehicle.
     */
    virtual void leaveDetectorByLaneChange(SUMOVehicle& veh, SUMOReal lastPos);
    /// @}


protected:
    /// @name Function for summing up values
    ///@{

    /// @brief Adds up VehicleData::speedM
    static inline SUMOReal speedSum(SUMOReal sumSoFar, const MSInductLoop::VehicleData& data) {
        return sumSoFar + data.speedM;
    }

    /// @brief Adds up VehicleData::lengthM
    static inline SUMOReal lengthSum(SUMOReal sumSoFar, const MSInductLoop::VehicleData& data) {
        return sumSoFar + data.lengthM;
    }
    ///@}


protected:
    /// @brief Detector's position on lane [m]
    const SUMOReal myPosition;

    /// @brief Whether additional information split by vehicle classes shall be generated
    bool mySplitByType;

    /// @brief Leave-time of the last vehicle detected [s]
    SUMOReal myLastLeaveTime;

    /// @brief Occupancy by the last vehicle detected.
    SUMOReal myLastOccupancy;

    /// @brief The number of dismissed vehicles
    int myDismissedVehicleNumber;


    /// @brief Type of myVehicleDataCont.
    typedef std::deque< VehicleData > VehicleDataCont;

    /// @brief Data of vehicles that have completely passed the detector
    VehicleDataCont myVehicleDataCont;

    /// @brief Data of vehicles that have completely passed the detector in the last time interval
    VehicleDataCont myLastVehicleDataCont;


    /// @brief Type of myVehiclesOnDet
    typedef std::map< SUMOVehicle*, SUMOReal > VehicleMap;

    /// @brief Data for vehicles that have entered the detector (vehicle -> enter time)
    VehicleMap myVehiclesOnDet;

    void writeTypedXMLOutput(OutputDevice& dev, SUMOTime startTime, SUMOTime stopTime,
                             const std::string& type, const VehicleDataCont& vdc, const VehicleMap& vm);

private:
    /// @brief Invalidated copy constructor.
    MSInductLoop(const MSInductLoop&);

    /// @brief Invalidated assignment operator.
    MSInductLoop& operator=(const MSInductLoop&);


};


#endif

/****************************************************************************/

