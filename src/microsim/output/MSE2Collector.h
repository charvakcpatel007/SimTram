/****************************************************************************/
/// @file    MSE2Collector.h
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Robbin Blokpoel
/// @author  Jakob Erdmann
/// @date    Mon Feb 03 2014 14:13 CET
/// @version $Id: MSE2Collector.h 21201 2016-07-19 11:57:22Z behrisch $
///
// An areal (along a single lane) detector
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2014 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSE2Collector_h
#define MSE2Collector_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <list>
#include <microsim/MSLane.h>
#include <microsim/MSNet.h>
#include <microsim/MSMoveReminder.h>
#include <microsim/output/MSDetectorFileOutput.h>
#include <utils/common/UtilExceptions.h>
#include <utils/vehicle/SUMOVehicle.h>

//Debug
//#define SWARM_DEBUG
#include <utils/common/SwarmDebug.h>

// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSE2Collector
 * @brief An areal (along a single lane) detector
 *
 * This detector traces vehicles which are at a part of a single lane. A
 *  vehicle that enters the detector is stored and the stored vehicles' speeds
 *  are used within each timestep to compute the detector values. As soon as the
 *  vehicle leaves the detector, it is no longer tracked.
 *
 * Determining entering and leaving vehicles is done via the MSMoveReminder
 *  interface. The values are computed by an event-callback (at the end of
 *  a time step).
 *
 * Jams are determined as following: A vehicle must at least drive haltingTimeThreshold
 *  at a speed lesser than haltingSpeedThreshold to be a "halting" vehicle.
 *  Two or more vehicles are "in jam" if they are halting and their distance is
 *  lesser than jamDistThreshold.
 *
 * @see Named
 * @see MSMoveReminder
 * @see MSDetectorFileOutput
 * @see Command
 */
class MSE2Collector : public MSMoveReminder, public MSDetectorFileOutput {
public:
    /** @brief Constructor
     *
     * @param[in] id The detector's unique id.
     * @param[in] usage Information how the detector is used
     * @param[in] lane The lane to place the detector at
     * @param[in] startPos Begin position of the detector
     * @param[in] detLength Length of the detector
     * @param[in] haltingTimeThreshold The time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold The speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold The distance between two vehicles in order to not count them to one jam
     * @todo The lane should not be given as a pointer
     */
    MSE2Collector(const std::string& id, DetectorUsage usage,
                  MSLane* const lane, SUMOReal startPos, SUMOReal detLength,
                  SUMOTime haltingTimeThreshold, SUMOReal haltingSpeedThreshold,
                  SUMOReal jamDistThreshold);


    /// @brief Destructor
    virtual ~MSE2Collector();


    /** @brief Returns the detector's usage type
     *
     * @see DetectorUsage
     * @return How the detector is used.
     */
    virtual DetectorUsage getUsageType() const {
        return myUsage;
    }



    /// @name Methods inherited from MSMoveReminder
    /// @{

    /** @brief Adds/removes vehicles from the list of vehicles to regard
     *
     * As soon as the reported vehicle enters the detector area (position>myStartPos)
     *  it is added to the list of vehicles to regard (myKnownVehicles). It
     *  is removed from this list if it leaves the detector (position<lengt>myEndPos).
     * The method returns true as long as the vehicle is not beyond the detector.
     *
     * @param[in] veh The vehicle in question.
     * @param[in] oldPos Position before the move-micro-timestep.
     * @param[in] newPos Position after the move-micro-timestep.
     * @param[in] newSpeed Unused here.
     * @return False, if vehicle passed the detector entierly, else true.
     * @see MSMoveReminder
     * @see MSMoveReminder::notifyMove
     */
    bool notifyMove(SUMOVehicle& veh, SUMOReal oldPos, SUMOReal newPos,
                    SUMOReal newSpeed);


    /** @brief Removes a known vehicle due to its lane-change
     *
     * If the reported vehicle is known, it is removed from the list of
     *  vehicles to regard (myKnownVehicles).
     *
     * @param[in] veh The leaving vehicle.
     * @param[in] lastPos Position on the lane when leaving.
     * @param[in] isArrival whether the vehicle arrived at its destination
     * @param[in] isLaneChange whether the vehicle changed from the lane
     * @see MSMoveReminder::notifyLeave
     */
    bool notifyLeave(SUMOVehicle& veh, SUMOReal lastPos, MSMoveReminder::Notification reason);


    /** @brief Adds the vehicle to known vehicles if not beyond the dector
     *
     * If the vehicles is within the detector are, it is added to the list
     *  of known vehicles.
     * The method returns true as long as the vehicle is not beyond the detector.
     *
     * @param[in] veh The entering vehicle.
     * @param[in] reason how the vehicle enters the lane
     * @return False, if vehicle passed the detector entirely, else true.
     * @see MSMoveReminder::notifyEnter
     * @see MSMoveReminder::Notification
     */
    bool notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason);
    /// @}



    /** @brief Computes the detector values in each time step
     *
     * This method should be called at the end of a simulation step, when
     *  all vehicles have moved. The current values are computed and
     *  summed up with the previous.
     *
     * @param[in] currentTime The current simulation time
     */
    void detectorUpdate(const SUMOTime step);



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


    /** @brief Returns the begin position of the detector
     *
     * @return The detector's begin position
     */
    SUMOReal getStartPos() const {
        return myStartPos;
    }


    /** @brief Returns the end position of the detector
     *
     * @return The detector's end position
     */
    SUMOReal getEndPos() const {
        return myEndPos;
    }


    /** @brief Resets all values
     *
     * This method is called on initialisation and as soon as the values
     *  were written. Values for the next interval may be collected, then.
     * The list of known vehicles stays untouched.
     */
    void reset();

    /** @brief Returns an estimate of the number of vehicles currently on the detector */
    int getEstimatedCurrentVehicleNumber(SUMOReal speedThreshold) const;

    /** @brief Returns an estimate of the lenght of the queue of vehicles currently stopped on the detector */
    SUMOReal getEstimateQueueLength() const;

    /// @name Methods returning current values
    /// @{

    /** @brief Returns the number of vehicles currently on the detector */
    int getCurrentVehicleNumber() const;

    /** @brief Returns the curent detector occupancy */
    SUMOReal getCurrentOccupancy() const;

    /** @brief Returns the mean vehicle speed of vehicles currently on the detector*/
    SUMOReal getCurrentMeanSpeed() const;

    /** @brief Returns the mean vehicle length of vehicles currently on the detector*/
    SUMOReal getCurrentMeanLength() const;

    /** @brief Returns the current number of jams */
    int getCurrentJamNumber() const;

    /** @brief Returns the length in vehicles of the currently largest jam */
    int getCurrentMaxJamLengthInVehicles() const;

    /** @brief Returns the length in meters of the currently largest jam */
    SUMOReal getCurrentMaxJamLengthInMeters() const;

    /** @brief Returns the length of all jams in vehicles */
    int getCurrentJamLengthInVehicles() const;

    /** @brief Returns the length of all jams in meters */
    SUMOReal getCurrentJamLengthInMeters() const;

    /** @brief Returns the length of all jams in meters */
    int getCurrentStartedHalts() const;

    /** @brief Returns the number of current haltings within the area
    *
    * If no vehicle is within the area, 0 is returned.
    *
    * @return The mean number of haltings within the area
    */
    int getCurrentHaltingNumber() const;

    /** @brief Returns the IDs of the vehicles within the area
     *
     * @return The IDs of the vehicles that have passed the entry, but not yet an exit point
     */
    std::vector<std::string> getCurrentVehicleIDs() const;
    /** @brief Returns the vehicles within the area
     *
     * @return The vehicles that have passed the entry, but not yet an exit point
     */
    const std::list<SUMOVehicle*>& getCurrentVehicles() const;
    /// @}

    /** \brief Returns the number of vehicles passed over the sensor.
     *
     * @return number of cars passed over the sensor
     */
    int getPassedVeh() {
        return myPassedVeh;
    }

    /** \brief Subtract the number of vehicles indicated from passed from the sensor count.
     *
     * @param[in] passed - int that indicates the number of vehicles to subtract
     */
    void subtractPassedVeh(int passed) {
        myPassedVeh -= passed;
    }

protected:
    /** @brief Internal representation of a jam
     *
     * Used in execute, instances of this structure are used to track
     *  begin and end positions (as vehicles) of a jam.
     */
    struct JamInfo {
        /// @brief The first standing vehicle
        std::list<SUMOVehicle*>::const_iterator firstStandingVehicle;

        /// @brief The last standing vehicle
        std::list<SUMOVehicle*>::const_iterator lastStandingVehicle;
    };


    /** @brief A class used to sort known vehicles by their position
     *
     * Sorting is needed, because the order may change if a vehicle has
     *  entered the lane by lane changing.
     *
     * We need to have the lane, because the vehicle's position - used
     *  for the sorting - may be beyond the lane's end (the vehicle may
     *  be on a new lane) and we have to ask for the vehicle's position
     *  using this information.
     */
    class by_vehicle_position_sorter {
    public:
        /** @brief constructor
         *
         * @param[in] lane The lane the detector is placed at
         */
        by_vehicle_position_sorter(const MSLane* const lane)
            : myLane(lane) { }


        /** @brief copy constructor
         *
         * @param[in] s The instance to copy
         */
        by_vehicle_position_sorter(const by_vehicle_position_sorter& s)
            : myLane(s.myLane) { }


        /** @brief Comparison funtcion
         *
         * @param[in] v1 First vehicle to compare
         * @param[in] v2 Second vehicle to compare
         * @return Whether the position of the first vehicles is smaller than the one of the second
         */
        int operator()(const SUMOVehicle* v1, const SUMOVehicle* v2);

    private:
        by_vehicle_position_sorter& operator=(const by_vehicle_position_sorter&); // just to avoid a compiler warning
    private:
        /// @brief The lane the detector is placed at
        const MSLane* const myLane;
    };


private:
    /// @name Detector parameter
    /// @{

    /// @brief A vehicle must driver slower than this to be counted as a part of a jam
    SUMOReal myJamHaltingSpeedThreshold;
    /// @brief A vehicle must be that long beyond myJamHaltingSpeedThreshold to be counted as a part of a jam
    SUMOTime myJamHaltingTimeThreshold;
    /// @brief Two standing vehicles must be closer than this to be counted into the same jam
    SUMOReal myJamDistanceThreshold;
    /// @brief The position the detector starts at
    SUMOReal myStartPos;
    /// @brief The position the detector ends at
    SUMOReal myEndPos;
    /// @}

    /// @brief Information about how this detector is used
    DetectorUsage myUsage;

    /// @brief List of known vehicles
    std::list<SUMOVehicle*> myKnownVehicles;

    /// @brief Storage for halting durations of known vehicles (for halting vehicles)
    std::map<const SUMOVehicle*, SUMOTime> myHaltingVehicleDurations;

    /// @brief Storage for halting durations of known vehicles (current interval)
    std::map<const SUMOVehicle*, SUMOTime> myIntervalHaltingVehicleDurations;

    /// @brief Halting durations of ended halts [s]
    std::vector<SUMOTime> myPastStandingDurations;

    /// @brief Halting durations of ended halts for the current interval [s]
    std::vector<SUMOTime> myPastIntervalStandingDurations;


    /// @name Values generated for aggregated file output
    /// @{

    /// @brief The sum of collected vehicle speeds [m/s]
    SUMOReal mySpeedSum;
    /// @brief The number of started halts [#]
    SUMOReal myStartedHalts;
    /// @brief The sum of jam lengths [m]
    SUMOReal myJamLengthInMetersSum;
    /// @brief The sum of jam lengths [#veh]
    int myJamLengthInVehiclesSum;
    /// @brief The number of collected samples [#]
    int myVehicleSamples;
    /// @brief The current aggregation duration [#steps]
    int myTimeSamples;
    /// @brief The sum of occupancies [%]
    SUMOReal myOccupancySum;
    /// @brief The maximum occupancy [%]
    SUMOReal myMaxOccupancy;
    /// @brief The mean jam length [#veh]
    int myMeanMaxJamInVehicles;
    /// @brief The mean jam length [m]
    SUMOReal myMeanMaxJamInMeters;
    /// @brief The max jam length [#veh]
    int myMaxJamInVehicles;
    /// @brief The max jam length [m]
    SUMOReal myMaxJamInMeters;
    /// @brief The mean number of vehicles [#veh]
    int myMeanVehicleNumber;
    /// @brief The max number of vehicles [#veh]
    int myMaxVehicleNumber;
    /// @}


    /// @name Values generated describing the current state
    /// @{

    /// @brief The current occupancy
    SUMOReal myCurrentOccupancy;
    /// @brief The current mean speed
    SUMOReal myCurrentMeanSpeed;
    /// @brief The current mean length
    SUMOReal myCurrentMeanLength;
    /// @brief The current jam number
    int myCurrentJamNo;
    /// @brief the current maximum jam length in meters
    SUMOReal myCurrentMaxJamLengthInMeters;
    /// @brief The current maximum jam length in vehicles
    int myCurrentMaxJamLengthInVehicles;
    /// @brief The overall jam length in meters
    SUMOReal myCurrentJamLengthInMeters;
    /// @brief The overall jam length in vehicles
    int myCurrentJamLengthInVehicles;
    /// @brief The number of started halts in the last step
    int myCurrentStartedHalts;
    /// @brief The number of halted vehicles [#]
    int myCurrentHaltingsNumber;
    /// @brief The number of vehicles passed on the sensor
    int myPassedVeh;
    /// @}


private:
    /// @brief Invalidated copy constructor.
    MSE2Collector(const MSE2Collector&);

    /// @brief Invalidated assignment operator.
    MSE2Collector& operator=(const MSE2Collector&);


};


#endif

/****************************************************************************/

