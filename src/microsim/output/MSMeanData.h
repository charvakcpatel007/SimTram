/****************************************************************************/
/// @file    MSMeanData.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, 17.11.2009
/// @version $Id: MSMeanData.h 20482 2016-04-18 20:49:42Z behrisch $
///
// Data collector for edges/lanes
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
#ifndef MSMeanData_h
#define MSMeanData_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <set>
#include <list>
#include <limits>
#include <microsim/output/MSDetectorFileOutput.h>
#include <microsim/MSMoveReminder.h>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class MSEdge;
class MSLane;
class SUMOVehicle;

typedef std::vector<MSEdge*> MSEdgeVector;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSMeanData
 * @brief Data collector for edges/lanes
 *
 * This structure does not contain the data itself, it is stored within
 *  MeanDataValues-MoveReminder objects.
 * This class is used to build the output, optionally, in the case
 *  of edge-based dump, aggregated over the edge's lanes.
 *
 * @todo consider error-handling on write (using IOError)
 */
class MSMeanData : public MSDetectorFileOutput {
public:
    /**
     * @class MeanDataValues
     * @brief Data structure for mean (aggregated) edge/lane values
     *
     * Structure holding values that describe the emissions aggregated
     *  over some seconds.
     */
    class MeanDataValues : public MSMoveReminder {
    public:
        /** @brief Constructor */
        MeanDataValues(MSLane* const lane, const SUMOReal length, const bool doAdd, const std::set<std::string>* const vTypes = 0);

        /** @brief Destructor */
        virtual ~MeanDataValues();


        /** @brief Resets values so they may be used for the next interval
         */
        virtual void reset(bool afterWrite = false) = 0;

        /** @brief Add the values of this to the given one and store them there
         *
         * @param[in] val The meandata to add to
         */
        virtual void addTo(MeanDataValues& val) const = 0;


        /** @brief Called if the vehicle enters the reminder's lane
         *
         * @param[in] veh The entering vehicle.
         * @param[in] reason how the vehicle enters the lane
         * @see MSMoveReminder
         * @see MSMoveReminder::notifyEnter
         * @see MSMoveReminder::Notification
         */
        virtual bool notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason);


        /** @brief Checks whether the reminder still has to be notified about the vehicle moves
         *
         * Indicator if the reminders is still active for the passed
         * vehicle/parameters. If false, the vehicle will erase this reminder
         * from it's reminder-container.
         *
         * @param[in] veh Vehicle that asks this reminder.
         * @param[in] oldPos Position before move.
         * @param[in] newPos Position after move with newSpeed.
         * @param[in] newSpeed Moving speed.
         *
         * @return True if vehicle hasn't passed the reminder completely.
         */
        bool notifyMove(SUMOVehicle& veh, SUMOReal oldPos,
                        SUMOReal newPos, SUMOReal newSpeed);


        /** @brief Called if the vehicle leaves the reminder's lane
         *
         * @param veh The leaving vehicle.
         * @param[in] lastPos Position on the lane when leaving.
         * @param[in] reason how the vehicle leaves the lane
         * @see MSMoveReminder
         * @see MSMoveReminder::notifyLeave
         */
        virtual bool notifyLeave(SUMOVehicle& veh, SUMOReal lastPos,
                                 MSMoveReminder::Notification reason);


        /** @brief Tests whether the vehicles type is to be regarded
         *
         * @param[in] veh The regarded vehicle
         * @return whether the type of the vehicle is in the set of regarded types
         */
        bool vehicleApplies(const SUMOVehicle& veh) const;


        /** @brief Returns whether any data was collected.
         *
         * @return whether no data was collected
         */
        virtual bool isEmpty() const;


        /** @brief Called if a per timestep update is needed. Default does nothing.
         */
        virtual void update();

        /** @brief Writes output values into the given stream
         *
         * @param[in] dev The output device to write the data into
         * @param[in] period Length of the period the data were gathered
         * @param[in] numLanes The total number of lanes for which the data was collected
         * @exception IOError If an error on writing occurs (!!! not yet implemented)
         */
        virtual void write(OutputDevice& dev, const SUMOTime period,
                           const SUMOReal numLanes, const SUMOReal defaultTravelTime,
                           const int numVehicles = -1) const = 0;

        /** @brief Returns the number of collected sample seconds.
         * @return the number of collected sample seconds
         */
        virtual SUMOReal getSamples() const;

    protected:
        /// @brief The length of the lane / edge the data collector is on
        const SUMOReal myLaneLength;

        /// @name Collected values
        /// @{
        /// @brief The number of sampled vehicle movements (in s)
        SUMOReal sampleSeconds;
    public:
        /// @brief The sum of the distances the vehicles travelled
        SUMOReal travelledDistance;
        //@}

    protected:
        /// @brief The vehicle types to look for (0 or empty means all)
        const std::set<std::string>* const myVehicleTypes;

    };


    /**
     * @class MeanDataValueTracker
     * @brief Data structure for mean (aggregated) edge/lane values for tracked vehicles
     */
    class MeanDataValueTracker : public MeanDataValues {
    public:
        /** @brief Constructor */
        MeanDataValueTracker(MSLane* const lane, const SUMOReal length,
                             const std::set<std::string>* const vTypes = 0,
                             const MSMeanData* const parent = 0);

        /** @brief Destructor */
        virtual ~MeanDataValueTracker();

        /** @brief Resets values so they may be used for the next interval
         */
        void reset(bool afterWrite);

        /** @brief Add the values of this to the given one and store them there
         *
         * @param[in] val The meandata to add to
         */
        void addTo(MSMeanData::MeanDataValues& val) const;

        /// @name Methods inherited from MSMoveReminder
        /// @{

        /** @brief Internal notification about the vehicle moves
         *
         * Indicator if the reminders is still active for the passed
         * vehicle/parameters. If false, the vehicle will erase this reminder
         * from it's reminder-container.
         *
         * @param[in] veh Vehicle that asks this reminder.
         * @param[in] timeOnLane time the vehicle spent on the lane.
         * @param[in] speed Moving speed.
         */
        void notifyMoveInternal(SUMOVehicle& veh, SUMOReal timeOnLane, SUMOReal speed);


        /** @brief Called if the vehicle leaves the reminder's lane
         *
         * @param veh The leaving vehicle.
         * @param[in] lastPos Position on the lane when leaving.
         * @param[in] isArrival whether the vehicle arrived at its destination
         * @param[in] isLaneChange whether the vehicle changed from the lane
         * @see MSMoveReminder
         * @see MSMoveReminder::notifyLeave
         */
        bool notifyLeave(SUMOVehicle& veh, SUMOReal lastPos, MSMoveReminder::Notification reason);


        /** @brief Computes current values and adds them to their sums
         *
         * The fraction of time the vehicle is on the lane is computed and
         *  used as a weight for the vehicle's current values.
         *  The "emitted" field is incremented, additionally.
         *
         * @param[in] veh The entering vehicle.
         * @param[in] reason how the vehicle enters the lane
         * @see MSMoveReminder::notifyEnter
         * @return Always true
         */
        bool notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason);
        //@}

        bool isEmpty() const;

        /** @brief Writes output values into the given stream
         *
         * @param[in] dev The output device to write the data into
         * @param[in] period Length of the period the data were gathered
         * @param[in] numLanes The total number of lanes for which the data was collected
         * @exception IOError If an error on writing occurs (!!! not yet implemented)
         */
        void write(OutputDevice& dev, const SUMOTime period,
                   const SUMOReal numLanes, const SUMOReal defaultTravelTime,
                   const int numVehicles = -1) const;

        int getNumReady() const;

        void clearFirst();

        SUMOReal getSamples() const;

    private:
        class TrackerEntry {
        public:
            /** @brief Constructor */
            TrackerEntry(MeanDataValues* const values)
                : myNumVehicleEntered(0), myNumVehicleLeft(0), myValues(values) {}

            /** @brief Constructor */
            virtual ~TrackerEntry() {
                delete myValues;
            }

            /// @brief The number of vehicles which entered in the current interval
            int myNumVehicleEntered;

            /// @brief The number of vehicles which left in the current interval
            int myNumVehicleLeft;

            /// @brief The number of vehicles which left in the current interval
            MeanDataValues* myValues;
        };

        /// @brief The map of vehicles to data entries
        std::map<SUMOVehicle*, TrackerEntry*> myTrackedData;

        /// @brief The currently active meandata "intervals"
        std::list<TrackerEntry*> myCurrentData;

        /// @brief The meandata parent
        const MSMeanData* myParent;

    };


public:
    /** @brief Constructor
     *
     * @param[in] id The id of the detector
     * @param[in] dumpBegin Begin time of dump
     * @param[in] dumpEnd End time of dump
     * @param[in] useLanes Information whether lane-based or edge-based dump shall be generated
     * @param[in] withEmpty Information whether empty lanes/edges shall be written
     * @param[in] withInternal Information whether internal lanes/edges shall be written
     * @param[in] trackVehicles Information whether vehicles shall be tracked
     * @param[in] maxTravelTime the maximum travel time to use when calculating per vehicle output
     * @param[in] defaultEffort the value to use when calculating defaults
     * @param[in] minSamples the minimum number of sample seconds before the values are valid
     * @param[in] vTypes the set of vehicle types to consider
     */
    MSMeanData(const std::string& id,
               const SUMOTime dumpBegin, const SUMOTime dumpEnd,
               const bool useLanes, const bool withEmpty,
               const bool printDefaults, const bool withInternal,
               const bool trackVehicles, const SUMOReal minSamples,
               const SUMOReal maxTravelTime,
               const std::set<std::string> vTypes);


    /// @brief Destructor
    virtual ~MSMeanData();

    /** @brief Adds the value collectors to all relevant edges.
     */
    void init();

    /// @name Methods inherited from MSDetectorFileOutput.
    /// @{

    /** @brief Writes collected values into the given stream
     *
     * At first, it is checked whether the values for the current interval shall be written.
     *  If not, a reset is performed, only, using "resetOnly". Otherwise,
     *  both the list of single-lane edges and the list of multi-lane edges
     *  are gone through and each edge is written using "writeEdge".
     *
     * @param[in] dev The output device to write the data into
     * @param[in] startTime First time step the data were gathered
     * @param[in] stopTime Last time step the data were gathered
     * @see MSDetectorFileOutput::writeXMLOutput
     * @see write
     * @exception IOError If an error on writing occurs (!!! not yet implemented)
     */
    void writeXMLOutput(OutputDevice& dev, SUMOTime startTime, SUMOTime stopTime);

    /** @brief Opens the XML-output using "netstats" as root element
     *
     * @param[in] dev The output device to write the root into
     * @see MSDetectorFileOutput::writeXMLDetectorProlog
     * @exception IOError If an error on writing occurs (!!! not yet implemented)
     */
    virtual void writeXMLDetectorProlog(OutputDevice& dev) const;
    /// @}

    /** @brief Updates the detector
     */
    virtual void detectorUpdate(const SUMOTime step);


protected:
    /** @brief Create an instance of MeanDataValues
     *
     * @param[in] lane The lane to create for
     * @param[in] doAdd whether to add the values as reminder to the lane
     */
    virtual MSMeanData::MeanDataValues* createValues(MSLane* const lane, const SUMOReal length, const bool doAdd) const = 0;

    /** @brief Resets network value in order to allow processing of the next interval
     *
     * Goes through the lists of edges and starts "resetOnly" for each edge.
     * @param[in] edge The last time step that is reported
     */
    void resetOnly(SUMOTime stopTime);

    /** @brief Return the relevant edge id
     *
     * @param[in] edge The edge to retrieve the id for
     */
    virtual std::string getEdgeID(const MSEdge* const edge);

    /** @brief Writes edge values into the given stream
     *
     * microsim: It is checked whether the dump shall be generated edge-
     *  or lane-wise. In the first case, the lane-data are collected
     *  and aggregated and written directly. In the second case, "writeLane"
     *  is used to write each lane's state.
     *
     * @param[in] dev The output device to write the data into
     * @param[in] edgeValues List of this edge's value collectors
     * @param[in] edge The edge to write the dump of
     * @param[in] startTime First time step the data were gathered
     * @param[in] stopTime Last time step the data were gathered
     * @exception IOError If an error on writing occurs (!!! not yet implemented)
     */
    void writeEdge(OutputDevice& dev, const std::vector<MeanDataValues*>& edgeValues,
                   MSEdge* edge, SUMOTime startTime, SUMOTime stopTime);

    /** @brief Writes the interval opener
     *
     * @param[in] dev The output device to write the data into
     * @param[in] startTime First time step the data were gathered
     * @param[in] stopTime Last time step the data were gathered
     */
    virtual void openInterval(OutputDevice& dev, const SUMOTime startTime, const SUMOTime stopTime);

    /** @brief Checks for emptiness and writes prefix into the given stream
     *
     * @param[in] dev The output device to write the data into
     * @param[in] values The values to check for emptiness
     * @param[in] tag The xml tag to write (lane / edge)
     * @param[in] id The id for the lane / edge to write
     * @return whether further output should be generated
     * @exception IOError If an error on writing occurs (!!! not yet implemented)
     */
    virtual bool writePrefix(OutputDevice& dev, const MeanDataValues& values,
                             const SumoXMLTag tag, const std::string id) const;

protected:
    /// @brief the minimum sample seconds
    const SUMOReal myMinSamples;

    /// @brief the maximum travel time to write
    const SUMOReal myMaxTravelTime;

    /// @brief The vehicle types to look for (empty means all)
    const std::set<std::string> myVehicleTypes;

    /// @brief Value collectors; sorted by edge, then by lane
    std::vector<std::vector<MeanDataValues*> > myMeasures;

    /// @brief Whether empty lanes/edges shall be written
    const bool myDumpEmpty;

private:
    /// @brief Information whether the output shall be edge-based (not lane-based)
    const bool myAmEdgeBased;

    /// @brief The first and the last time step to write information (-1 indicates always)
    const SUMOTime myDumpBegin, myDumpEnd;

    /// @brief The corresponding first edges
    MSEdgeVector myEdges;

    /// @brief Whether empty lanes/edges shall be written
    const bool myPrintDefaults;

    /// @brief Whether internal lanes/edges shall be written
    const bool myDumpInternal;

    /// @brief Whether vehicles are tracked
    const bool myTrackVehicles;

    /// @brief The intervals for which output still has to be generated (only in the tracking case)
    std::list< std::pair<SUMOTime, SUMOTime> > myPendingIntervals;

private:
    /// @brief Invalidated copy constructor.
    MSMeanData(const MSMeanData&);

    /// @brief Invalidated assignment operator.
    MSMeanData& operator=(const MSMeanData&);

};


#endif

/****************************************************************************/

