/****************************************************************************/
/// @file    NLDetectorBuilder.h
/// @author  Daniel Krajzewicz
/// @author  Clemens Honomichl
/// @author  Christian Roessel
/// @author  Michael Behrisch
/// @date    Mon, 15 Apr 2002
/// @version $Id: NLDetectorBuilder.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Builds detectors for microsim
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
#ifndef NLDetectorBuilder_h
#define NLDetectorBuilder_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <microsim/output/MSCrossSection.h>
#include <microsim/traffic_lights/MSTLLogicControl.h>
#include <microsim/output/MSE2Collector.h>

// ===========================================================================
// class declarations
// ===========================================================================
class MSDetectorFileOutput;
class MSLane;
class MSEdge;

class MEInductLoop;
class MESegment;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NLDetectorBuilder
 * @brief Builds detectors for microsim
 *
 * The building methods may be overridden, to build guisim-instances of the triggers,
 *  for example.
 */
class NLDetectorBuilder {
public:
    /** @brief Constructor
     *
     * @param[in] net The network to which's detector control built detector shall be added
     */
    NLDetectorBuilder(MSNet& net);


    /// @brief Destructor
    virtual ~NLDetectorBuilder();


    /// @name Value parsing and detector building methods
    /// @{

    /** @brief Builds an e1 detector and adds it to the net
     *
     * Checks the given values, first. If one of the values is invalid
     *  (lane is not known, sampling frequency<=0, position is larger
     *  than lane's length, the id is already in use), an InvalidArgument is thrown.
     *
     * Otherwise the e1 detector is built by calling "createInductLoop".
     *
     * Internally, there is also a distinction whether a mesosim e1 detector
     *  shall be built.
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The name of the lane the detector is placed at
     * @param[in] pos The definition of the position on the lane the detector shall be placed at
     * @param[in] splInterval The aggregation time span the detector shall use
     * @param[in] device The output device the detector shall write into
     * @param[in] friendlyPos Whether the position information shall be used "friendly" (see user docs)
     * @param[in] splitByType Whether vehicle types shall be disaggregated
     * @exception InvalidArgument If one of the values is invalid
     */
    void buildInductLoop(const std::string& id,
                         const std::string& lane, SUMOReal pos, SUMOTime splInterval,
                         const std::string& device, bool friendlyPos, bool splitByType);


    /** @brief Builds an instantenous induction and adds it to the net
     *
     * Checks the given values, first. If one of the values is invalid
     *  (lane is not known, sampling frequency<=0, position is larger
     *  than lane's length, the id is already in use), an InvalidArgument is thrown.
     *
     * Otherwise the e1 detector is built by calling "createInductLoop".
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The name of the lane the detector is placed at
     * @param[in] pos The definition of the position on the lane the detector shall be placed at
     * @param[in] device The output device the detector shall write into
     * @param[in] friendlyPos Whether the position information shall be used "friendly" (see user docs)
     * @exception InvalidArgument If one of the values is invalid
     */
    void buildInstantInductLoop(const std::string& id,
                                const std::string& lane, SUMOReal pos,
                                const std::string& device, bool friendlyPos);


    /** @brief Builds an e2 detector with a fixed interval and adds it to the net
     *
     * Checks the given values, first. If one of the values is invalid
     *  (lane is not known, sampling frequency<=0, position is larger
     *  than lane's length, length is too large, the id is already in use),
     *  an InvalidArgument is thrown.
     *
     * Otherwise the e2 detector is built, either by calling "buildMultiLaneE2Det"
     *  if the detector shall continue on consecutive lanes, or by calling
     *  "buildSingleLaneE2Det" if it is a one-lane detector.
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The name of the lane the detector is placed at
     * @param[in] pos The definition of the position on the lane the detector shall be placed at
     * @param[in] length The definition of the length the detector shall have
     * @param[in] cont Whether the detector shall continue on predeceeding lanes
     * @param[in] splInterval The aggregation time span the detector shall use
     * @param[in] device The output device the detector shall write into
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
     * @param[in] friendlyPos Whether the position information shall be used "friendly" (see user docs)
     * @exception InvalidArgument If one of the values is invalid
     */
    void buildE2Detector(const std::string& id, const std::string& lane, SUMOReal pos, SUMOReal length,
                         bool cont, SUMOTime splInterval, const std::string& device, SUMOTime haltingTimeThreshold,
                         SUMOReal haltingSpeedThreshold, SUMOReal jamDistThreshold,
                         bool friendlyPos);


    /** @brief Builds an e2 detector connected to a lsa
     *
     * Checks the given values, first. If one of the values is invalid
     *  (lane is not known, position is larger than lane's length, length is too large,
     *  the tls is not known, the id is already in use),
     *  an InvalidArgument is thrown.
     *
     * Otherwise the e2 detector is built, either by calling "buildMultiLaneE2Det"
     *  if the detector shall continue on consecutive lanes, or by calling
     *  "buildSingleLaneE2Det" if it is a one-lane detector.
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The name of the lane the detector is placed at
     * @param[in] pos The definition of the position on the lane the detector shall be placed at
     * @param[in] length The definition of the length the detector shall have
     * @param[in] cont Whether the detector shall continue on predeceeding lanes
     * @param[in] tlls The tls the detector is assigned to
     * @param[in] device The output device the detector shall write into
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
     * @param[in] friendlyPos Whether the position information shall be used "friendly" (see user docs)
     * @exception InvalidArgument If one of the values is invalid
     */
    void buildE2Detector(const std::string& id, const std::string& lane, SUMOReal pos, SUMOReal length,
                         bool cont, MSTLLogicControl::TLSLogicVariants& tlls,
                         const std::string& device, SUMOTime haltingTimeThreshold,
                         SUMOReal haltingSpeedThreshold, SUMOReal jamDistThreshold,
                         bool friendlyPos);


    /** @brief Builds an e2 detector connected to a link's state
     *
     * Checks the given values, first. If one of the values is invalid
     *  (lane is not known, position is larger than lane's length, length is too large,
     *  the tls or the destination lane is not known, the id is already in use),
     *  an InvalidArgument is thrown.
     *
     * Otherwise the e2 detector is built, either by calling "buildMultiLaneE2Det"
     *  if the detector shall continue on consecutive lanes, or by calling
     *  "buildSingleLaneE2Det" if it is a one-lane detector.
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The name of the lane the detector is placed at
     * @param[in] pos The definition of the position on the lane the detector shall be placed at
     * @param[in] length The definition of the length the detector shall have
     * @param[in] cont Whether the detector shall continue on predeceeding lanes
     * @param[in] tlls The tls the detector is assigned to
     * @param[in] tolane The name of the lane to which the link to which the detector to build shall be assigned to points
     * @param[in] device The output device the detector shall write into
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
     * @param[in] friendlyPos Whether the position information shall be used "friendly" (see user docs)
     * @exception InvalidArgument If one of the values is invalid
     */
    void buildE2Detector(const std::string& id, const std::string& lane, SUMOReal pos, SUMOReal length,
                         bool cont, MSTLLogicControl::TLSLogicVariants& tlls, const std::string& tolane,
                         const std::string& device, SUMOTime haltingTimeThreshold,
                         SUMOReal haltingSpeedThreshold, SUMOReal jamDistThreshold,
                         bool friendlyPos);


    /** @brief Stores temporary the initial information about an e3 detector to build
     *
     * If the given sample interval is < 0, an InvalidArgument is thrown. Otherwise,
     *  the values are stored in a new instance of E3DetectorDefinition within
     *  "myE3Definition".
     *
     * @param[in] id The id the detector shall have
     * @param[in] device The output device the detector shall write into
     * @param[in] splInterval The aggregation time span the detector shall use
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @exception InvalidArgument If one of the values is invalid
     */
    void beginE3Detector(const std::string& id, const std::string& device, SUMOTime splInterval,
                         SUMOReal haltingSpeedThreshold, SUMOTime haltingTimeThreshold);


    /** @brief Builds an entry point of an e3 detector
     *
     * If the lane is not known or the position information is not within the lane,
     *  an InvalidArgument is thrown. Otherwise a MSCrossSection is built
     *  using the obtained values and added to the list of entries of the e3 definition
     *  stored in "myE3Definition".
     *
     * @param[in] lane The id of the lane the entry shall be placed at
     * @param[in] pos The position on the lane the entry shall be placed at
     * @param[in] friendlyPos Whether the position information shall be used "friendly" (see user docs)
     * @exception InvalidArgument If one of the values is invalid
     */
    void addE3Entry(const std::string& lane, SUMOReal pos, bool friendlyPos);


    /** @brief Builds an exit point of an e3 detector
     *
     * If the lane is not known or the position information is not within the lane,
     *  an InvalidArgument is thrown. Otherwise a MSCrossSection is built
     *  using the obtained values and added to the list of exits of the e3 definition
     *  stored in "myE3Definition".
     *
     * @param[in] lane The id of the lane the exit shall be placed at
     * @param[in] pos The position on the lane the exit shall be placed at
     * @param[in] friendlyPos Whether the position information shall be used "friendly" (see user docs)
     * @exception InvalidArgument If one of the values is invalid
     */
    void addE3Exit(const std::string& lane, SUMOReal pos, bool friendlyPos);


    /** @brief Builds of an e3 detector using collected values
     *
     * The parameter collected are used to build an e3 detector using
     *  "createE3Detector". The resulting detector is added to the net.
     *
     * @param[in] lane The id of the lane the exit shall be placed at
     * @param[in] pos The position on the lane the exit shall be placed at
     * @exception InvalidArgument If one of the values is invalid
     */
    void endE3Detector();


    /** @brief Returns the id of the currently built e3 detector
     *
     * This is used for error-message generation only. If no id is known,
     *  "<unknown>" is returned.
     *
     * @return The id of the currently processed e3 detector
     */
    std::string getCurrentE3ID() const;


    /** @brief Builds a vTypeProbe and adds it to the net
     *
     * Checks the given values, first. If one of the values is invalid
     *  (sampling frequency<=0), an InvalidArgument is thrown.
     *
     * Otherwise the vTypeProbe is built (directly).
     *
     * @param[in] id The id the detector shall have
     * @param[in] vtype The name of the vehicle type the detector shall observe
     * @param[in] frequency The reporting frequency
     * @param[in] device The output device the detector shall write into
     * @exception InvalidArgument If one of the values is invalid
     */
    void buildVTypeProbe(const std::string& id,
                         const std::string& vtype, SUMOTime frequency,
                         const std::string& device);


    /** @brief Builds a routeProbe and adds it to the net
     *
     * Checks the given values, first. If one of the values is invalid
     *  (sampling frequency<=0), an InvalidArgument is thrown.
     *
     * Otherwise the routeProbe is built (directly).
     *
     * @param[in] id The id the detector shall have
     * @param[in] edge The name of the edge the detector shall observe
     * @param[in] frequency The reporting frequency
     * @param[in] begin The start of the first reporting interval
     * @param[in] device The output device the detector shall write into
     * @exception InvalidArgument If one of the values is invalid
     */
    void buildRouteProbe(const std::string& id, const std::string& edge,
                         SUMOTime frequency, SUMOTime begin,
                         const std::string& device);
    /// @}



    /// @name Detector creating methods
    ///
    /// Virtual, so they may be overwritten, for generating gui-versions of the detectors, for example.
    /// @{

    /** @brief Creates an instance of an e1 detector using the given values
     *
     * Simply calls the MSInductLoop constructor
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The lane the detector is placed at
     * @param[in] pos The position on the lane the detector is placed at
     * @param[in] splitByType Whether additional information split by vehicle classes shall be generated
     * @param[in] show Whether to show the detector in the gui if available
     */
    virtual MSDetectorFileOutput* createInductLoop(const std::string& id,
            MSLane* lane, SUMOReal pos, bool splitByType, bool show = true);


    /** @brief Creates an instance of an e1 detector using the given values
     *
     * Simply calls the MSInductLoop constructor
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The lane the detector is placed at
     * @param[in] pos The position on the lane the detector is placed at
     * @param[in] od The output device the loop shall use
     */
    virtual MSDetectorFileOutput* createInstantInductLoop(const std::string& id,
            MSLane* lane, SUMOReal pos, const std::string& od);

    /** @brief Creates an instance of an e2 detector using the given values
     *
     * Simply calls the MSE2Collector constructor
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The lane the detector is placed at
     * @param[in] pos The position on the lane the detector is placed at
     * @param[in] length The length the detector has
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
     */
    virtual MSE2Collector* createSingleLaneE2Detector(const std::string& id,
            DetectorUsage usage, MSLane* lane, SUMOReal pos, SUMOReal length,
            SUMOTime haltingTimeThreshold,
            SUMOReal haltingSpeedThreshold,
            SUMOReal jamDistThreshold);


    /** @brief Creates an instance of an e2ol-detector using the given values
     *
     * Simply calls the MS_E2_ZS_CollectorOverLanes constructor. After this call,
     *  the detector must be initialised.
     *
     * @param[in] id The id the detector shall have
     * @param[in] lane The lane the detector is placed at
     * @param[in] pos The position on the lane the detector is placed at
     * @param[in] length The length the detector has
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
     */
    virtual MSDetectorFileOutput* createMultiLaneE2Detector(
        const std::string& id, DetectorUsage usage, MSLane* lane, SUMOReal pos,
        SUMOTime haltingTimeThreshold, SUMOReal haltingSpeedThreshold,
        SUMOReal jamDistThreshold);


    /** @brief Creates an instance of an e3 detector using the given values
     *
     * Simply calls the MSE3Collector constructor.
     *
     * @param[in] id The id the detector shall have
     * @param[in] entries The list of this detector's entries
     * @param[in] exits The list of this detector's exits
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     */
    virtual MSDetectorFileOutput* createE3Detector(const std::string& id,
            const CrossSectionVector& entries, const CrossSectionVector& exits,
            SUMOReal haltingSpeedThreshold, SUMOTime haltingTimeThreshold);


    /** @brief Creates edge based mean data collector using the given specification
     *
     * @param[in] id The id the detector shall have
     * @param[in] frequency The aggregation interval the detector shall use
     * @param[in] begin dump begin time
     * @param[in] end dump end time
     * @param[in] type The type of values to be generated
     * @param[in] useLanes Information whether lane-based or edge-based dump shall be generated
     * @param[in] withEmpty Information whether empty lanes/edges shall be written
     * @param[in] withInternal Information whether internal lanes/edges shall be written
     * @param[in] trackVehicles Information whether information shall be collected per vehicle
     * @param[in] maxTravelTime the maximum travel time to output
     * @param[in] minSamples the minimum number of sample seconds before the values are valid
     * @param[in] haltSpeed the maximum speed to consider a vehicle waiting
     * @param[in] vTypes the set of vehicle types to consider
     * @exception InvalidArgument If one of the values is invalid
     */
    void createEdgeLaneMeanData(const std::string& id, SUMOTime frequency,
                                SUMOTime begin, SUMOTime end, const std::string& type,
                                const bool useLanes, const bool withEmpty, const bool printDefaults,
                                const bool withInternal, const bool trackVehicles,
                                const SUMOReal maxTravelTime, const SUMOReal minSamples,
                                const SUMOReal haltSpeed, const std::string& vTypes,
                                const std::string& device);
    /// @}



    /** @brief Builds an e2 detector that lies on only one lane
     *
     * @param[in] id The id the detector shall have
     * @param[in] usage Information how the detector is used within the simulation
     * @param[in] lane The lane the detector is placed at
     * @param[in] pos The position on the lane the detector is placed at
     * @param[in] length The length the detector has
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
     * @todo Check whether this method is really needful
     */
    MSE2Collector* buildSingleLaneE2Det(const std::string& id,
                                        DetectorUsage usage, MSLane* lane, SUMOReal pos, SUMOReal length,
                                        SUMOTime haltingTimeThreshold, SUMOReal haltingSpeedThreshold,
                                        SUMOReal jamDistThreshold);


    /** @brief Builds an e2 detector that continues on preceeding lanes
     *
     * @param[in] id The id the detector shall have
     * @param[in] usage Information how the detector is used within the simulation
     * @param[in] lane The lane the detector is placed at
     * @param[in] pos The position on the lane the detector is placed at
     * @param[in] length The length the detector has
     * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
     * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
     * @param[in] jamDistThreshold Detector parameter: the distance between two vehicles in order to not count them to one jam
     * @todo Check whether this method is really needful
     */
    MSDetectorFileOutput* buildMultiLaneE2Det(const std::string& id, DetectorUsage usage, MSLane* lane, SUMOReal pos, SUMOReal length,
            SUMOTime haltingTimeThreshold, SUMOReal haltingSpeedThreshold,
            SUMOReal jamDistThreshold);




protected:
    /**
     * @class E3DetectorDefinition
     * @brief Holds the incoming definitions of an e3 detector unless the detector is build.
     */
    class E3DetectorDefinition {
    public:
        /** @brief Constructor
         * @param[in] id The id the detector shall have
         * @param[in] device The output device the detector shall write into
         * @param[in] haltingSpeedThreshold Detector parameter: the speed a vehicle's speed must be below to be assigned as jammed
         * @param[in] haltingTimeThreshold Detector parameter: the time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
         * @param[in] splInterval The aggregation time span the detector shall use
         */
        E3DetectorDefinition(const std::string& id,
                             const std::string& device, SUMOReal haltingSpeedThreshold,
                             SUMOTime haltingTimeThreshold, SUMOTime splInterval);

        /// @brief Destructor
        ~E3DetectorDefinition();

        /// @brief The id of the detector
        std::string myID;
        /// @brief The device the detector shall use
        const std::string myDevice;
        /// @brief The speed a vehicle's speed must be below to be assigned as jammed
        SUMOReal myHaltingSpeedThreshold;
        /// @brief The time a vehicle's speed must be below haltingSpeedThreshold to be assigned as jammed
        SUMOTime myHaltingTimeThreshold;
        /// @brief List of detector's entries
        CrossSectionVector myEntries;
        /// @brief List of detector's exits
        CrossSectionVector myExits;
        /// @brief The aggregation interval
        SUMOTime mySampleInterval;
        //@}

    private:
        /// @brief Invalidated copy constructor.
        E3DetectorDefinition(const E3DetectorDefinition&);

        /// @brief Invalidated assignment operator.
        E3DetectorDefinition& operator=(const E3DetectorDefinition&);

    };


protected:
    /** @brief Computes the position to use
     *
     * At first, it is checked whether the given position is negative. If so, the
     *  position is added to the lane's length to obtain the position counted
     *  backwards.
     *
     * If the resulting position is beyond or in front (<0) of the lane, it is either
     *  set to the according lane's boundary (.1 or length-.1) if friendlyPos
     *  is set, or, if friendlyPos is not set, an InvalidArgument is thrown.
     *
     * @param[in] pos Definition of the position on the lane
     * @param[in] lane The lane the position must be valid for
     * @param[in] friendlyPos Whether false positions shall be made acceptable
     * @param[in] detid The id of the currently built detector (for error message generation)
     * @exception InvalidArgument If the defined position is invalid
     */
    SUMOReal getPositionChecking(SUMOReal pos, MSLane* lane, bool friendlyPos,
                                 const std::string& detid);


    /** @brief Converts the length and the position information for an e2 detector
     *
     * @param[in] id The id of the currently built detector (for error message generation)
     * @param[in] clane The lane the detector is placed at
     * @param[in, out] pos The position definition to convert
     * @param[in, out] length The length definition to convert
     * @exception InvalidArgument If the defined position or the defined length is invalid
     */
    void convUncontE2PosLength(const std::string& id, MSLane* clane,
                               SUMOReal& pos, SUMOReal& length, bool frinedly_pos);


    /** @brief Converts the length and the position information for an e2ol-detector
     *
     * @param[in] id The id of the currently built detector (for error message generation)
     * @param[in] clane The lane the detector is placed at
     * @param[in, out] pos The position definition to convert
     * @param[in, out] length The length definition to convert
     * @exception InvalidArgument If the defined position or the defined length is invalid
     */
    void convContE2PosLength(const std::string& id, MSLane* clane,
                             SUMOReal& pos, SUMOReal& length, bool frinedly_pos);



    /// @name Value checking/adapting methods
    /// @{

    /** @brief Returns the named edge
     * @param[in] edgeID The id of the lane
     * @param[in] type The type of the detector (for error message generation)
     * @param[in] detid The id of the currently built detector (for error message generation)
     * @exception InvalidArgument If the named edge is not known
     */
    MSEdge* getEdgeChecking(const std::string& edgeID, SumoXMLTag type,
                            const std::string& detid);


    /** @brief Returns the named lane
     * @param[in] laneID The id of the lane
     * @param[in] type The type of the detector (for error message generation)
     * @param[in] detid The id of the currently built detector (for error message generation)
     * @exception InvalidArgument If the named lane is not known
     */
    MSLane* getLaneChecking(const std::string& laneID, SumoXMLTag type,
                            const std::string& detid);


    /** @brief Checks whether the given frequency (sample interval) is valid
     * @param[in] splInterval The sample interval
     * @param[in] type The type of the detector (for error message generation)
     * @param[in] id The id of the detector (for error message generation)
     * @exception InvalidArgument If the given sample interval is invalid (<=0)
     * @todo Why is splInterval an int???
     */
    void checkSampleInterval(SUMOTime splInterval, SumoXMLTag type, const std::string& id);
    /// @}


protected:
    /// @brief The net to fill
    MSNet& myNet;


private:
    /// @brief definition of the currently parsed e3 detector
    E3DetectorDefinition* myE3Definition;


private:
    /// @brief Invalidated copy constructor.
    NLDetectorBuilder(const NLDetectorBuilder&);

    /// @brief Invalidated assignment operator.
    NLDetectorBuilder& operator=(const NLDetectorBuilder&);

};


#endif

/****************************************************************************/

