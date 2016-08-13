/****************************************************************************/
/// @file    GNEDetectorE1.h
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
/// @version $Id: GNEDetector.h 21182 2016-07-18 06:46:01Z behrisch $
///
/// A abstract class to define common parameters of detectors
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2001-2013 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef GNEDetector_h
#define GNEDetector_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GNEAdditional.h"

// ===========================================================================
// class declarations
// ===========================================================================
class GNEAdditional;
class GNEViewNet;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEdetector
 * @briefA abstract class to define common parameters and functions of detectors
 */
class GNEDetector : public GNEAdditional {
public:
    /**@brief Constructor.
     * @param[in] id Gl-id of the detector (Must be unique)
     * @param[in] lane Lane of this detector belongs
     * @param[in] viewNet pointer to GNEViewNet of this additional element belongs
     * @param[in] tag Type of xml tag that define the detector (SUMO_TAG_E1DETECTOR, SUMO_TAG_LANE_AREA_DETECTOR, etc...)
     * @param[in] posOverLane position of detector in lane
     * @param[in] freq the aggregation period the values the detector collects shall be summed up.
     * @param[in] filename The path to the output file.
     * @param[in] blocked set initial blocking state of item
     * @param[in] parent pointer to parent, if this additional belongs to an additionalSet
     */
    GNEDetector(const std::string& id, GNEViewNet* viewNet, SumoXMLTag tag, GNELane* lane, SUMOReal posOverLane, int freq, const std::string& filename, bool blocked = false, GNEAdditionalSet* parent = NULL);

    /// @brief Destructor
    ~GNEDetector();

    /// @brief update pre-computed geometry information
    virtual void updateGeometry() = 0;

    /// @brief Returns position of additional in view
    virtual Position getPositionInView() const = 0;

    /**@brief change the position of the additional geometry
     * @param[in] posx new x position of additional over lane
     * @param[in] posy unused
     * @param[in] undoList pointer to the undo list
     */
    void moveAdditional(SUMOReal posx, SUMOReal posy, GNEUndoList* undoList);

    /**@brief writte additional element into a xml file
     * @param[in] device device in which write parameters of additional element
     * @param[in] currentDirectory current directory in which this additional are writted
     */
    virtual void writeAdditional(OutputDevice& device, const std::string& currentDirectory) = 0;

    /// @brief Returns pointer to Lane of detector
    GNELane* getLane() const;

    /// @brief Remove reference to Lane of stopping place
    /// @note will be automatic called in lane destructor
    void removeLaneReference();

    /// @brief Returns the position of the detector over lane
    SUMOReal getPositionOverLane() const;

    /// @brief returns the aggregation period the values the detector collects shall be summed up.
    int getFrequency() const;

    /// @brief returns the path to the output file
    std::string getFilename() const;

    /**@brief Set a new position of detector over lane
     * @param[in] pos new position of detector over lane
     * @throws InvalidArgument if value of pos isn't valid
     */
    void setPositionOverLane(SUMOReal pos);

    /**@brief Set a new frequency in detector
     * @param[in] freq new frequency of detector
     * @throws InvalidArgument if value of frequency isn't valid
     */
    void setFrequency(int freq);

    /**@brief Set a new filename in detector
     * @param[in] filename new filename of detector
     */
    void setFilename(std::string filename);

    /// @brief change lane of detector
    void changeLane(GNELane* newLane);

    /// @name inherited from GNEAdditional
    /// @{
    /// @brief Returns the name of the parent object
    /// @return This object's parent id
    const std::string& getParentName() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    virtual void drawGL(const GUIVisualizationSettings& s) const = 0;
    /// @}

    /// @name inherited from GNEAttributeCarrier
    /// @{
    /* @brief method for getting the Attribute of an XML key
     * @param[in] key The attribute key
     * @return string with the value associated to key
     */
    virtual std::string getAttribute(SumoXMLAttr key) const = 0;

    /* @brief method for setting the attribute and letting the object perform additional changes
     * @param[in] key The attribute key
     * @param[in] value The new value
     * @param[in] undoList The undoList on which to register changes
     */
    virtual void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) = 0;

    /* @brief method for checking if the key and their conrrespond attribute are valids
     * @param[in] key The attribute key
     * @param[in] value The value asociated to key key
     * @return true if the value is valid, false in other case
     */
    virtual bool isValid(SumoXMLAttr key, const std::string& value) = 0;
    /// @}

protected:
    /// @brief The lane this detector belongs
    GNELane* myLane;

    /// @brief The aggregation period the values the detector collects shall be summed up.
    int myFreq;

    /// @brief The path to the output file
    std::string myFilename;

    /// @name members and functions relative to detector icon
    /// @{
    /// @brief set Rotation of block Icon
    void drawDetectorIcon(const int GNELogoID, SUMOReal sizex = 0.5, SUMOReal sizey = 0.5) const;

    /// @brief The position of detector
    Position myDetectorLogoOffset;
    /// @}

private:
    /// @brief set attribute after validation
    virtual void setAttribute(SumoXMLAttr key, const std::string& value) = 0;

    /// @brief Invalidate return position of additional
    const Position& getPosition() const;

    /// @brief Invalidate set new position in the view
    void setPosition(const Position& pos);
};

#endif

/****************************************************************************/
