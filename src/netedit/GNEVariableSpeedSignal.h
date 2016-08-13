/****************************************************************************/
/// @file    GNEVariableSpeedSignal.h
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
/// @version $Id: GNEVariableSpeedSignal.h 21150 2016-07-12 12:28:35Z behrisch $
///
///
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
#ifndef GNEVariableSpeedSignal_h
#define GNEVariableSpeedSignal_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "GNEAdditionalSet.h"

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GNEVariableSpeedSignal
 * ------------
 */
class GNEVariableSpeedSignal : public GNEAdditionalSet {
public:

    /**@brief Constructor
     * @param[in] id The storage of gl-ids to get the one for this lane representation from
     * @param[in] viewNet pointer to GNEViewNet of this additional element belongs
     * @param[in] pos position (center) of the rerouter in the map
     * @param[in] lanes vector with the lanes of variable speed signal
     * @param[in] filename The path to the definition file
     * @param[in] VSSValues values with the interval and speeds of variable speed signals
     * @param[in] blocked set initial blocking state of item
     */
    GNEVariableSpeedSignal(const std::string& id, GNEViewNet* viewNet, Position pos, std::vector<GNELane*> lanes, const std::string& filename, const std::map<SUMOTime, SUMOReal>& VSSValues, bool blocked);

    /// @brief Destructor
    ~GNEVariableSpeedSignal();

    /// @brief update pre-computed geometry information
    /// @note: must be called when geometry changes (i.e. lane moved)
    void updateGeometry();

    /// @brief Returns position of Variable Speed Signal in view
    Position getPositionInView() const;

    /// @brief open GNEVariableSpeedSignalDialog
    void openAdditionalDialog();

    /**@brief change the position of the rerouter geometry
     * @param[in] posx new x position of rerouter in the map
     * @param[in] posy new y position of rerouter in the map
     * @param[in] undoList pointer to the undo list
     */
    void moveAdditional(SUMOReal posx, SUMOReal posy, GNEUndoList* undoList);

    /**@brief writte additional element into a xml file
     * @param[in] device device in which write parameters of additional element
     * @param[in] currentDirectory current directory in which this additional are writted
     */
    void writeAdditional(OutputDevice& device, const std::string& currentDirectory);

    /// @brief get filename of rerouter
    std::string getFilename() const;

    /// @brief get values of variable speed signal
    std::map<SUMOTime, SUMOReal> getVariableSpeedSignalSteps() const;

    /// @brief set filename of rerouter
    void setFilename(std::string filename);

    /// @brief set values of variable speed signal
    void setVariableSpeedSignalSteps(const std::map<SUMOTime, SUMOReal>& vssValues);

    /// @brief insert a new step in variable speed signal
    /// @return true if step was sucesfully inserted, false in other case (Time duplicated)
    bool insertStep(const SUMOTime time, const SUMOReal speed);

    /// @name inherited from GUIGlObject
    /// @{
    /// @brief Returns the name of the parent object
    /// @return This object's parent id
    const std::string& getParentName() const;

    /**@brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    /// @}

    /// @name inherited from GNEAttributeCarrier
    /// @{
    /* @brief method for getting the Attribute of an XML key
     * @param[in] key The attribute key
     * @return string with the value associated to key
     */
    std::string getAttribute(SumoXMLAttr key) const;

    /* @brief method for setting the attribute and letting the object perform additional changes
     * @param[in] key The attribute key
     * @param[in] value The new value
     * @param[in] undoList The undoList on which to register changes
     */
    void setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList);

    /* @brief method for checking if the key and their correspond attribute are valids
     * @param[in] key The attribute key
     * @param[in] value The value asociated to key key
     * @return true if the value is valid, false in other case
     */
    bool isValid(SumoXMLAttr key, const std::string& value);
    /// @}

protected:
    /// @brief filename of rerouter
    std::string myFilename;

    /// @brief values of variable speed signal
    std::map<SUMOTime, SUMOReal> myVSSValues;

    /// @brief enable or disable save in external filename
    bool mySaveInFilename;

private:
    /// @brief set attribute after validation
    void setAttribute(SumoXMLAttr key, const std::string& value);

    /// @brief Invalidated copy constructor.
    GNEVariableSpeedSignal(const GNEVariableSpeedSignal&);

    /// @brief Invalidated assignment operator.
    GNEVariableSpeedSignal& operator=(const GNEVariableSpeedSignal&);
};

#endif

/****************************************************************************/
