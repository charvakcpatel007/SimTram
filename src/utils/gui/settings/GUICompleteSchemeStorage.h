/****************************************************************************/
/// @file    GUICompleteSchemeStorage.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    :find(mySortedSchemeNames.begin(), mySortedSchemeNames.end(), name)==mySortedSchemeNames.end()) {
/// @version $Id: GUICompleteSchemeStorage.h 21186 2016-07-18 12:04:16Z namdre $
///
// Storage for available visualization settings
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
#ifndef GUICompleteSchemeStorage_h
#define GUICompleteSchemeStorage_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <utils/gui/windows/GUISUMOAbstractView.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUICompleteSchemeStorage
 * @brief Storage for available visualization settings
 */
class GUICompleteSchemeStorage {
public:
    /// @brief Constructor
    GUICompleteSchemeStorage();


    /// @brief Destructor
    ~GUICompleteSchemeStorage();


    /** @brief Adds a visualization scheme
     * @param[in] scheme The visualization scheme to add
     */
    void add(const GUIVisualizationSettings& scheme);


    /** @brief Returns the named scheme
     * @param[in] name The name of the visualization scheme to return
     * @return The named visualization scheme
     */
    GUIVisualizationSettings& get(const std::string& name);


    /** @brief Returns the default scheme
     * @return The default visualization scheme
     */
    GUIVisualizationSettings& getDefault();


    /** @brief Returns the information whether a setting with the given name is stored
     * @param[in] name The name of regarded scheme
     * @return Whether the named scheme is known
     */
    bool contains(const std::string& name) const;


    /** @brief Removes the setting with the given name
     * @param[in] name The name of the scheme to remove
     */
    void remove(const std::string& name);


    /** @brief Makes the scheme with the given name the default
     * @param[in] name The name of the scheme to marks as default
     */
    void setDefault(const std::string& name);


    /** @brief Returns a list of stored settings names
     * @return The names of known schemes
     */
    const std::vector<std::string>& getNames() const;


    /** @brief Returns the number of initial settings
     * @return The number of default schemes
     */
    int getNumInitialSettings() const;


    /** @brief Initialises the storage with some default settings
     * @param[in] app The application
     */
    void init(FXApp* app);


    /** @brief Writes the current scheme into the registry
     * @param[in] app The application
     */
    void writeSettings(FXApp* app);


    /** @brief Makes the given viewport the default
     * @param[in] x The x-offset
     * @param[in] y The y-offset
     * @param[in] z The camera height
     */
    void saveViewport(const SUMOReal x, const SUMOReal y, const SUMOReal z);


    /** @brief Sets the default viewport
     * @param[in] parent the view for which the viewport has to be set
     */
    void setViewport(GUISUMOAbstractView* view);


protected:
    /// @brief A map of settings referenced by their names
    std::map<std::string, GUIVisualizationSettings> mySettings;

    /// @brief List of known setting names
    std::vector<std::string> mySortedSchemeNames;

    /// @brief Name of the default setting
    std::string myDefaultSettingName;

    /// @brief The number of settings which were present at startup
    int myNumInitialSettings;

    /// @brief The default viewport
    Position myLookFrom, myLookAt;


};

extern GUICompleteSchemeStorage gSchemeStorage;


#endif

/****************************************************************************/

