/****************************************************************************/
/// @file    GUITrafficLightLogicWrapper.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Oct/Nov 2003
/// @version $Id: GUITrafficLightLogicWrapper.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A wrapper for tl-logics to allow their visualisation and interaction
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
#ifndef GUITrafficLightLogicWrapper_h
#define GUITrafficLightLogicWrapper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <gui/GUITLLogicPhasesTrackerWindow.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSTrafficLightLogic;
class GUIMainWindow;


// ===========================================================================
// class definition
// ===========================================================================
/**
 * @class GUITrafficLightLogicWrapper
 * This class is responsible for the visualisation of tl-logics and the
 *  interaction with them.
 */
class GUITrafficLightLogicWrapper : public GUIGlObject {
public:
    /// Constructor
    GUITrafficLightLogicWrapper(MSTLLogicControl& control, MSTrafficLightLogic& tll);

    /// Destructor
    ~GUITrafficLightLogicWrapper();



    /// @name inherited from GUIGlObject
    //@{

    /** @brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app,
                                       GUISUMOAbstractView& parent);


    /** @brief Returns an own parameter window
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     * @see GUIGlObject::getParameterWindow
     */
    GUIParameterTableWindow* getParameterWindow(GUIMainWindow& app,
            GUISUMOAbstractView& parent);


    /** @brief Returns the boundary to which the view shall be centered in order to show the object
     *
     * @return The boundary the object is within
     * @see GUIGlObject::getCenteringBoundary
     */
    Boundary getCenteringBoundary() const;


    /** @brief Draws the object
     * @param[in] s The settings for the current view (may influence drawing)
     * @see GUIGlObject::drawGL
     */
    void drawGL(const GUIVisualizationSettings& s) const;
    //@}


    /// Builds a GUITLLogicPhasesTrackerWindow which will receive new phases
    void begin2TrackPhases();

    /// Builds a GUITLLogicPhasesTrackerWindow which displays the phase diagram
    void showPhases();

    /// Builds a GUITLLogicPhasesTrackerWindow which displays the phase diagram
    void switchTLSLogic(int to);

    /// Returns the index of the given link within the according tls
    int getLinkIndex(const MSLink* const link) const;


public:
    /**
     * @class GUITrafficLightLogicWrapperPopupMenu
     * The popup-menu for a TLS-logic. Adds the functionality to open a
     *  view on the tls-logic and to start tracking of the tls-logic.
     */
    class GUITrafficLightLogicWrapperPopupMenu : public GUIGLObjectPopupMenu {
        FXDECLARE(GUITrafficLightLogicWrapperPopupMenu)
    public:
        /// Constructor
        GUITrafficLightLogicWrapperPopupMenu(GUIMainWindow& app,
                                             GUISUMOAbstractView& parent, GUIGlObject& o);

        /// Destructor
        ~GUITrafficLightLogicWrapperPopupMenu();

        /// Called if the phases shall be shown
        long onCmdShowPhases(FXObject*, FXSelector, void*);

        /// Called if the phases shall be begun to track
        long onCmdBegin2TrackPhases(FXObject*, FXSelector, void*);

        long onCmdSwitchTLS2Off(FXObject*, FXSelector, void*);
        long onCmdSwitchTLSLogic(FXObject*, FXSelector, void*);

    protected:
        /// protected constructor for FOX
        GUITrafficLightLogicWrapperPopupMenu() { }

    };

private:
    /// Reference to the according tls
    MSTLLogicControl& myTLLogicControl;

    /// The wrapped tl-logic
    MSTrafficLightLogic& myTLLogic;

    /// The main application
    GUIMainWindow* myApp;

};


#endif

/****************************************************************************/

