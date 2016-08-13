/****************************************************************************/
/// @file    GUITLLogicPhasesTrackerWindow.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Oct/Nov 2003
/// @version $Id: GUITLLogicPhasesTrackerWindow.h 21182 2016-07-18 06:46:01Z behrisch $
///
// A window displaying the phase diagram of a tl-logic
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
#ifndef GUITLLogicPhasesTrackerWindow_h
#define GUITLLogicPhasesTrackerWindow_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <string>
#include <bitset>
#include <utility>
#include <fx.h>
#include <fx3d.h>
#include <microsim/traffic_lights/MSSimpleTrafficLightLogic.h>
#include <microsim/traffic_lights/MSTLLogicControl.h>
#include <utils/common/ValueRetriever.h>
#include <utils/gui/div/GLObjectValuePassConnector.h>
#include <microsim/logging/FunctionBinding.h>
#include <utils/common/SUMOTime.h>
#include <utils/foxtools/FXRealSpinDial.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GUIMainWindow;
class MSTrafficLightLogic;
class GUITrafficLightLogicWrapper;
class MFXMutex;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUITLLogicPhasesTrackerWindow
 * @brief This window displays a phase diagram for a chosen tl-logic.
 */
class GUITLLogicPhasesTrackerWindow
    : public FXMainWindow,
      public ValueRetriever<std::pair<SUMOTime, MSPhaseDefinition> > {
    FXDECLARE(GUITLLogicPhasesTrackerWindow)
public:
    /** @brief Constructor to track current phases
     * @param[in] app The main application window
     * @param[in] logic The logic to track
     * @param[in] wrapper The tracked logic's wrapper
     * @param[in] src The value source to track
     */
    GUITLLogicPhasesTrackerWindow(GUIMainWindow& app,
                                  MSTrafficLightLogic& logic, GUITrafficLightLogicWrapper& wrapper,
                                  ValueSource<std::pair<SUMOTime, MSPhaseDefinition> >* src);


    /** @brief Constructor to show the complete phase diagram
     * @param[in] app The main application window
     * @param[in] logic The logic to show
     * @param[in] wrapper The shown logic's wrapper
     * @param[in] phases The phases to show
     */
    GUITLLogicPhasesTrackerWindow(
        GUIMainWindow& app,
        MSTrafficLightLogic& logic, GUITrafficLightLogicWrapper& wrapper,
        const MSSimpleTrafficLightLogic::Phases& phases);


    /// @brief Destructor
    ~GUITLLogicPhasesTrackerWindow();


    /// @brief Creates the window (FOX-Toolkit)
    void create();


    /** @brief Adds a further phase definition
     * @param[in] def The definition to add
     */
    void addValue(std::pair<SUMOTime, MSPhaseDefinition> def);


    /** @brief Sets the time the display shall be shown as beginning at
     * @param[in] time The time to start to show the phases from
     */
    void setBeginTime(SUMOTime time);


    /// @name FOX-callbacks
    /// {

    /// @brief called on size change
    long onConfigure(FXObject* sender, FXSelector sel, void* data);

    /// @brief called if the widget shall be repainted
    long onPaint(FXObject* sender, FXSelector sel, void* data);

    /// @brief called on a simulation step
    long onSimStep(FXObject* sender, FXSelector sel, void* data);
    /// }


public:
    /// @brief Definition of a storage for phases
    typedef std::vector<MSPhaseDefinition> PhasesVector;

    /// @brief Definition of a storage for durations
    typedef std::vector<SUMOTime> DurationsVector;


    /**
     * @class GUITLLogicPhasesTrackerPanel
     * @brief The canvas for the visualisation of phases
     *
     * The drawing itself id done by the parent.
     */
    class GUITLLogicPhasesTrackerPanel : public FXGLCanvas {
        FXDECLARE(GUITLLogicPhasesTrackerPanel)
    public:
        /** @brief Constructor
         * @param[in] c The container for this panel
         * @param[in] app The main application window
         * @param[in] parent This panel's logial parent
         */
        GUITLLogicPhasesTrackerPanel(FXComposite* c,
                                     GUIMainWindow& app, GUITLLogicPhasesTrackerWindow& parent);

        /// @brief Destructor
        ~GUITLLogicPhasesTrackerPanel();

        /// needed to update
        friend class GUITLLogicPhasesTrackerWindow;


        /// @name FOX-callbacks
        /// {

        /// @brief called on size change
        long onConfigure(FXObject*, FXSelector, void*);

        /// @brief called if the widget shall be repainted
        long onPaint(FXObject*, FXSelector, void*);
        /// }


    private:
        /// @brief The parent window
        GUITLLogicPhasesTrackerWindow* myParent;

    protected:
        /// @brief protected constructor for FOX
        GUITLLogicPhasesTrackerPanel() { }

    };


    /** @brief Draws all values
     * @param[in] The target panel
     */
    void drawValues(GUITLLogicPhasesTrackerPanel& caller);


private:
    /// @brief The main application
    GUIMainWindow* myApplication;

    /// @brief The logic to display
    MSTrafficLightLogic* myTLLogic;

    /// @brief The list of phases
    PhasesVector myPhases;

    /// @brief The list of phase durations
    DurationsVector myDurations;

    /// @brief The panel to draw on
    GUITLLogicPhasesTrackerPanel* myPanel;

    /// @brief A lock to avoid addition of new values while drawing
    MFXMutex myLock;

    /** @brief The names of links
     *
     * This holds an enumeration only - used to avoid time consuming string representation of ints */
    std::vector<std::string> myLinkNames;

    /// @brief The index of the first phase that fits into the window
    int myFirstPhase2Show;

    /// @brief The offset to draw the first phase (left offset)
    int myFirstPhaseOffset;

    /// @brief The time the diagram begins at
    SUMOTime myFirstTime2Show;

    /// @brief The first time a phase was added at
    SUMOTime myBeginTime;

    /// @brief The last time a phase was added at
    SUMOTime myLastTime;

    /// @brief The connector for retrieval of further phases
    GLObjectValuePassConnector<std::pair<SUMOTime, MSPhaseDefinition> >* myConnector;

    /// @brief Information whether the tracking mode is on
    bool myAmInTrackingMode;

    /// @brief The tool bar drag (tracking mode)
    FXToolBarShell* myToolBarDrag;

    /// @brief The tool bar (tracking mode)
    FXToolBar* myToolBar;

    /// @brief The offset changer (tracking mode)
    FXRealSpinDial* myBeginOffset;


protected:
    /// protected constructor for FOX
    GUITLLogicPhasesTrackerWindow() { }


};


#endif

/****************************************************************************/

