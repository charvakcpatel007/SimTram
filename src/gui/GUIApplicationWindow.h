/****************************************************************************/
/// @file    GUIApplicationWindow.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Christian Roessel
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: GUIApplicationWindow.h 21182 2016-07-18 06:46:01Z behrisch $
///
// The main window of the SUMO-gui.
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
#ifndef GUIApplicationWindow_h
#define GUIApplicationWindow_h


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
#include <iostream>
#include <fx.h>
#include <utils/foxtools/MFXEventQue.h>
#include <utils/foxtools/FXThreadEvent.h>
#include <utils/foxtools/MFXInterThreadEventClient.h>
#include <utils/foxtools/FXRealSpinDial.h>
#include <utils/foxtools/FXLCDLabel.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/common/ValueRetriever.h>
#include <utils/common/ValueSource.h>
#include <utils/common/RandomDistributor.h>
#include "GUISUMOViewParent.h"


// ===========================================================================
// class declarations
// ===========================================================================
class GUILoadThread;
class GUIRunThread;
class GUIMessageWindow;
class GUIEvent;
class GUIParameterTracker;
class GUIParameterTableWindow;


// ===========================================================================
// class definition
// ===========================================================================
/**
 * @class GUIApplicationWindow
 * @brief The main window of the SUMO-gui.
 *
 * Beside views on the simulation, shown within a MDI-window, the main window
 * may also have some further views (children) assigned which are stored
 * within a separate list.
 */
class GUIApplicationWindow :
    public GUIMainWindow, public MFXInterThreadEventClient {
    // FOX-declarations
    FXDECLARE(GUIApplicationWindow)
public:

    /** @brief Constructor
     * @param[in] a The FOX application
     * @param[in] configPattern The pattern used for loading configurations
     */
    GUIApplicationWindow(FXApp* a, const std::string& configPattern);


    /// @brief Destructor
    virtual ~GUIApplicationWindow();


    /// @name FOX-interactions
    /// {

    /// @brief Creates the main window (required by FOX)
    virtual void create();

    /// @brief Detaches the tool/menu bar
    virtual void detach();
    /// @}

    void loadOnStartup();


    void dependentBuild();

    void setStatusBarText(const std::string& text);

    void addRecentFile(const FX::FXString& f, const bool isNet);

    FXGLCanvas* getBuildGLCanvas() const;
    SUMOTime getCurrentSimTime() const;

    FXCursor* getDefaultCursor();




    /// @name Inter-thread event handling
    /// @{

    virtual void eventOccured();
    void handleEvent_SimulationLoaded(GUIEvent* e);
    void handleEvent_SimulationStep(GUIEvent* e);
    void handleEvent_Message(GUIEvent* e);
    void handleEvent_SimulationEnded(GUIEvent* e);
    void handleEvent_Screenshot(GUIEvent* e);
    /// @}



    /// @name FOX-callbacks
    /// @{

    /// @brief Called on menu File->Open Configuration
    long onCmdOpenConfiguration(FXObject*, FXSelector, void*);

    /// @brief Called on menu File->Open Network
    long onCmdOpenNetwork(FXObject*, FXSelector, void*);

    /// @brief Called on menu File->Load Shapes
    long onCmdOpenShapes(FXObject*, FXSelector, void*);

    /// @brief Called on reload
    long onCmdReload(FXObject*, FXSelector, void*);

    /// @brief Called on opening a recent file
    long onCmdOpenRecent(FXObject*, FXSelector, void*);

    /// @brief Called on menu File->Close
    long onCmdClose(FXObject*, FXSelector, void*);

    /** @brief Called by FOX if the application shall be closed
     *
     * Called either by FileMenu->Quit, the normal close-menu or SIGINT */
    long onCmdQuit(FXObject*, FXSelector, void*);

    /// @brief Called on menu Edit->Edit Chosen
    long onCmdEditChosen(FXObject*, FXSelector, void*);

    /// @brief Called on menu Edit->Edit Breakpoints
    long onCmdEditBreakpoints(FXObject*, FXSelector, void*);

    /// @brief called if the user selects help->Documentation
    long onCmdHelp(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief Called on menu Edit->Netedit
    long onCmdNetedit(FXObject*, FXSelector, void*);

    /// @brief Opens the application settings menu (Settings->Application Settings...)
    long onCmdAppSettings(FXObject*, FXSelector, void*);

    /// @brief Toggle gaming mode
    long onCmdGaming(FXObject*, FXSelector, void*);

    /// @brief Toggle full screen mode
    long onCmdFullScreen(FXObject*, FXSelector, void*);

    /// @brief Toggle listing of internal structures
    long onCmdListInternal(FXObject*, FXSelector, void*);

    /// @brief Shows the about dialog
    long onCmdAbout(FXObject*, FXSelector, void*);

    /// @brief Called on "play"
    long onCmdStart(FXObject*, FXSelector, void*);

    /// @brief Called on "stop"
    long onCmdStop(FXObject*, FXSelector, void*);

    /// @brief Called on "step"
    long onCmdStep(FXObject*, FXSelector, void*);

    /// @brief Called on "time toggle"
    long onCmdTimeToggle(FXObject*, FXSelector, void*);

    /// @brief Called on "delay toggle"
    long onCmdDelayToggle(FXObject*, FXSelector, void*);

    /// @brief Called if a new view shall be opened (2D view)
    long onCmdNewView(FXObject*, FXSelector, void*);

#ifdef HAVE_OSG
    /// @brief Called if a new 3D view shall be opened
    long onCmdNewOSG(FXObject*, FXSelector, void*);
#endif

    /// @brief Determines whether opening is enabled
    long onUpdOpen(FXObject*, FXSelector, void*);

    /// @brief Determines whether reloading is enabled
    long onUpdReload(FXObject*, FXSelector, void*);

    /// @brief Determines whether opening a recent file is enabled
    long onUpdOpenRecent(FXObject*, FXSelector, void*);

    /// @brief Determines whether adding a view is enabled
    long onUpdAddView(FXObject*, FXSelector, void*);

    /// @brief Determines whether "play" is enabled
    long onUpdStart(FXObject* sender, FXSelector, void* ptr);

    /// @brief Determines whether "stop" is enabled
    long onUpdStop(FXObject*, FXSelector, void*);

    /// @brief Determines whether "step" is enabled
    long onUpdStep(FXObject*, FXSelector, void*);

    /// @brief Determines whether some buttons which require an active simulation may be shown
    long onUpdNeedsSimulation(FXObject*, FXSelector, void*);

    /// @brief Called if the message window shall be cleared
    long onCmdClearMsgWindow(FXObject*, FXSelector, void*);

    /// @brief Called on menu commands from the Locator menu
    long onCmdLocate(FXObject*, FXSelector, void*);

    /// @brief Called on commands from the statistic buttons
    long onCmdShowStats(FXObject*, FXSelector, void*);

    /// @brief Called on an event from the loading thread
    long onLoadThreadEvent(FXObject*, FXSelector, void*);

    /// @brief Called on an event from the simulation thread
    long onRunThreadEvent(FXObject*, FXSelector, void*);

    /// @brief Somebody wants our clipped text
    long onClipboardRequest(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief handle keys
    long onKeyPress(FXObject* o, FXSelector sel, void* data);
    long onKeyRelease(FXObject* o, FXSelector sel, void* data);
    /// @}


    /** @brief Returns the simulation delay
     * @return delay in milliseconds
     */
    virtual SUMOReal getDelay() const {
        return mySimDelayTarget->getValue();
    }

    /** @brief Sets the delay of the parent application
     */
    virtual void setDelay(SUMOReal delay) {
        mySimDelayTarget->setValue(delay);
    }

    /** @brief Sends an event from the application thread to the GUI and waits until it is handled
     * @param event the event to send
     */
    virtual void sendBlockingEvent(GUIEvent* event);

protected:
    virtual void addToWindowsMenu(FXMenuPane*) { }

private:
    /** starts to load a simulation */
    void loadConfigOrNet(const std::string& file, bool isNet);

    /** this method closes all windows and deletes the current simulation */
    void closeAllWindows();

    /// @brief updates the simulation time display
    void updateTimeLCD(SUMOTime time);

    /** opens a new simulation display */
    GUISUMOAbstractView* openNewView(GUISUMOViewParent::ViewType vt = GUISUMOViewParent::VIEW_2D_OPENGL);

    /// @brief handles additional game-related events
    void checkGamingEvents();

protected:
    /// FOX needs this for static members
    GUIApplicationWindow() { }

protected:
    /// Builds the menu bar
    virtual void fillMenuBar();

    /// Builds the tool bar
    virtual void buildToolBars();

protected:
    /** the name of the simulation */
    std::string myName;

    /** the thread that loads simulations */
    GUILoadThread* myLoadThread;

    /** the thread that runs simulations */
    GUIRunThread* myRunThread;

    /** the information whether the simulation was started before */
    bool myWasStarted;

    /// The current view number
    int myViewNumber;

    /// information whether the gui is currently loading and the load-options shall be greyed out
    bool myAmLoading;

    /// the submenus
    FXMenuPane* myFileMenu, *myEditMenu, *mySelectByPermissions, *mySettingsMenu,
                *myLocatorMenu, *myControlMenu,
                *myWindowsMenu, *myHelpMenu;

    /// Buttons showing and running values and triggering statistic windows
    std::vector<FXButton*> myStatButtons;

    /// A window to display messages, warnings and error in
    GUIMessageWindow* myMessageWindow;

    /// The splitter that divides the main window into vies and the log window
    FXSplitter* myMainSplitter;

    /// for some menu detaching fun
    FXToolBarShell* myToolBarDrag1, *myToolBarDrag2, *myToolBarDrag3,
                    *myToolBarDrag4, *myToolBarDrag5, *myMenuBarDrag;

    ///
    FXRealSpinDial* mySimDelayTarget;

    /// The alternate simulation delay for toggling
    SUMOTime myAlternateSimDelay;

    /// List of got requests
    MFXEventQue<GUIEvent*> myEvents;

    /// The menu used for the MDI-windows
    FXMDIMenu* myMDIMenu;

    /// The application menu bar
    FXMenuBar* myMenuBar;

    /// The application tool bar
    FXToolBar* myToolBar1, *myToolBar2, *myToolBar3, *myToolBar4, *myToolBar5;

    /// the simulation step display
    FXEX::FXLCDLabel* myLCDLabel;

    /// io-event with the load-thread
    FXEX::FXThreadEvent myLoadThreadEvent;

    /// io-event with the run-thread
    FXEX::FXThreadEvent myRunThreadEvent;

    /// List of recent config files
    FXRecentFiles myRecentConfigs;

    /// List of recent nets
    FXRecentFiles myRecentNets;

    /// Input file pattern
    std::string myConfigPattern;

    bool hadDependentBuild;

    /// @brief whether to show time as hour:minute:second
    bool myShowTimeAsHMS;

    /// @brief whether to show the window in full screen mode
    bool myAmFullScreen;

    /// @brief whether the simulation end was already announced
    bool myHaveNotifiedAboutSimEnd;

    /// @brief the mutex for the waiting semaphore
    FXMutex myEventMutex;

    /// @brief the semaphore when waiting for event completion
    FXCondition myEventCondition;

    /// @name game related things
    /// {
    RandomDistributor<std::string> myJamSounds;
    RandomDistributor<std::string> myCollisionSounds;
    /// @brief waiting time after which vehicles trigger jam sounds
    SUMOReal myJamSoundTime;
    /// @brief A random number generator used to choose a gaming sound
    static MTRand myGamingRNG;
    int myPreviousCollisionNumber;

    /// performance indicators
    FXEX::FXLCDLabel* myWaitingTimeLabel;
    FXEX::FXLCDLabel* myTimeLossLabel;
    SUMOTime myWaitingTime;
    SUMOTime myTimeLoss;
    FXToolBar* myToolBar6, *myToolBar7;
    FXToolBarShell* myToolBarDrag6, *myToolBarDrag7;
    ////}

};


#endif

/****************************************************************************/

