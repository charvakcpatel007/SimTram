/****************************************************************************/
/// @file    GUICalibrator.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 26.04.2004
/// @version $Id: GUICalibrator.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Changes flow and speed on a set of lanes (gui version)
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
#ifndef GUICalibrator_h
#define GUICalibrator_h


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
#include <microsim/trigger/MSCalibrator.h>
#include <utils/gui/globjects/GUIGlObject_AbstractAdd.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/foxtools/FXRealSpinDial.h>
#include <gui/GUIManipulator.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUICalibrator
 * @brief Changes the speed allowed on a set of lanes (gui version)
 *
 * This is the gui-version of the MSCalibrator-object
 */
class GUICalibrator : public MSCalibrator, public GUIGlObject_AbstractAdd {
public:
    /** @brief Constructor
     * @param[in] idStorage The gl-id storage for giving this object an gl-id
     * @param[in] id The id of the lane speed trigger
     * @param[in] destLanes List of lanes affected by this speed trigger
     * @param[in] file Name of the file to read the speeds to set from
     */
    GUICalibrator(const std::string& id,
                  MSEdge* edge, SUMOReal pos,
                  const std::string& aXMLFilename,
                  const std::string& outputFilename,
                  const SUMOTime freq,
                  const MSRouteProbe* probe);


    /** destructor */
    ~GUICalibrator();



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



    GUIManipulator* openManipulator(GUIMainWindow& app,
                                    GUISUMOAbstractView& parent);

public:
    class GUICalibratorPopupMenu : public GUIGLObjectPopupMenu {
        FXDECLARE(GUICalibratorPopupMenu)
    public:

        GUICalibratorPopupMenu(GUIMainWindow& app,
                               GUISUMOAbstractView& parent, GUIGlObject& o);

        ~GUICalibratorPopupMenu();

        /** @brief Called if the object's manipulator shall be shown */
        long onCmdOpenManip(FXObject*, FXSelector, void*);

    protected:
        GUICalibratorPopupMenu() { }

    };

    class GUIManip_Calibrator : public GUIManipulator {
        FXDECLARE(GUIManip_Calibrator)
    public:
        enum {
            MID_USER_DEF = FXDialogBox::ID_LAST,
            MID_PRE_DEF,
            MID_OPTION,
            MID_CLOSE,
            ID_LAST
        };
        /// Constructor
        GUIManip_Calibrator(GUIMainWindow& app,
                            const std::string& name, GUICalibrator& o,
                            int xpos, int ypos);

        /// Destructor
        virtual ~GUIManip_Calibrator();

        long onCmdOverride(FXObject*, FXSelector, void*);
        long onCmdClose(FXObject*, FXSelector, void*);
        long onCmdUserDef(FXObject*, FXSelector, void*);
        long onUpdUserDef(FXObject*, FXSelector, void*);
        long onCmdPreDef(FXObject*, FXSelector, void*);
        long onUpdPreDef(FXObject*, FXSelector, void*);
        long onCmdChangeOption(FXObject*, FXSelector, void*);

    private:
        GUIMainWindow* myParent;

        FXint myChosenValue;

        FXDataTarget myChosenTarget;

        SUMOReal mySpeed;

        FXDataTarget mySpeedTarget;

        FXRealSpinDial* myUserDefinedSpeed;

        FXComboBox* myPredefinedValues;

        GUICalibrator* myObject;

    protected:
        GUIManip_Calibrator() { }

    };

private:
    /// Definition of a positions container
    typedef std::vector<Position> PosCont;

    /// Definition of a rotation container
    typedef std::vector<SUMOReal> RotCont;

private:
    /// The positions in full-geometry mode
    PosCont myFGPositions;

    /// The rotations in full-geometry mode
    RotCont myFGRotations;

    /// The boundary of this rerouter
    Boundary myBoundary;

    /// The information whether the speed shall be shown in m/s or km/h
    bool myShowAsKMH;

};


#endif

/****************************************************************************/

