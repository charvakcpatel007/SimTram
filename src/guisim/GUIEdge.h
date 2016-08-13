/****************************************************************************/
/// @file    GUIEdge.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: GUIEdge.h 21182 2016-07-18 06:46:01Z behrisch $
///
// A road/street connecting two junctions (gui-version)
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
#ifndef GUIEdge_h
#define GUIEdge_h


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
#include <microsim/MSEdge.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/gui/settings/GUIPropertySchemeStorage.h>
#include <utils/foxtools/MFXMutex.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MESegment;
class MSBaseVehicle;
class GUILane;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIEdge
 * @brief A road/street connecting two junctions (gui-version)
 *
 * @see MSEdge
 */
class GUIEdge : public MSEdge, public GUIGlObject {
public:
    /** @brief Constructor.
     * @param[in] id The id of the edge
     * @param[in] numericalID The numerical id (index) of the edge
     * @see MSEdge
     */
    GUIEdge(const std::string& id, int numericalID,
            const EdgeBasicFunction function,
            const std::string& streetName, const std::string& edgeType, int priority);


    /// @brief Destructor.
    ~GUIEdge();


    /* @brief Returns the gl-ids of all known edges
     * @param[in] includeInternal Whether to include ids of internal edges
     */
    static std::vector<GUIGlID> getIDs(bool includeInternal);

    /* @brief Returns the combined length of all edges
     * @param[in] includeInternal Whether to include lengths of internal edges
     * @param[in] eachLane Whether to count each lane separately
     */
    static SUMOReal getTotalLength(bool includeInternal, bool eachLane);

    /// Returns the street's geometry
    Boundary getBoundary() const;

    /// returns the enumerated lane (!!! why not private with a friend?)
    MSLane& getLane(int laneNo);



    /** returns the position on the line given by the coordinates where "prev"
        is the length of the line and "wanted" the distance from the begin
        !!! should be within another class */
    static std::pair<SUMOReal, SUMOReal> getLaneOffsets(SUMOReal x1, SUMOReal y1,
            SUMOReal x2, SUMOReal y2, SUMOReal prev, SUMOReal wanted);

    static void fill(std::vector<GUIEdge*>& netsWrappers);



    /// @name inherited from GUIGlObject
    //@{

    /** @brief Returns an own popup-menu
     *
     * @param[in] app The application needed to build the popup-menu
     * @param[in] parent The parent window needed to build the popup-menu
     * @return The built popup-menu
     * @see GUIGlObject::getPopUpMenu
     */
    virtual GUIGLObjectPopupMenu* getPopUpMenu(GUIMainWindow& app,
            GUISUMOAbstractView& parent);


    /** @brief Returns an own parameter window
     *
     * @param[in] app The application needed to build the parameter window
     * @param[in] parent The parent window needed to build the parameter window
     * @return The built parameter window
     * @see GUIGlObject::getParameterWindow
     */
    virtual GUIParameterTableWindow* getParameterWindow(GUIMainWindow& app,
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


    void addPerson(MSTransportable* p) const {
        AbstractMutex::ScopedLocker locker(myLock);
        MSEdge::addPerson(p);
    }

    void removePerson(MSTransportable* p) const {
        AbstractMutex::ScopedLocker locker(myLock);
        MSEdge::removePerson(p);
    }


    void addContainer(MSTransportable* c) const {
        AbstractMutex::ScopedLocker locker(myLock);
        MSEdge::addContainer(c);
    }

    void removeContainer(MSTransportable* c) const {
        AbstractMutex::ScopedLocker locker(myLock);
        MSEdge::removeContainer(c);
    }

    int getVehicleNo() const;
    std::string getVehicleIDs() const;
    SUMOReal getBruttoOccupancy() const;
    SUMOReal getAllowedSpeed() const;
    /// @brief return flow based on meanSpead @note: may produced incorrect results when jammed
    SUMOReal getFlow() const;
    /// @brief return meanSpead divided by allowedSpeed
    SUMOReal getRelativeSpeed() const;

    /// @brief sets the color according to the currente settings
    void setColor(const GUIVisualizationSettings& s) const;

    /// @brief sets the color according to the current scheme index and some edge function
    bool setFunctionalColor(int activeScheme) const;

    /// @brief sets multiple colors according to the current scheme index and edge function
    bool setMultiColor(const GUIColorer& c) const;

    /// @brief gets the color value according to the current scheme index
    SUMOReal getColorValue(int activeScheme) const;

    /// @brief gets the scaling value according to the current scheme index
    SUMOReal getScaleValue(int activeScheme) const;

    /// @brief returns the segment closest to the given position
    MESegment* getSegmentAtPosition(const Position& pos);

    void drawMesoVehicles(const GUIVisualizationSettings& s) const;

    /// @brief grant exclusive access to the mesoscopic state
    void lock() const {
        myLock.lock();
    }

    /// @brief release exclusive access to the mesoscopic state
    void unlock() const {
        myLock.unlock();
    }

    /// @brief close this edge for traffic
    void closeTraffic(const GUILane* lane);

    /// @brief add a rerouter
    void addRerouter();

    /// @brief return segment colors (meso)
    const std::vector<RGBColor>& getSegmentColors() const {
        return mySegmentColors;
    }

    /// The color of the segments (cached)
    mutable std::vector<RGBColor> mySegmentColors;

private:
    /// @brief invalidated copy constructor
    GUIEdge(const GUIEdge& s);

    /// @brief invalidated assignment operator
    GUIEdge& operator=(const GUIEdge& s);


private:
    /// The mutex used to avoid concurrent updates of myPersons/ myContainers
    mutable MFXMutex myLock;

};


#endif

/****************************************************************************/

