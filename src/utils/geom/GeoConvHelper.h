/****************************************************************************/
/// @file    GeoConvHelper.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    2006-08-01
/// @version $Id: GeoConvHelper.h 20433 2016-04-13 08:00:14Z behrisch $
///
// static methods for processing the coordinates conversion for the current net
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
#ifndef GeoConvHelper_h
#define GeoConvHelper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <map>
#include <string>
#include <utils/geom/Position.h>
#include <utils/geom/Boundary.h>

#ifdef HAVE_PROJ
#include <proj_api.h>
#endif


// ===========================================================================
// class declarations
// ===========================================================================
class OptionsCont;
class PositionVector;
class OutputDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GeoConvHelper
 * @brief static methods for processing the coordinates conversion for the current net
 */
class GeoConvHelper {
public:

    /** @brief Constructor based on the stored options
     * @param[in] oc The OptionsCont from which to read options
     */
    GeoConvHelper(OptionsCont& oc);

    /** @brief Constructor
     */
    GeoConvHelper(const std::string& proj, const Position& offset,
                  const Boundary& orig, const Boundary& conv, int shift = 0, bool inverse = false);


    /// @brief Destructor
    ~GeoConvHelper();


    /** @brief Adds projection options to the given container
     *
     * @param[in] oc The options container to add the options to
     * @todo let the container be retrieved
     */
    static void addProjectionOptions(OptionsCont& oc);

    /// Initialises the processing and the final instance using the given options
    static bool init(OptionsCont& oc);

    /// Initialises the processing and the final instance using the given proj.4-definition and complete network parameter
    static void init(const std::string& proj,
                     const Position& offset,
                     const Boundary& orig,
                     const Boundary& conv,
                     int shift = 0);

    /** @brief the coordinate transformation to use for input conversion and processing
     * @note instance is modified during use: boundary may adapt to new coordinates
     */
    static GeoConvHelper& getProcessing() {
        return myProcessing;
    }

    /** @brief the coordinate transformation that was loaded fron an input file
     */
    static GeoConvHelper& getLoaded() {
        return myLoaded;
    }

    /** @brief compute the location attributes which will be used for output
     * based on the loaded location data, the given options and the transformations applied during processing
     */
    static void computeFinal(bool lefthand = false);


    /** @brief the coordinate transformation for writing the location element
     * and for tracking the original coordinate system
     */
    static const GeoConvHelper& getFinal() {
        return myFinal;
    }


    /** @brief sets the coordinate transformation loaded from a location element
     */
    static void setLoaded(const GeoConvHelper& loaded);


    /** @brief resets loaded location elements
     */
    static void resetLoaded();

    /// Converts the given cartesian (shifted) position to its geo (lat/long) representation
    void cartesian2geo(Position& cartesian) const;

    /** Converts the given coordinate into a cartesian
     * and optionally update myConvBoundary
     * @note: initializes UTM / DHDN projection on first use (select zone)
     */
    bool x2cartesian(Position& from, bool includeInBoundary = true);

    /// Converts the given coordinate into a cartesian using the previous initialisation
    bool x2cartesian_const(Position& from) const;

    /// Returns whether a transformation from geo to metric coordinates will be performed
    bool usingGeoProjection() const;

    /// Returns the information whether an inverse transformation will happen
    bool usingInverseGeoProjection() const;

    /// Shifts the converted boundary by the given amounts
    void moveConvertedBy(SUMOReal x, SUMOReal y);

    /// Returns the original boundary
    const Boundary& getOrigBoundary() const;

    /// Returns the converted boundary
    const Boundary& getConvBoundary() const;

    /// sets the converted boundary
    void setConvBoundary(const Boundary& boundary) {
        myConvBoundary = boundary;
    }

    /// Returns the network offset
    const Position getOffset() const;

    /// Returns the network base
    const Position getOffsetBase() const;

    /// Returns the network offset
    const std::string& getProjString() const;

    /// @brief writes the location element
    static void writeLocation(OutputDevice& into);

private:
    enum ProjectionMethod {
        NONE,
        SIMPLE,
        UTM,
        DHDN,
        DHDN_UTM,
        PROJ
    };

    /// A proj options string describing the proj.4-projection to use
    std::string myProjString;

#ifdef HAVE_PROJ
    /// The proj.4-projection to use
    projPJ myProjection;

    /// The inverse proj.4-projection to use first
    projPJ myInverseProjection;

    /// The geo proj.4-projection which is the target of the inverse projection
    projPJ myGeoProjection;
#endif

    /// The offset to apply
    Position myOffset;

    /// The scaling to apply to geo-coordinates
    double myGeoScale;

    /// Information whether no projection shall be done
    ProjectionMethod myProjectionMethod;

    /// Information whether inverse projection shall be used
    bool myUseInverseProjection;

    /// The boundary before conversion (x2cartesian)
    Boundary myOrigBoundary;

    /// The boundary after conversion (x2cartesian)
    Boundary myConvBoundary;

    /// @brief coordinate transformation to use for input conversion and processing
    static GeoConvHelper myProcessing;

    /// @brief coordinate transformation loaded from a location element
    static GeoConvHelper myLoaded;

    /// @brief coordinate transformation to use for writing the location element and for tracking the original coordinate system
    static GeoConvHelper myFinal;

    /// @brief the numer of coordinate transformations loaded from location elements
    static int myNumLoaded;

    /// @brief assignment operator.
    GeoConvHelper& operator=(const GeoConvHelper&);

    /// @brief invalidated copy constructor.
    GeoConvHelper(const GeoConvHelper&);

};


#endif

/****************************************************************************/

