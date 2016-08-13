/****************************************************************************/
/// @file    Shape.h
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Oct 2012
/// @version $Id: Shape.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A 2D- or 3D-Shape
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2012-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef Shape_h
#define Shape_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <utils/common/Named.h>
#include <utils/common/RGBColor.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Shape
 * @brief A 2D- or 3D-Shape
 */
class Shape : public Named {
public:
    /** @brief Constructor
     * @param[in] id The name of the shape
     * @param[in] type The (abstract) type of the shape
     * @param[in] color The color of the shape
     * @param[in] layer The layer of the shape
     * @param[in] angle The rotation of the shape in navigational degrees
     * @param[in] imgFile The raster image of the shape
     */
    Shape(const std::string& id, const std::string& type,
          const RGBColor& color, SUMOReal layer,
          SUMOReal angle, const std::string& imgFile);


    /// @brief Destructor
    virtual ~Shape();


    /// @name Getter
    /// @{

    /** @brief Returns the (abstract) type of the Shape
     * @return The Shape's (abstract) type
     */
    inline const std::string& getType() const {
        return myType;
    }


    /** @brief Returns the color of the Shape
     * @return The Shape's color
     */
    inline const RGBColor& getColor() const {
        return myColor;
    }


    /** @brief Returns the layer of the Shape
     * @return The Shape's layer
     */
    inline SUMOReal getLayer() const {
        return myLayer;
    }

    /** @brief Returns the angle of the Shape in navigational degrees
     * @return The Shape's rotation angle
     */
    inline SUMOReal getNaviDegree() const {
        return myNaviDegreeAngle;
    }

    /** @brief Returns the imgFile of the Shape
     * @return The Shape's rotation imgFile
     */
    inline const std::string& getImgFile() const {
        return myImgFile;
    }
    /// @}


    /// @name Setter
    /// @{

    /** @brief Sets a new type
     * @param[in] type The new type to use
     */
    inline void setType(const std::string& type) {
        myType = type;
    }


    /** @brief Sets a new color
     * @param[in] col The new color to use
     */
    inline void setColor(const RGBColor& col) {
        myColor = col;
    }


    /** @brief Sets a new layer
     * @param[in] layer The new layer to use
     */
    inline void setLayer(const SUMOReal layer) {
        myLayer = layer;
    }


    /** @brief Sets a new angle in navigational degrees
     * @param[in] layer The new angle to use
     */
    inline void setNaviDegree(const SUMOReal angle) {
        myNaviDegreeAngle = angle;
    }

    /** @brief Sets a new imgFile
     * @param[in] imgFile The new imgFile to use
     */
    inline void setImgFile(const std::string& imgFile) {
        myImgFile = imgFile;
    }
    /// @}

    static const std::string DEFAULT_TYPE;
    static const SUMOReal DEFAULT_LAYER;
    static const SUMOReal DEFAULT_ANGLE;
    static const std::string DEFAULT_IMG_FILE;
    static const SUMOReal DEFAULT_IMG_WIDTH;
    static const SUMOReal DEFAULT_IMG_HEIGHT;

protected:
    /// @brief The type of the Shape
    std::string myType;

    /// @brief The color of the Shape
    RGBColor myColor;

    /// @brief The layer of the Shape
    SUMOReal myLayer;

    /// @brief The angle of the Shape
    SUMOReal myNaviDegreeAngle;

    /// @brief The angle of the Shape
    std::string myImgFile;
};


#endif

/****************************************************************************/

