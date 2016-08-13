/****************************************************************************/
/// @file    GLHelper.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
/// @version $Id: GLHelper.h 21182 2016-07-18 06:46:01Z behrisch $
///
// Some methods which help to draw certain geometrical objects in openGL
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
#ifndef GLHelper_h
#define GLHelper_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <vector>
#include <utility>
#include <utils/common/RGBColor.h>
#include <utils/geom/PositionVector.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GLHelper
 * @brief Some methods which help to draw certain geometrical objects in openGL
 *
 * This class offers some static methods for drawing primitives in openGL.
 */
class GLHelper {
public:
    /** @brief Draws a filled polygon described by the list of points
     * @note this only works well for convex polygons
     *
     * @param[in] v The polygon to draw
     * @param[in] close Whether the first point shall be appended
     */
    static void drawFilledPoly(const PositionVector& v, bool close);


    /** @brief Draws a filled polygon described by the list of points
     * @note this works for convex and concave polygons but is slower than
     * drawFilledPoly
     *
     * @param[in] v The polygon to draw
     * @param[in] close Whether the first point shall be appended
     */
    static void drawFilledPolyTesselated(const PositionVector& v, bool close);


    /** @brief Draws a thick line
     *
     * The line is drawn as a GL_QUADS.
     *
     * @param[in] beg The begin position of the line
     * @param[in] rot The direction the line shall be drawn to (in radiants)
     * @param[in] visLength The length of the line
     * @param[in] width The width of the line
     * @param[in] offset The orthogonal offset
     */
    static void drawBoxLine(const Position& beg, SUMOReal rot,
                            SUMOReal visLength, SUMOReal width, SUMOReal offset = 0);


    /** @brief Draws a thick line using the mean of both given points as begin position
     *
     * The line is drawn as a GL_QUADS.
     *
     * @param[in] beg1 One of the begin positions of the line to use the mean of
     * @param[in] beg2 One of the begin positions of the line to use the mean of
     * @param[in] rot The direction the line shall be drawn to (in radiants)
     * @param[in] visLength The length of the line
     * @param[in] width The width of the line
     */
    static void drawBoxLine(const Position& beg1, const Position& beg2,
                            SUMOReal rot, SUMOReal visLength, SUMOReal width);


    /** @brief Draws thick lines
     *
     * Each line is drawn using drawBoxLine.
     *
     * @param[in] geom The list of begin positions of the lines
     * @param[in] rots The directions the lines shall be drawn to (in radiants)
     * @param[in] lengths The lengths of the lines
     * @param[in] width The width of the lines
     * @param[in] cornerDetail Detail level for filling the corners between angled segments
     * @param[in] the orthogonal offset
     * @see drawBoxLine
     */
    static void drawBoxLines(const PositionVector& geom,
                             const std::vector<SUMOReal>& rots, const std::vector<SUMOReal>& lengths,
                             SUMOReal width, int cornerDetail = 0, SUMOReal offset = 0);

    /** @brief Draws thick lines with varying color
     *
     * Each line is drawn using drawBoxLine.
     *
     * @param[in] geom The list of begin positions of the lines
     * @param[in] rots The directions the lines shall be drawn to (in radiants)
     * @param[in] lengths The lengths of the lines
     * @param[in] cols The colors of the lines
     * @param[in] width The width of the lines
     * @param[in] cornerDetail Detail level for filling the corners between angled segments
     * @param[in] the orthogonal offset
     * @see drawBoxLine
     */
    static void drawBoxLines(const PositionVector& geom,
                             const std::vector<SUMOReal>& rots, const std::vector<SUMOReal>& lengths,
                             const std::vector<RGBColor>& cols,
                             SUMOReal width, int cornerDetail = 0, SUMOReal offset = 0);


    /** @brief Draws thick lines using the mean of the points given in the point lists as begin positions
     *
     * Each line is drawn using drawBoxLine.
     *
     * @param[in] geom1 One of the lists to obtain the lines' begin positions to use the mean of
     * @param[in] geom2 One of the lists to obtain the lines' begin positions to use the mean of
     * @param[in] rots The directions the lines shall be drawn to (in radiants)
     * @param[in] lengths The lengths of the lines
     * @param[in] width The width of the lines
     * @see drawBoxLine
     */
    static void drawBoxLines(const PositionVector& geom1,
                             const PositionVector& geom2,
                             const std::vector<SUMOReal>& rots, const std::vector<SUMOReal>& lengths,
                             SUMOReal width);


    /** @brief Draws thick lines
     *
     * Widths and length are computed internally by this method, each line is then
     *  drawn using drawBoxLine.
     *
     * @param[in] geom The list of begin positions of the lines
     * @param[in] width The width of the lines
     * @see drawBoxLine
     */
    static void drawBoxLines(const PositionVector& geom, SUMOReal width);


    /** @brief Draws a thin line
     *
     * The line is drawn as a GL_LINES.
     *
     * @param[in] beg The begin position of the line
     * @param[in] rot The direction the line shall be drawn to (in radiants)
     * @param[in] visLength The length of the line
     */
    static void drawLine(const Position& beg, SUMOReal rot,
                         SUMOReal visLength);


    /** @brief Draws a thin line using the mean of both given points as begin position
     *
     * The line is drawn as a GL_LINES.
     *
     * @param[in] beg1 One of the begin positions of the line to use the mean of
     * @param[in] beg2 One of the begin positions of the line to use the mean of
     * @param[in] rot The direction the line shall be drawn to (in radiants)
     * @param[in] visLength The length of the line
     */
    static void drawLine(const Position& beg1, const Position& beg2,
                         SUMOReal rot, SUMOReal visLength);


    /** @brief Draws a thin line along the given position vector
     *
     * The line is drawn as a GL_LINES.
     *
     * @param[in] v The positions vector to use
     */
    static void drawLine(const PositionVector& v);


    /** @brief Draws a thin line along the given position vector with variable color
     *
     * The line is drawn as a GL_LINES.
     *
     * @param[in] v The positions vector to use
     */
    static void drawLine(const PositionVector& v, const std::vector<RGBColor>& cols);


    /** @brief Draws a thin line between the two points
     *
     * The line is drawn as a GL_LINES.
     *
     * @param[in] beg Begin of the line
     * @param[in] end End of the line
     */
    static void drawLine(const Position& beg, const Position& end);


    /** @brief Draws a filled circle around (0,0)
     *
     * The circle is drawn by calling drawFilledCircle(width, steps, 0, 360).
     *
     * @param[in] width The width of the circle
     * @param[in] steps The number of steps to divide the circle into
     */
    static void drawFilledCircle(SUMOReal width, int steps = 8);


    /** @brief Draws a filled circle around (0,0)
     *
     * The circle is drawn use GL_TRIANGLES.
     *
     * @param[in] width The width of the circle
     * @param[in] steps The number of steps to divide the circle into
     * @param[in] beg The begin angle in degress
     * @param[in] end The end angle in degress
     */
    static void drawFilledCircle(SUMOReal width, int steps,
                                 SUMOReal beg, SUMOReal end);


    /** @brief Draws an unfilled circle around (0,0)
     *
     * The circle is drawn by calling drawOutlineCircle(width, iwidth, steps, 0, 360).
     *
     * @param[in] width The (outer) width of the circle
     * @param[in] iwidth The inner width of the circle
     * @param[in] steps The number of steps to divide the circle into
     */
    static void drawOutlineCircle(SUMOReal width, SUMOReal iwidth,
                                  int steps = 8);


    /** @brief Draws an unfilled circle around (0,0)
     *
     * The circle is drawn use GL_TRIANGLES.
     *
     * @param[in] width The (outer) width of the circle
     * @param[in] iwidth The inner width of the circle
     * @param[in] steps The number of steps to divide the circle into
     * @param[in] beg The begin angle in degress
     * @param[in] end The end angle in degress
     */
    static void drawOutlineCircle(SUMOReal width, SUMOReal iwidth,
                                  int steps, SUMOReal beg, SUMOReal end);


    /** @brief Draws a triangle at the end of the given line
     *
     * @param[in] p1 The start of the line at which end the triangle shall be drawn
     * @param[in] p2 The end of the line at which end the triangle shall be drawn
     * @param[in] tLength The length of the triangle
     * @param[in] tWidth The width of the triangle
     */
    static void drawTriangleAtEnd(const Position& p1, const Position& p2,
                                  SUMOReal tLength, SUMOReal tWidth);

    /// @brief Sets the gl-color to this value
    static void setColor(const RGBColor& c);

    /// @brief gets the gl-color
    static RGBColor getColor();

    /// @brief draw Text with given parameters
    static void drawText(const std::string& text, const Position& pos,
                         const SUMOReal layer, const SUMOReal size,
                         const RGBColor& col = RGBColor::BLACK, const SUMOReal angle = 0);

    /// @brief draw Text box with given parameters
    static void drawTextBox(const std::string& text, const Position& pos,
                            const SUMOReal layer, const SUMOReal size,
                            const RGBColor& txtColor = RGBColor::BLACK,
                            const RGBColor& bgColor = RGBColor::WHITE,
                            const RGBColor& borderColor = RGBColor::BLACK,
                            const SUMOReal angle = 0);

    /// @brief draw vertex numbers for the given shape (in a random color)
    static void debugVertices(const PositionVector& shape, SUMOReal size, SUMOReal layer = 256);

private:
    /// @brief normalize angle for lookup in myCircleCoords
    static int angleLookup(SUMOReal angleDeg);

    /// @brief whether the road makes a right turn (or goes straight)
    static bool rightTurn(SUMOReal angle1, SUMOReal angle2);

private:
    /// @brief Storage for precomputed sin/cos-values describing a circle
    static std::vector<std::pair<SUMOReal, SUMOReal> > myCircleCoords;

};


#endif

/****************************************************************************/

