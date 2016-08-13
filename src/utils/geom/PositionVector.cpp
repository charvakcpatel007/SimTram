/****************************************************************************/
/// @file    PositionVector.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @date    Sept 2002
/// @version $Id: PositionVector.cpp 21160 2016-07-14 08:03:04Z behrisch $
///
// A list of positions
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


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <queue>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <iterator>
#include <limits>
#include <utils/common/StdDefs.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include "AbstractPoly.h"
#include "Position.h"
#include "PositionVector.h"
#include "GeomHelper.h"
#include "Helper_ConvexHull.h"
#include "Boundary.h"

#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS

// ===========================================================================
// method definitions
// ===========================================================================

PositionVector::PositionVector() {}


PositionVector::PositionVector(const std::vector<Position>& v) {
    std::copy(v.begin(), v.end(), std::back_inserter(*this));
}


PositionVector::PositionVector(const std::vector<Position>::const_iterator beg, const std::vector<Position>::const_iterator end) {
    std::copy(beg, end, std::back_inserter(*this));
}


PositionVector::PositionVector(const Position& p1, const Position& p2) {
    push_back(p1);
    push_back(p2);
}


PositionVector::~PositionVector() {}


bool
PositionVector::around(const Position& p, SUMOReal offset) const {
    if (offset != 0) {
        PositionVector tmp(*this);
        tmp.scaleAbsolute(offset);
        return tmp.around(p);
    }
    SUMOReal angle = 0;
    for (const_iterator i = begin(); i != end() - 1; i++) {
        Position p1(
            (*i).x() - p.x(),
            (*i).y() - p.y());
        Position p2(
            (*(i + 1)).x() - p.x(),
            (*(i + 1)).y() - p.y());
        angle += GeomHelper::angle2D(p1, p2);
    }
    Position p1(
        (*(end() - 1)).x() - p.x(),
        (*(end() - 1)).y() - p.y());
    Position p2(
        (*(begin())).x() - p.x(),
        (*(begin())).y() - p.y());
    angle += GeomHelper::angle2D(p1, p2);
    return (!(fabs(angle) < M_PI));
}


bool
PositionVector::overlapsWith(const AbstractPoly& poly, SUMOReal offset) const {
    for (const_iterator i = begin(); i != end() - 1; i++) {
        if (poly.around(*i, offset)) {
            return true;
        }
    }
    return false;
}


bool
PositionVector::intersects(const Position& p1, const Position& p2) const {
    if (size() < 2) {
        return false;
    }
    for (const_iterator i = begin(); i != end() - 1; i++) {
        if (intersects(*i, *(i + 1), p1, p2)) {
            return true;
        }
    }
    return false;
}


bool
PositionVector::intersects(const PositionVector& v1) const {
    if (size() < 2) {
        return false;
    }
    for (const_iterator i = begin(); i != end() - 1; i++) {
        if (v1.intersects(*i, *(i + 1))) {
            return true;
        }
    }
    return false;
}


Position
PositionVector::intersectionPosition2D(const Position& p1, const Position& p2, const SUMOReal withinDist) const {
    for (const_iterator i = begin(); i != end() - 1; i++) {
        SUMOReal x, y, m;
        if (intersects(*i, *(i + 1), p1, p2, withinDist, &x, &y, &m)) {
            return Position(x, y);
        }
    }
    return Position::INVALID;
}


Position
PositionVector::intersectionPosition2D(const PositionVector& v1) const {
    for (const_iterator i = begin(); i != end() - 1; i++) {
        if (v1.intersects(*i, *(i + 1))) {
            return v1.intersectionPosition2D(*i, *(i + 1));
        }
    }
    return Position::INVALID;
}


const Position&
PositionVector::operator[](int index) const {
    if (index >= 0) {
        return at(index);
    } else {
        return at((int)size() + index);
    }
}


Position&
PositionVector::operator[](int index) {
    if (index >= 0) {
        return at(index);
    } else {
        return at((int)size() + index);
    }
}


Position
PositionVector::positionAtOffset(SUMOReal pos, SUMOReal lateralOffset) const {
    const_iterator i = begin();
    SUMOReal seenLength = 0;
    do {
        const SUMOReal nextLength = (*i).distanceTo(*(i + 1));
        if (seenLength + nextLength > pos) {
            return positionAtOffset(*i, *(i + 1), pos - seenLength, lateralOffset);
        }
        seenLength += nextLength;
    } while (++i != end() - 1);
    return back();
}


Position
PositionVector::positionAtOffset2D(SUMOReal pos, SUMOReal lateralOffset) const {
    const_iterator i = begin();
    SUMOReal seenLength = 0;
    do {
        const SUMOReal nextLength = (*i).distanceTo2D(*(i + 1));
        if (seenLength + nextLength > pos) {
            return positionAtOffset2D(*i, *(i + 1), pos - seenLength, lateralOffset);
        }
        seenLength += nextLength;
    } while (++i != end() - 1);
    return back();
}


SUMOReal
PositionVector::rotationAtOffset(SUMOReal pos) const {
    if (pos < 0) {
        pos += length();
    }
    const_iterator i = begin();
    SUMOReal seenLength = 0;
    do {
        const Position& p1 = *i;
        const Position& p2 = *(i + 1);
        const SUMOReal nextLength = p1.distanceTo(p2);
        if (seenLength + nextLength > pos) {
            return p1.angleTo2D(p2);
        }
        seenLength += nextLength;
    } while (++i != end() - 1);
    const Position& p1 = (*this)[-2];
    const Position& p2 = back();
    return p1.angleTo2D(p2);
}


SUMOReal
PositionVector::rotationDegreeAtOffset(SUMOReal pos) const {
    return GeomHelper::legacyDegree(rotationAtOffset(pos));
}


SUMOReal
PositionVector::slopeDegreeAtOffset(SUMOReal pos) const {
    const_iterator i = begin();
    SUMOReal seenLength = 0;
    do {
        const Position& p1 = *i;
        const Position& p2 = *(i + 1);
        const SUMOReal nextLength = p1.distanceTo(p2);
        if (seenLength + nextLength > pos) {
            return RAD2DEG(atan2(p2.z() - p1.z(), p1.distanceTo2D(p2)));
        }
        seenLength += nextLength;
    } while (++i != end() - 1);
    const Position& p1 = (*this)[-2];
    const Position& p2 = back();
    return RAD2DEG(atan2(p2.z() - p1.z(), p1.distanceTo2D(p2)));
}


Position
PositionVector::positionAtOffset(const Position& p1, const Position& p2, SUMOReal pos, SUMOReal lateralOffset) {
    const SUMOReal dist = p1.distanceTo(p2);
    if (pos < 0 || dist < pos) {
        return Position::INVALID;
    }
    if (lateralOffset != 0) {
        const Position offset = sideOffset(p1, p2, -lateralOffset); // move in the same direction as Position::move2side
        if (pos == 0.) {
            return p1 + offset;
        }
        return p1 + (p2 - p1) * (pos / dist) + offset;
    }
    if (pos == 0.) {
        return p1;
    }
    return p1 + (p2 - p1) * (pos / dist);
}


Position
PositionVector::positionAtOffset2D(const Position& p1, const Position& p2, SUMOReal pos, SUMOReal lateralOffset) {
    const SUMOReal dist = p1.distanceTo2D(p2);
    if (pos < 0 || dist < pos) {
        return Position::INVALID;
    }
    if (lateralOffset != 0) {
        const Position offset = sideOffset(p1, p2, -lateralOffset); // move in the same direction as Position::move2side
        if (pos == 0.) {
            return p1 + offset;
        }
        return p1 + (p2 - p1) * (pos / dist) + offset;
    }
    if (pos == 0.) {
        return p1;
    }
    return p1 + (p2 - p1) * (pos / dist);
}


Boundary
PositionVector::getBoxBoundary() const {
    Boundary ret;
    for (const_iterator i = begin(); i != end(); i++) {
        ret.add(*i);
    }
    return ret;
}


Position
PositionVector::getPolygonCenter() const {
    SUMOReal x = 0;
    SUMOReal y = 0;
    SUMOReal z = 0;
    for (const_iterator i = begin(); i != end(); i++) {
        x += (*i).x();
        y += (*i).y();
        z += (*i).z();
    }
    return Position(x / (SUMOReal) size(), y / (SUMOReal) size(), z / (SUMOReal)size());
}


Position
PositionVector::getCentroid() const {
    PositionVector tmp = *this;
    if (!isClosed()) { // make sure its closed
        tmp.push_back(tmp[0]);
    }
    const int endIndex = (int)tmp.size() - 1;
    SUMOReal div = 0; // 6 * area including sign
    SUMOReal x = 0;
    SUMOReal y = 0;
    if (tmp.area() != 0) { // numerical instability ?
        // http://en.wikipedia.org/wiki/Polygon
        for (int i = 0; i < endIndex; i++) {
            const SUMOReal z = tmp[i].x() * tmp[i + 1].y() - tmp[i + 1].x() * tmp[i].y();
            div += z; // area formula
            x += (tmp[i].x() + tmp[i + 1].x()) * z;
            y += (tmp[i].y() + tmp[i + 1].y()) * z;
        }
        div *= 3; //  6 / 2, the 2 comes from the area formula
        return Position(x / div, y / div);
    } else {
        // compute by decomposing into line segments
        // http://en.wikipedia.org/wiki/Centroid#By_geometric_decomposition
        SUMOReal lengthSum = 0;
        for (int i = 0; i < endIndex; i++) {
            SUMOReal length = tmp[i].distanceTo(tmp[i + 1]);
            x += (tmp[i].x() + tmp[i + 1].x()) * length / 2;
            y += (tmp[i].y() + tmp[i + 1].y()) * length / 2;
            lengthSum += length;
        }
        if (lengthSum == 0) {
            // it is probably only one point
            return tmp[0];
        }
        return Position(x / lengthSum, y / lengthSum);
    }
}


void
PositionVector::scaleRelative(SUMOReal factor) {
    Position centroid = getCentroid();
    for (int i = 0; i < static_cast<int>(size()); i++) {
        (*this)[i] = centroid + (((*this)[i] - centroid) * factor);
    }
}


void
PositionVector::scaleAbsolute(SUMOReal offset) {
    Position centroid = getCentroid();
    for (int i = 0; i < static_cast<int>(size()); i++) {
        (*this)[i] = centroid + (((*this)[i] - centroid) + offset);
    }
}


Position
PositionVector::getLineCenter() const {
    if (size() == 1) {
        return (*this)[0];
    }
    return positionAtOffset(SUMOReal((length() / 2.)));
}


SUMOReal
PositionVector::length() const {
    SUMOReal len = 0;
    for (const_iterator i = begin(); i != end() - 1; i++) {
        len += (*i).distanceTo(*(i + 1));
    }
    return len;
}


SUMOReal
PositionVector::length2D() const {
    SUMOReal len = 0;
    for (const_iterator i = begin(); i != end() - 1; i++) {
        len += (*i).distanceTo2D(*(i + 1));
    }
    return len;
}


SUMOReal
PositionVector::area() const {
    if (size() < 3) {
        return 0;
    }
    SUMOReal area = 0;
    PositionVector tmp = *this;
    if (!isClosed()) { // make sure its closed
        tmp.push_back(tmp[0]);
    }
    const int endIndex = (int)tmp.size() - 1;
    // http://en.wikipedia.org/wiki/Polygon
    for (int i = 0; i < endIndex; i++) {
        area += tmp[i].x() * tmp[i + 1].y() - tmp[i + 1].x() * tmp[i].y();
    }
    if (area < 0) { // we whether we had cw or ccw order
        area *= -1;
    }
    return area / 2;
}


bool
PositionVector::partialWithin(const AbstractPoly& poly, SUMOReal offset) const {
    for (const_iterator i = begin(); i != end() - 1; i++) {
        if (poly.around(*i, offset)) {
            return true;
        }
    }
    return false;
}


bool
PositionVector::crosses(const Position& p1, const Position& p2) const {
    return intersects(p1, p2);
}


std::pair<PositionVector, PositionVector>
PositionVector::splitAt(SUMOReal where) const {
    if (size() < 2) {
        throw InvalidArgument("Vector to short for splitting");
    }
    if (where <= POSITION_EPS || where >= length() - POSITION_EPS) {
        WRITE_WARNING("Splitting vector close to end (pos: " + toString(where) + ", length: " + toString(length()) + ")");
    }
    PositionVector first, second;
    first.push_back((*this)[0]);
    SUMOReal seen = 0;
    const_iterator it = begin() + 1;
    SUMOReal next = first.back().distanceTo(*it);
    // see how many points we can add to first
    while (where >= seen + next + POSITION_EPS) {
        seen += next;
        first.push_back(*it);
        it++;
        next = first.back().distanceTo(*it);
    }
    if (fabs(where - (seen + next)) > POSITION_EPS || it == end() - 1) {
        // we need to insert a new point because 'where' is not close to an
        // existing point or it is to close to the endpoint
        const Position p = positionAtOffset(first.back(), *it, where - seen);
        first.push_back(p);
        second.push_back(p);
    } else {
        first.push_back(*it);
    }
    // add the remaining points to second
    for (; it != end(); it++) {
        second.push_back(*it);
    }
    assert(first.size() >= 2);
    assert(second.size() >= 2);
    assert(first.back() == second.front());
    assert(fabs(first.length() + second.length() - length()) < 2 * POSITION_EPS);
    return std::pair<PositionVector, PositionVector>(first, second);
}


std::ostream&
operator<<(std::ostream& os, const PositionVector& geom) {
    for (PositionVector::const_iterator i = geom.begin(); i != geom.end(); i++) {
        if (i != geom.begin()) {
            os << " ";
        }
        os << (*i);
    }
    return os;
}


void
PositionVector::sortAsPolyCWByAngle() {
    std::sort(begin(), end(), as_poly_cw_sorter());
}


void
PositionVector::add(SUMOReal xoff, SUMOReal yoff, SUMOReal zoff) {
    for (int i = 0; i < static_cast<int>(size()); i++) {
        (*this)[i].add(xoff, yoff, zoff);
    }
}


void
PositionVector::add(const Position& offset) {
    add(offset.x(), offset.y(), offset.z());
}


void
PositionVector::mirrorX() {
    for (int i = 0; i < static_cast<int>(size()); i++) {
        (*this)[i].mul(1, -1);
    }
}


PositionVector::as_poly_cw_sorter::as_poly_cw_sorter() {}


int
PositionVector::as_poly_cw_sorter::operator()(const Position& p1, const Position& p2) const {
    return atan2(p1.x(), p1.y()) < atan2(p2.x(), p2.y());
}


void
PositionVector::sortByIncreasingXY() {
    std::sort(begin(), end(), increasing_x_y_sorter());
}


PositionVector::increasing_x_y_sorter::increasing_x_y_sorter() {}


int
PositionVector::increasing_x_y_sorter::operator()(const Position& p1,
        const Position& p2) const {
    if (p1.x() != p2.x()) {
        return p1.x() < p2.x();
    }
    return p1.y() < p2.y();
}


SUMOReal
PositionVector::isLeft(const Position& P0, const Position& P1,  const Position& P2) const {
    return (P1.x() - P0.x()) * (P2.y() - P0.y()) - (P2.x() - P0.x()) * (P1.y() - P0.y());
}


PositionVector
PositionVector::convexHull() const {
    PositionVector ret = *this;
    ret.sortAsPolyCWByAngle();
    return simpleHull_2D(ret);
}


void
PositionVector::append(const PositionVector& v, SUMOReal sameThreshold) {
    if (size() > 0 && v.size() > 0 && back().distanceTo(v[0]) < sameThreshold) {
        copy(v.begin() + 1, v.end(), back_inserter(*this));
    } else {
        copy(v.begin(), v.end(), back_inserter(*this));
    }
}


PositionVector
PositionVector::getSubpart(SUMOReal beginOffset, SUMOReal endOffset) const {
    PositionVector ret;
    Position begPos = front();
    if (beginOffset > POSITION_EPS) {
        begPos = positionAtOffset(beginOffset);
    }
    Position endPos = back();
    if (endOffset < length() - POSITION_EPS) {
        endPos = positionAtOffset(endOffset);
    }
    ret.push_back(begPos);

    SUMOReal seen = 0;
    const_iterator i = begin();
    // skip previous segments
    while ((i + 1) != end()
            &&
            seen + (*i).distanceTo(*(i + 1)) < beginOffset) {
        seen += (*i).distanceTo(*(i + 1));
        i++;
    }
    // append segments in between
    while ((i + 1) != end()
            &&
            seen + (*i).distanceTo(*(i + 1)) < endOffset) {

        ret.push_back_noDoublePos(*(i + 1));
        seen += (*i).distanceTo(*(i + 1));
        i++;
    }
    // append end
    ret.push_back_noDoublePos(endPos);
    return ret;
}


PositionVector
PositionVector::getSubpart2D(SUMOReal beginOffset, SUMOReal endOffset) const {
    PositionVector ret;
    Position begPos = front();
    if (beginOffset > POSITION_EPS) {
        begPos = positionAtOffset2D(beginOffset);
    }
    Position endPos = back();
    if (endOffset < length2D() - POSITION_EPS) {
        endPos = positionAtOffset2D(endOffset);
    }
    ret.push_back(begPos);

    SUMOReal seen = 0;
    const_iterator i = begin();
    // skip previous segments
    while ((i + 1) != end()
            &&
            seen + (*i).distanceTo2D(*(i + 1)) < beginOffset) {
        seen += (*i).distanceTo2D(*(i + 1));
        i++;
    }
    // append segments in between
    while ((i + 1) != end()
            &&
            seen + (*i).distanceTo2D(*(i + 1)) < endOffset) {

        ret.push_back_noDoublePos(*(i + 1));
        seen += (*i).distanceTo2D(*(i + 1));
        i++;
    }
    // append end
    ret.push_back_noDoublePos(endPos);
    return ret;
}


PositionVector
PositionVector::getSubpartByIndex(int beginIndex, int count) const {
    if (beginIndex < 0) {
        beginIndex += (int)size();
    }
    assert(count >= 0);
    assert(beginIndex < (int)size());
    assert(beginIndex + count <= (int)size());
    PositionVector result;
    for (int i = beginIndex; i < beginIndex + count; ++i) {
        result.push_back((*this)[i]);
    }
    return result;
}


SUMOReal
PositionVector::beginEndAngle() const {
    return front().angleTo2D(back());
}


SUMOReal
PositionVector::nearest_offset_to_point2D(const Position& p, bool perpendicular) const {
    SUMOReal minDist = std::numeric_limits<SUMOReal>::max();
    SUMOReal nearestPos = GeomHelper::INVALID_OFFSET;
    SUMOReal seen = 0;
    for (const_iterator i = begin(); i != end() - 1; i++) {
        const SUMOReal pos =
            GeomHelper::nearest_offset_on_line_to_point2D(*i, *(i + 1), p, perpendicular);
        const SUMOReal dist = pos == GeomHelper::INVALID_OFFSET ? minDist : p.distanceTo2D(positionAtOffset2D(*i, *(i + 1), pos));
        if (dist < minDist) {
            nearestPos = pos + seen;
            minDist = dist;
        }
        if (perpendicular && i != begin() && pos == GeomHelper::INVALID_OFFSET) {
            // even if perpendicular is set we still need to check the distance to the inner points
            const SUMOReal cornerDist = p.distanceTo2D(*i);
            if (cornerDist < minDist) {
                const SUMOReal pos1 =
                    GeomHelper::nearest_offset_on_line_to_point2D(*(i - 1), *i, p, false);
                const SUMOReal pos2 =
                    GeomHelper::nearest_offset_on_line_to_point2D(*i, *(i + 1), p, false);
                if (pos1 == (*(i - 1)).distanceTo2D(*i) && pos2 == 0.) {
                    nearestPos = seen;
                    minDist = cornerDist;
                }
            }
        }
        seen += (*i).distanceTo2D(*(i + 1));
    }
    return nearestPos;
}


Position
PositionVector::transformToVectorCoordinates(const Position& p, bool extend) const {
    // @toDo this duplicates most of the code in nearest_offset_to_point2D. It should be refactored
    if (extend) {
        PositionVector extended = *this;
        const SUMOReal dist = 2 * distance2D(p);
        extended.extrapolate(dist);
        return extended.transformToVectorCoordinates(p) - Position(dist, 0);
    }
    SUMOReal minDist = std::numeric_limits<SUMOReal>::max();
    SUMOReal nearestPos = -1;
    SUMOReal seen = 0;
    int sign = 1;
    for (const_iterator i = begin(); i != end() - 1; i++) {
        const SUMOReal pos =
            GeomHelper::nearest_offset_on_line_to_point2D(*i, *(i + 1), p, true);
        const SUMOReal dist = pos < 0 ? minDist : p.distanceTo2D(positionAtOffset(*i, *(i + 1), pos));
        if (dist < minDist) {
            nearestPos = pos + seen;
            minDist = dist;
            sign = isLeft(*i, *(i + 1), p) >= 0 ? -1 : 1;
        }
        if (i != begin() && pos == GeomHelper::INVALID_OFFSET) {
            // even if perpendicular is set we still need to check the distance to the inner points
            const SUMOReal cornerDist = p.distanceTo2D(*i);
            if (cornerDist < minDist) {
                const SUMOReal pos1 =
                    GeomHelper::nearest_offset_on_line_to_point2D(*(i - 1), *i, p, false);
                const SUMOReal pos2 =
                    GeomHelper::nearest_offset_on_line_to_point2D(*i, *(i + 1), p, false);
                if (pos1 == (*(i - 1)).distanceTo2D(*i) && pos2 == 0.) {
                    nearestPos = seen;
                    minDist = cornerDist;
                    sign = isLeft(*(i - 1), *i, p) >= 0 ? -1 : 1;
                }
            }
        }
        seen += (*i).distanceTo2D(*(i + 1));
    }
    if (nearestPos != -1) {
        return Position(nearestPos, sign * minDist);
    } else {
        return Position::INVALID;
    }
}


int
PositionVector::indexOfClosest(const Position& p) const {
    assert(size() > 0);
    SUMOReal minDist = std::numeric_limits<SUMOReal>::max();
    SUMOReal dist;
    int closest = 0;
    for (int i = 0; i < (int)size(); i++) {
        dist = p.distanceTo((*this)[i]);
        if (dist < minDist) {
            closest = i;
            minDist = dist;
        }
    }
    return closest;
}


int
PositionVector::insertAtClosest(const Position& p) {
    SUMOReal minDist = std::numeric_limits<SUMOReal>::max();
    int insertionIndex = 1;
    for (int i = 0; i < (int)size() - 1; i++) {
        const SUMOReal length = GeomHelper::nearest_offset_on_line_to_point2D((*this)[i], (*this)[i + 1], p, false);
        const Position& outIntersection = PositionVector::positionAtOffset2D((*this)[i], (*this)[i + 1], length);
        const SUMOReal dist = p.distanceTo2D(outIntersection);
        if (dist < minDist) {
            insertionIndex = i + 1;
            minDist = dist;
        }
    }
    insert(begin() + insertionIndex, p);
    return insertionIndex;
}


int
PositionVector::removeClosest(const Position& p) {
    if (size() == 0) {
        return -1;
    }
    SUMOReal minDist = std::numeric_limits<SUMOReal>::max();
    int removalIndex = 0;
    for (int i = 0; i < (int)size(); i++) {
        const SUMOReal dist = p.distanceTo2D((*this)[i]);
        if (dist < minDist) {
            removalIndex = i;
            minDist = dist;
        }
    }
    erase(begin() + removalIndex);
    return removalIndex;
}


std::vector<SUMOReal>
PositionVector::intersectsAtLengths2D(const PositionVector& other) const {
    std::vector<SUMOReal> ret;
    for (const_iterator i = other.begin(); i != other.end() - 1; i++) {
        std::vector<SUMOReal> atSegment = intersectsAtLengths2D(*i, *(i + 1));
        copy(atSegment.begin(), atSegment.end(), back_inserter(ret));
    }
    return ret;
}


std::vector<SUMOReal>
PositionVector::intersectsAtLengths2D(const Position& lp1, const Position& lp2) const {
    std::vector<SUMOReal> ret;
    SUMOReal pos = 0;
    for (const_iterator i = begin(); i != end() - 1; i++) {
        const Position& p1 = *i;
        const Position& p2 = *(i + 1);
        SUMOReal x, y, m;
        if (intersects(p1, p2, lp1, lp2, 0., &x, &y, &m)) {
            ret.push_back(Position(x, y).distanceTo2D(p1) + pos);
        }
        pos += p1.distanceTo2D(p2);
    }
    return ret;
}


void
PositionVector::extrapolate(const SUMOReal val, const bool onlyFirst) {
    assert(size() > 1);
    Position& p1 = (*this)[0];
    Position& p2 = (*this)[1];
    const Position offset = (p2 - p1) * (val / p1.distanceTo(p2));
    p1.sub(offset);
    if (!onlyFirst) {
        if (size() == 2) {
            p2.add(offset);
        } else {
            const Position& e1 = (*this)[-2];
            Position& e2 = (*this)[-1];
            e2.sub((e1 - e2) * (val / e1.distanceTo(e2)));
        }
    }
}


void
PositionVector::extrapolate2D(const SUMOReal val, const bool onlyFirst) {
    assert(size() > 1);
    Position& p1 = (*this)[0];
    Position& p2 = (*this)[1];
    const Position offset = (p2 - p1) * (val / p1.distanceTo2D(p2));
    p1.sub(offset);
    if (!onlyFirst) {
        if (size() == 2) {
            p2.add(offset);
        } else {
            const Position& e1 = (*this)[-2];
            Position& e2 = (*this)[-1];
            e2.sub((e1 - e2) * (val / e1.distanceTo2D(e2)));
        }
    }
}


PositionVector
PositionVector::reverse() const {
    PositionVector ret;
    for (const_reverse_iterator i = rbegin(); i != rend(); i++) {
        ret.push_back(*i);
    }
    return ret;
}


Position
PositionVector::sideOffset(const Position& beg, const Position& end, const SUMOReal amount) {
    const SUMOReal scale = amount / beg.distanceTo2D(end);
    return Position((beg.y() - end.y()) * scale, (end.x() - beg.x()) * scale);
}


void
PositionVector::move2side(SUMOReal amount) {
    if (size() < 2) {
        return;
    }
    PositionVector shape;
    for (int i = 0; i < static_cast<int>(size()); i++) {
        if (i == 0) {
            const Position& from = (*this)[i];
            const Position& to = (*this)[i + 1];
            shape.push_back(from - sideOffset(from, to, amount));
        } else if (i == static_cast<int>(size()) - 1) {
            const Position& from = (*this)[i - 1];
            const Position& to = (*this)[i];
            shape.push_back(to - sideOffset(from, to, amount));
        } else {
            const Position& from = (*this)[i - 1];
            const Position& me = (*this)[i];
            const Position& to = (*this)[i + 1];
            PositionVector fromMe(from, me);
            fromMe.extrapolate2D(me.distanceTo2D(to));
            const SUMOReal extrapolateDev = fromMe[1].distanceTo2D(to);
            if (fabs(extrapolateDev) < POSITION_EPS) {
                // parallel case, just shift the middle point
                shape.push_back(me - sideOffset(from, to, amount));
            } else if (fabs(extrapolateDev - 2 * me.distanceTo2D(to)) < POSITION_EPS) {
                // counterparallel case, just shift the middle point
                PositionVector fromMe(from, me);
                fromMe.extrapolate2D(amount);
                shape.push_back(fromMe[1]);
            } else {
                Position offsets = sideOffset(from, me, amount);
                Position offsets2 = sideOffset(me, to, amount);
                PositionVector l1(from - offsets, me - offsets);
                PositionVector l2(me - offsets2, to - offsets2);
                shape.push_back(l1.intersectionPosition2D(l2[0], l2[1], 100));
                if (shape.back() == Position::INVALID) {
                    throw InvalidArgument("no line intersection");
                }
            }
            // copy original z value
            shape.back().set(shape.back().x(), shape.back().y(), me.z());
        }
    }
    *this = shape;
}


SUMOReal
PositionVector::angleAt2D(int pos) const {
    assert((int)size() > pos + 1);
    return (*this)[pos].angleTo2D((*this)[pos + 1]);
}


void
PositionVector::closePolygon() {
    if (size() == 0 || (*this)[0] == back()) {
        return;
    }
    push_back((*this)[0]);
}


std::vector<SUMOReal>
PositionVector::distances(const PositionVector& s, bool perpendicular) const {
    std::vector<SUMOReal> ret;
    const_iterator i;
    for (i = begin(); i != end(); i++) {
        const SUMOReal dist = s.distance2D(*i, perpendicular);
        if (dist != GeomHelper::INVALID_OFFSET) {
            ret.push_back(dist);
        }
    }
    for (i = s.begin(); i != s.end(); i++) {
        const SUMOReal dist = distance2D(*i, perpendicular);
        if (dist != GeomHelper::INVALID_OFFSET) {
            ret.push_back(dist);
        }
    }
    return ret;
}


SUMOReal
PositionVector::distance2D(const Position& p, bool perpendicular) const {
    if (size() == 0) {
        return std::numeric_limits<double>::max();
    } else if (size() == 1) {
        return front().distanceTo(p);
    }
    const SUMOReal nearestOffset = nearest_offset_to_point2D(p, perpendicular);
    if (nearestOffset == GeomHelper::INVALID_OFFSET) {
        return GeomHelper::INVALID_OFFSET;
    } else {
        return p.distanceTo2D(positionAtOffset2D(nearestOffset));
    }
}


void
PositionVector::push_back_noDoublePos(const Position& p) {
    if (size() == 0 || !p.almostSame(back())) {
        push_back(p);
    }
}


void
PositionVector::push_front_noDoublePos(const Position& p) {
    if (size() == 0 || !p.almostSame(front())) {
        insert(begin(), p);
    }
}


bool
PositionVector::isClosed() const {
    return size() >= 2 && (*this)[0] == back();
}


void
PositionVector::removeDoublePoints(SUMOReal minDist, bool assertLength) {
    if (size() > 1) {
        iterator last = begin();
        for (iterator i = begin() + 1; i != end() && (!assertLength || size() > 2);) {
            if (last->almostSame(*i, minDist)) {
                i = erase(i);
            } else {
                last = i;
                ++i;
            }
        }
    }
}


bool
PositionVector::operator==(const PositionVector& v2) const {
    if (size() == v2.size()) {
        for (int i = 0; i < (int)size(); i++) {
            if ((*this)[i] != v2[i]) {
                return false;
            }
        }
        return true;
    } else {
        return false;
    }
}


bool
PositionVector::hasElevation() const {
    if (size() > 2) {
        return false;
    }
    for (const_iterator i = begin(); i != end() - 1; i++) {
        if ((*i).z() != (*(i + 1)).z()) {
            return true;
        }
    }
    return false;
}


bool
PositionVector::intersects(const Position& p11, const Position& p12, const Position& p21, const Position& p22, const SUMOReal withinDist, SUMOReal* x, SUMOReal* y, SUMOReal* mu) {
    const SUMOReal eps = std::numeric_limits<SUMOReal>::epsilon();
    const double denominator = (p22.y() - p21.y()) * (p12.x() - p11.x()) - (p22.x() - p21.x()) * (p12.y() - p11.y());
    const double numera = (p22.x() - p21.x()) * (p11.y() - p21.y()) - (p22.y() - p21.y()) * (p11.x() - p21.x());
    const double numerb = (p12.x() - p11.x()) * (p11.y() - p21.y()) - (p12.y() - p11.y()) * (p11.x() - p21.x());
    /* Are the lines coincident? */
    if (fabs(numera) < eps && fabs(numerb) < eps && fabs(denominator) < eps) {
        SUMOReal a1;
        SUMOReal a2;
        SUMOReal a3;
        SUMOReal a4;
        SUMOReal a = -1e12;
        if (p11.x() != p12.x()) {
            a1 = p11.x() < p12.x() ? p11.x() : p12.x();
            a2 = p11.x() < p12.x() ? p12.x() : p11.x();
            a3 = p21.x() < p22.x() ? p21.x() : p22.x();
            a4 = p21.x() < p22.x() ? p22.x() : p21.x();
        } else {
            a1 = p11.y() < p12.y() ? p11.y() : p12.y();
            a2 = p11.y() < p12.y() ? p12.y() : p11.y();
            a3 = p21.y() < p22.y() ? p21.y() : p22.y();
            a4 = p21.y() < p22.y() ? p22.y() : p21.y();
        }
        if (a1 <= a3 && a3 <= a2) {
            if (a4 < a2) {
                a = (a3 + a4) / 2;
            } else {
                a = (a2 + a3) / 2;
            }
        }
        if (a3 <= a1 && a1 <= a4) {
            if (a2 < a4) {
                a = (a1 + a2) / 2;
            } else {
                a = (a1 + a4) / 2;
            }
        }
        if (a != -1e12) {
            if (x != 0) {
                if (p11.x() != p12.x()) {
                    *mu = (a - p11.x()) / (p12.x() - p11.x());
                    *x = a;
                    *y = p11.y() + (*mu) * (p12.y() - p11.y());
                } else {
                    *x = p11.x();
                    *y = a;
                    if (p12.y() == p11.y()) {
                        *mu = 0;
                    } else {
                        *mu = (a - p11.y()) / (p12.y() - p11.y());
                    }
                }
            }
            return true;
        }
        return false;
    }
    /* Are the lines parallel */
    if (fabs(denominator) < eps) {
        return false;
    }
    /* Is the intersection along the segments */
    double mua = numera / denominator;
    /* reduce rounding errors for lines ending in the same point */
    if (fabs(p12.x() - p22.x()) < eps && fabs(p12.y() - p22.y()) < eps) {
        mua = 1.;
    } else {
        const double offseta = withinDist / p11.distanceTo2D(p12);
        const double offsetb = withinDist / p21.distanceTo2D(p22);
        const double mub = numerb / denominator;
        if (mua < -offseta || mua > 1 + offseta || mub < -offsetb || mub > 1 + offsetb) {
            return false;
        }
    }
    if (x != 0) {
        *x = p11.x() + mua * (p12.x() - p11.x());
        *y = p11.y() + mua * (p12.y() - p11.y());
        *mu = mua;
    }
    return true;
}


void
PositionVector::rotate2D(SUMOReal angle) {
    const SUMOReal s = sin(angle);
    const SUMOReal c = cos(angle);
    for (int i = 0; i < (int)size(); i++) {
        const SUMOReal x = (*this)[i].x();
        const SUMOReal y = (*this)[i].y();
        const SUMOReal z = (*this)[i].z();
        const SUMOReal xnew = x * c - y * s;
        const SUMOReal ynew = x * s + y * c;
        (*this)[i].set(xnew, ynew, z);
    }
}

/****************************************************************************/

