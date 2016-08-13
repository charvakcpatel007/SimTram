/****************************************************************************/
/// @file    NamedRTree.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    27.10.2008
/// @version $Id: NamedRTree.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A RT-tree for efficient storing of SUMO's Named objects
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef NamedRTree_h
#define NamedRTree_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <set>
#include <foreign/rtree/RTree.h>
#include <utils/common/Named.h>


// specialized implementation for speedup and avoiding warnings
#define NAMED_RTREE_QUAL RTree<Named*, Named, float, 2, Named::StoringVisitor>

template<>
inline float NAMED_RTREE_QUAL::RectSphericalVolume(Rect* a_rect) {
    ASSERT(a_rect);
    const float extent0 = a_rect->m_max[0] - a_rect->m_min[0];
    const float extent1 = a_rect->m_max[1] - a_rect->m_min[1];
    return .78539816f * (extent0 * extent0 + extent1 * extent1);
}

template<>
inline NAMED_RTREE_QUAL::Rect NAMED_RTREE_QUAL::CombineRect(Rect* a_rectA, Rect* a_rectB) {
    ASSERT(a_rectA && a_rectB);
    Rect newRect;
    newRect.m_min[0] = rtree_min(a_rectA->m_min[0], a_rectB->m_min[0]);
    newRect.m_max[0] = rtree_max(a_rectA->m_max[0], a_rectB->m_max[0]);
    newRect.m_min[1] = rtree_min(a_rectA->m_min[1], a_rectB->m_min[1]);
    newRect.m_max[1] = rtree_max(a_rectA->m_max[1], a_rectB->m_max[1]);
    return newRect;
}

// ===========================================================================
// class definitions
// ===========================================================================
/** @class NamedRTree
 * @brief A RT-tree for efficient storing of SUMO's Named objects
 *
 * This class specialises the used RT-tree implementation from "rttree.h".
 * It stores names of "Named"-objects.
 * @see Named
 */
class NamedRTree : private NAMED_RTREE_QUAL {
public:
    /// @brief Constructor
    NamedRTree() : NAMED_RTREE_QUAL(&Named::addTo) {
    }


    /// @brief Destructor
    ~NamedRTree() {
    }


    /** @brief Insert entry
     * @param a_min Min of bounding rect
     * @param a_max Max of bounding rect
     * @param a_data The instance of a Named-object to add (the ID is added)
     * @see RTree::Insert
     */
    void Insert(const float a_min[2], const float a_max[2], Named* const& a_data) {
        NAMED_RTREE_QUAL::Insert(a_min, a_max, a_data);
    }


    /** @brief Remove entry
     * @param a_min Min of bounding rect
     * @param a_max Max of bounding rect
     * @param a_data The instance of a Named-object to remove
     * @see RTree::Remove
     */
    void Remove(const float a_min[2], const float a_max[2], Named* const& a_data) {
        NAMED_RTREE_QUAL::Remove(a_min, a_max, a_data);
    }


    /** @brief Remove all enrties
     * @see RTree::RemoveAll
     */
    void RemoveAll() {
        NAMED_RTREE_QUAL::RemoveAll();
    }


    /** @brief Find all within search rectangle
     * @param a_min Min of search bounding rect
     * @param a_max Max of search bounding rect
     * @param a_searchResult Search result array.  Caller should set grow size. Function will reset, not append to array.
     * @param a_resultCallback Callback function to return result.  Callback should return 'true' to continue searching
     * @param a_context User context to pass as parameter to a_resultCallback
     * @return Returns the number of entries found
     * @see RTree::Search
     */
    int Search(const float a_min[2], const float a_max[2], const Named::StoringVisitor& c) const {
        return NAMED_RTREE_QUAL::Search(a_min, a_max, c);
    }


};


#endif

/****************************************************************************/
