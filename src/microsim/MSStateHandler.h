/****************************************************************************/
/// @file    MSStateHandler.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Thu, 13 Dec 2012
/// @version $Id: MSStateHandler.h 21182 2016-07-18 06:46:01Z behrisch $
///
// Parser and output filter for routes and vehicles state saving and loading
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
#ifndef MSStateHandler_h
#define MSStateHandler_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/SUMOTime.h>
#include <utils/xml/SUMOSAXHandler.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MESegment;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSStateHandler
 * @brief Parser and output filter for routes and vehicles state saving and loading
 */
class MSStateHandler : public SUMOSAXHandler {
public:
    /// standard constructor
    MSStateHandler(const std::string& file, const SUMOTime offset);

    /// standard destructor
    virtual ~MSStateHandler();

    /** @brief Saves the current state
     *
     * @param[in] file The file to write the state into
     */
    static void saveState(const std::string& file, SUMOTime step);

    SUMOTime getTime() const {
        return myTime;
    }

protected:
    /// @name inherited from GenericSAXHandler
    //@{

    /** @brief Called on the opening of a tag;
     *
     * @param[in] element ID of the currently opened element
     * @param[in] attrs Attributes within the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myStartElement
     */
    void myStartElement(int element,
                        const SUMOSAXAttributes& attrs);


    /** @brief Called when a closing tag occurs
     *
     * @param[in] element ID of the currently opened element
     * @exception ProcessError If something fails
     * @see GenericSAXHandler::myEndElement
     */
    void myEndElement(int element);
    //@}


private:
    const SUMOTime myOffset;
    SUMOTime myTime;
    MESegment* mySegment;
    std::pair<int, int> myEdgeAndLane;
    int myQueIndex;

    /// @brief The currently parsed vehicle type
    SUMOVTypeParameter* myCurrentVType;

private:
    /// @brief Invalidated copy constructor
    MSStateHandler(const MSStateHandler& s);

    /// @brief Invalidated assignment operator
    MSStateHandler& operator=(const MSStateHandler& s);

};


#endif

/****************************************************************************/
