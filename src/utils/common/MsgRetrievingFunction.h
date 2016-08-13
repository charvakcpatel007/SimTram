/****************************************************************************/
/// @file    MsgRetrievingFunction.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Mon, 24 Oct 2003
/// @version $Id: MsgRetrievingFunction.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Encapsulates an object's method for using it as a message retriever
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
#ifndef MsgRetrievingFunction_h
#define MsgRetrievingFunction_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <sstream>
#include <utils/iodevices/OutputDevice.h>
#include "MsgHandler.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MsgRetrievingFunction
 * @brief Encapsulates an object's method for using it as a message retriever
 *
 * You may find an example for this class' usage in GUIRunThread.
 */
template< class T >
class MsgRetrievingFunction : public OutputDevice {
public:
    /// @brief Type of the function to execute.
    typedef void(T::* Operation)(const MsgHandler::MsgType, const std::string&);


    /** @brief Constructor
     * @param[in] object The object to call the method of
     * @param[in] operation The method to call
     * @param[in] type The type of the message
     */
    MsgRetrievingFunction(T* object, Operation operation, MsgHandler::MsgType type) :
        myObject(object),
        myOperation(operation),
        myMsgType(type) {}


    /// @brief Destructor
    ~MsgRetrievingFunction() {}


protected:
    /// @name Methods that override/implement OutputDevice-methods
    /// @{

    /** @brief Returns the associated ostream
     *
     * The stream is an ostringstream, actually, into which the message
     *  is written. It is sent when postWriteHook is called.
     *
     * @return The used stream
     * @see postWriteHook
     */
    std::ostream& getOStream() {
        return myMessage;
    }


    /** @brief Sends the data which was written to the string stream via the retrieving function.
     */
    virtual void postWriteHook() {
        (myObject->*myOperation)(myMsgType, myMessage.str());
        myMessage.str("");
    }
    /// @}


private:
    /// @brief The object the action is directed to.
    T* myObject;

    /// @brief The object's operation to perform.
    Operation myOperation;

    /// @brief The type of message to retrieve.
    MsgHandler::MsgType myMsgType;

    /// @brief message buffer
    std::ostringstream myMessage;

};


#endif

/****************************************************************************/

