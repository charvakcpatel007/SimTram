/****************************************************************************/
/// @file    GLObjectValuePassConnector.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Fri, 29.04.2005
/// @version $Id: GLObjectValuePassConnector.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Class passing values from a GUIGlObject to another object
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
#ifndef GLObjectValuePassConnector_h
#define GLObjectValuePassConnector_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <algorithm>
#include <vector>
#include <map>
#include <functional>
#include <utils/common/ValueSource.h>
#include <utils/common/ValueRetriever.h>
#include <utils/gui/globjects/GUIGlObject.h>
#include <utils/foxtools/MFXMutex.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GUIGlObject;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GLObjectValuePassConnector
 * @brief Class passing values from a GUIGlObject to another object
 *
 * A templated instance has some static member variables. They have to be defined
 *  in a cpp file. They may be found in GUINet.cpp. Two instances are used:
 * - one passing SUMOReal-values
 * - one passing time tls phase definitions
 *
 * @see GUIGlObject
 */
template<typename T>
class GLObjectValuePassConnector {
public:
    /** @brief Constructor
     * @param[in] o The object to get the value from
     * @param[in] source The method for obtaining the value
     * @param[in] retriever The object to pass the value to
     */
    GLObjectValuePassConnector(GUIGlObject& o, ValueSource<T>* source, ValueRetriever<T>* retriever)
        : myObject(o), mySource(source), myRetriever(retriever) { /*, myIsInvalid(false) */
        AbstractMutex::ScopedLocker locker(myLock);
        myContainer.push_back(this);
    }


    /// @brief Destructor
    virtual ~GLObjectValuePassConnector() {
        myLock.lock();
        typename std::vector< GLObjectValuePassConnector<T>* >::iterator i = std::find(myContainer.begin(), myContainer.end(), this);
        if (i != myContainer.end()) {
            myContainer.erase(i);
        }
        myLock.unlock();
        delete mySource;
    }


    /// @name static methods for interactions
    /// @{

    /** @brief Updates all instances (passes values)
     */
    static void updateAll() {
        AbstractMutex::ScopedLocker locker(myLock);
        std::for_each(myContainer.begin(), myContainer.end(), std::mem_fun(&GLObjectValuePassConnector<T>::passValue));
    }


    /** @brief Deletes all instances
     */
    static void clear() {
        AbstractMutex::ScopedLocker locker(myLock);
        while (!myContainer.empty()) {
            delete(*myContainer.begin());
        }
        myContainer.clear();
    }


    /** @brief Removes all instances that pass values from the object with the given id
     *
     * Used if for example a vehicle leaves the network
     * @param[in] o The object which shall no longer be asked for values
     */
    static void removeObject(GUIGlObject& o) {
        AbstractMutex::ScopedLocker locker(myLock);
        for (typename std::vector< GLObjectValuePassConnector<T>* >::iterator i = myContainer.begin(); i != myContainer.end();) {
            if ((*i)->myObject.getGlID() == o.getGlID()) {
                i = myContainer.erase(i);
            } else {
                ++i;
            }
        }
    }
    /// @}


protected:
    /** @brief Passes the value to the retriever
     *
     * Retrieves the value from the object, in the case the object is active.
     *  Passes the value to the retriever.
     * @see GUIGlObject::active
     */
    virtual bool passValue() {
        myRetriever->addValue(mySource->getValue());
        return true;
    }


protected:
    /// @brief The object to get the values of (the object that must be active)
    GUIGlObject& myObject;

    /// @brief The source for values
    ValueSource<T>* mySource;

    /// @brief The destination for values
    ValueRetriever<T>* myRetriever;

    /// @brief The mutex used to avoid concurrent updates of the connectors container
    static MFXMutex myLock;

    /// @brief The container of items that shall be updated
    static std::vector< GLObjectValuePassConnector<T>* > myContainer;


private:
    /// @brief Invalidated copy constructor.
    GLObjectValuePassConnector<T>(const GLObjectValuePassConnector<T>&);

    /// @brief Invalidated assignment operator.
    GLObjectValuePassConnector<T>& operator=(const GLObjectValuePassConnector<T>&);


};


template<typename T>
std::vector< GLObjectValuePassConnector<T>* > GLObjectValuePassConnector<T>::myContainer;
template<typename T>
MFXMutex GLObjectValuePassConnector<T>::myLock;


#endif

/****************************************************************************/

