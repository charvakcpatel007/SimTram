/****************************************************************************/
/// @file    GUIGlObjectStorage.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Oct 2002
/// @version $Id: GUIGlObjectStorage.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A storage for displayed objects via their numerical id
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
#ifndef GUIGlObjectStorage_h
#define GUIGlObjectStorage_h


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
#include <set>
#include <fx.h>
#include "GUIGlObject.h"
#include <utils/foxtools/MFXMutex.h>


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class GUIGlObjectStorage
 * @brief A storage for of displayed objects via their numerical id
 *
 * This is a container for GUIGlObject - objects, which may be displayed
 *  and due to this may generate tooltips or be grapped in other ways.
 *
 * As in case of vehicles (other, later implemented objects may have this
 *  property, too) they may be deleted by the simulation while being accessed
 *  - for example using a property window or something like that - this
 *  container posesses three storages: one containing all objects that are not
 *  accessed at all, one for objects currently accessed and one for objects that
 *  are accessed but shall be deleted.
 */
class GUIGlObjectStorage {
public:
    /// @brief Constructor
    GUIGlObjectStorage();


    /// @brief Destructor
    ~GUIGlObjectStorage();


    /** @brief Registers an object
     *
     * This done within the constructor of the GUIGlObject; The object's "setGLID"
     *  method is called giving the next free id.
     *
     * @param[in] object The object to register
     * @param[in] fullName The full name of the object to register
     * @return the GUIGlObject under which the object has been registered
     */
    GUIGlID registerObject(GUIGlObject* object, const std::string& fullName);


    /** @brief Returns the object from the container locking it
     *
     * The lock prevents the object from being deleted while it is accessed.
     * The object is moved from "myMap" to "myBlocked".
     *
     * @param[in] id The id of the object to return
     * @return The object with the given id or 0 if no such object is known
     */
    GUIGlObject* getObjectBlocking(GUIGlID id);


    /** @brief Returns the object from the container locking it
     *
     * The lock prevents the object from being deleted while it is accessed.
     * The object is moved from "myMap" to "myBlocked".
     *
     * @param[in] id The id of the object to return
     * @return The object with the given id or 0 if no such object is known
     */
    GUIGlObject* getObjectBlocking(const std::string& fullName);


    /** @brief Removes the named object from this container
     *
     * This function returns true if the object may be deleted;
     *  otherwise it's kept in an internal storage (for visualisation etc.)
     *  and will be removed by this class
     *
     * @param[in] id The id of the object to remove
     * @return Whether the object could be removed (and may be deleted)
     */
    bool remove(GUIGlID id);


    /** @brief Clears this container
     *
     * The objects are not deleted.
     */
    void clear();


    /** @brief Marks an object as unblocked
     *
     * The object is moved from "myBlocked" to "myMap".
     * @param[in] id The id of the object to unblock
     */
    void unblockObject(GUIGlID id);


    /** @brief Sets the given object as the "network" object
     * @param[in] object The object to set as network object
     */
    void setNetObject(GUIGlObject* object) {
        myNetObject = object;
    }


    /** @brief Returns the network object
     * @return The network object
     */
    GUIGlObject* getNetObject() const {
        return myNetObject;
    }


    /** @brief A single static instance of this class
     */
    static GUIGlObjectStorage gIDStorage;


    /** @brief Returns the set of all known ids
     */
    std::set<GUIGlID> getAllIDs() const;

private:
    /// @brief Definition of a container from numerical ids to objects
    typedef std::map<GUIGlID, GUIGlObject*> ObjectMap;

    /// @brief The known objects which are not accessed currently
    ObjectMap myMap;

    /* @brief The known objects by their fill name (used when loading selection
     * from file */
    std::map<std::string, GUIGlObject*>  myFullNameMap;

    /// @brief The currently accessed objects
    ObjectMap myBlocked;

    /// @brief Objects to delete
    ObjectMap my2Delete;

    /// @brief The next id to give; initially zero, increased by one with each object registration
    GUIGlID myAktID;

    /// @brief A lock to avoid parallel access on the storages
    mutable MFXMutex myLock;

    /// @brief The network object
    GUIGlObject* myNetObject;


private:
    /// @brief invalidated copy constructor
    GUIGlObjectStorage(const GUIGlObjectStorage& s);

    /// @brief invalidate assignment operator
    GUIGlObjectStorage& operator=(const GUIGlObjectStorage& s);


};


#endif

/****************************************************************************/

