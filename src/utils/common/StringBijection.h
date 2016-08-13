/****************************************************************************/
/// @file    StringBijection.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Mar 2011
/// @version $Id: StringBijection.h 21202 2016-07-19 13:40:35Z behrisch $
///
// Bidirectional map between string and something else
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2011-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef StringBijection_h
#define StringBijection_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <utils/common/UtilExceptions.h>

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * Template container for maintaining a bidirectional map between strings and something else
 * It is not always a bijection since it allows for duplicate entries on both sides if either
 * checkDuplicates is set to false in the constructor or the insert function or if
 * the addAlias function is used.
 */

template< class T  >
class StringBijection {

public:

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4510 4512 4610) // no default constructor and no assignment operator; conflicts with initializer
#endif
    struct Entry {
        const char* str;
        const T key;
    };
#ifdef _MSC_VER
#pragma warning(pop)
#endif


    StringBijection() {}


    StringBijection(Entry entries[], T terminatorKey, bool checkDuplicates = true) {
        int i = 0;
        do {
            insert(entries[i].str, entries[i].key, checkDuplicates);
        } while (entries[i++].key != terminatorKey);
    }


    void insert(const std::string str, const T key, bool checkDuplicates = true) {
        if (checkDuplicates) {
            if (has(key)) {
                // cannot use toString(key) because that might create an infinite loop
                throw InvalidArgument("Duplicate key.");
            }
            if (hasString(str)) {
                throw InvalidArgument("Duplicate string '" + str + "'.");
            }
        }
        myString2T[str] = key;
        myT2String[key] = str;
    }


    void addAlias(const std::string str, const T key) {
        myString2T[str] = key;
    }


    void remove(const std::string str, const T key) {
        myString2T.erase(str);
        myT2String.erase(key);
    }


    T get(const std::string& str) const {
        if (hasString(str)) {
            return myString2T.find(str)->second;
        } else {
            throw InvalidArgument("String '" + str + "' not found.");
        }
    }


    const std::string& getString(const T key) const {
        if (has(key)) {
            return myT2String.find(key)->second;
        } else {
            // cannot use toString(key) because that might create an infinite loop
            throw InvalidArgument("Key not found.");
        }
    }


    bool hasString(const std::string& str) const {
        return myString2T.count(str) != 0;
    }


    bool has(const T key) const {
        return myT2String.count(key) != 0;
    }


    int size() const {
        return (int)myString2T.size();
    }


    std::vector<std::string> getStrings() const {
        std::vector<std::string> result;
        typename std::map<T, std::string>::const_iterator it; // learn something new every day
        for (it = myT2String.begin(); it != myT2String.end(); it++) {
            result.push_back(it->second);
        }
        return result;
    }


    void addKeysInto(std::vector<T>& list) const {
        typename std::map<T, std::string>::const_iterator it; // learn something new every day
        for (it = myT2String.begin(); it != myT2String.end(); it++) {
            list.push_back(it->first);
        }
    }


private:
    std::map<std::string, T> myString2T;
    std::map<T, std::string> myT2String;

};

#endif

/****************************************************************************/

