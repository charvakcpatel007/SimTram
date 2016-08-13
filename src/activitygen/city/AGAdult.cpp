/****************************************************************************/
/// @file    AGAdult.cpp
/// @author  Piotr Woznica
/// @author  Walter Bamberger
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    July 2010
/// @version $Id: AGAdult.cpp 21182 2016-07-18 06:46:01Z behrisch $
///
// Person in working age: can be linked to a work position.
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2010-2016 DLR (http://www.dlr.de/) and contributors
// activitygen module
// Copyright 2010 TUM (Technische Universitaet Muenchen, http://www.tum.de/)
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

#include "AGAdult.h"
#include "AGWorkPosition.h"
#include <utils/common/RandHelper.h>
#include <iostream>


// ===========================================================================
// method definitions
// ===========================================================================
AGWorkPosition*
AGAdult::randomFreeWorkPosition(std::vector<AGWorkPosition>* wps) {
    std::vector<AGWorkPosition*> freePos;
    for (std::vector<AGWorkPosition>::iterator i = wps->begin(); i!= wps->end(); ++i) {
        if (!i->isTaken()) {
            freePos.push_back(&*i);
        }
    }
    if (freePos.empty()) {
        return 0;
    }
    return RandHelper::getRandomFrom(freePos);
}


AGAdult::AGAdult(int age)
    : AGPerson(age), work(0) {}


void
AGAdult::print() const {
    std::cout << "- AGAdult: Age=" << age << " Work=" << work << std::endl;
}


void
AGAdult::tryToWork(SUMOReal rate, std::vector<AGWorkPosition>* wps) {
    if (decide(rate)) {
        // Select the new work position before giving up the current one.
        // This avoids that the current one is the same as the new one.
        AGWorkPosition* newWork = randomFreeWorkPosition(wps);

        if (work != 0) {
            work->let();
        }
        work = newWork;
        work->take(this);
    } else {
        if (work != 0) {
            // Also sets work = 0 with the call back lostWorkPosition
            work->let();
        }
    }
}


bool
AGAdult::isWorking() const {
    return (work != 0);
}


void
AGAdult::lostWorkPosition() {
    work = 0;
}


void
AGAdult::resignFromWorkPosition() {
    if (work != 0) {
        work->let();
    }
}


const AGWorkPosition&
AGAdult::getWorkPosition() const {
    if (work != 0) {
        return *work;
    }
    throw std::runtime_error("AGAdult::getWorkPosition: Adult is unemployed.");
}

/****************************************************************************/
