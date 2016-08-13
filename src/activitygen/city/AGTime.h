/****************************************************************************/
/// @file    AGTime.h
/// @author  Piotr Woznica
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @date    July 2010
/// @version $Id: AGTime.h 20433 2016-04-13 08:00:14Z behrisch $
///
// Time manager: able to manipulate the time using Sumo's format (seconds)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2001-2016 DLR (http://www.dlr.de/) and contributors
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
#ifndef AGTIME_H
#define AGTIME_H


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <iostream>


// ===========================================================================
// class definitions
// ===========================================================================
class AGTime {
public:
    AGTime() {};
    AGTime(int seconds) :
        sec(seconds) {};
    AGTime(int hour, int minutes) :
        sec(convert(0, hour, minutes, 0)) {};
    AGTime(int day, int hour, int min) :
        sec(convert(day, hour, min, 0)) {};
    AGTime(int day, int hour, int min, int sec) :
        sec(convert(day, hour, min, sec)) {};
    AGTime(const AGTime& time);
    bool operator==(const AGTime& time);
    bool operator<(const AGTime& time);
    bool operator<=(const AGTime& time);
    void operator+=(const AGTime& time);
    void operator+=(int seconds);
    void operator-=(const AGTime& time);
    AGTime operator+(const AGTime& time);

    /********************
     * In/Out functions *
     ********************/
    int getDay();
    int getHour();
    int getMinute();
    int getSecond();
    int getSecondsInCurrentDay();
    /**
     * @brief: returns the number of seconds from the beginning of the first day of simulation
     * this includes
     */
    int getTime();

    void setDay(int d);
    void setHour(int h);
    void setMinute(int m);
    void setSecond(int s);
    /**
     * @brief: sets the time from the beginning of the first day of simulation in seconds
     */
    void setTime(int sec);


    /**************************
     * Manipulation functions *
     **************************/
    /**
     * @brief addition of seconds to the current moment
     *
     * @param[in] sec the number of seconds
     */
    void addSeconds(int sec);

    /**
     * @brief addition of minutes to the current moment
     *
     * @param[in] min the number of minutes
     */
    void addMinutes(int min);

    /**
     * @brief addition of hours to the current moment
     *
     * @param[in] hours the number of hours to add
     */
    void addHours(int hours);

    /**
     * @brief addition of days to the current moment
     *
     * @param[in] days the number of days to add
     */
    void addDays(int days);

    /**
     * @brief computes the number of seconds in the given minutes
     *
     * @param[in] minutes, can be fraction of minutes
     *
     * @return number of seconds
     */
    int getSecondsOf(SUMOReal minutes);

private:
    /**
     * @brief converts days, hours and minutes to seconds
     */
    int convert(int days, int hours, int minutes, int seconds);


    // @brief: the seconds representing this date (day, hour, minute)
    // @brief: used for in/out
    int sec;
};

#endif

/****************************************************************************/
