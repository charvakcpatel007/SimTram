/****************************************************************************/
/// @file    Distribution_MeanDev.h
/// @author  Daniel Krajzewicz
/// @date    Sept 2002
/// @version $Id: Distribution_MeanDev.h 20433 2016-04-13 08:00:14Z behrisch $
///
// A distribution described by the mean value and std-dev amount
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
#ifndef Distribution_MeanDev_h
#define Distribution_MeanDev_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "Distribution.h"

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Distribution_MeanDev
 * A description of distribution by the distribution's mean value and a
 *  standard deviation.
 * Incomplete and unused yet. This class should be overridden by derived
 *  classes
 */
class Distribution_MeanDev :
    public Distribution {
public:
    /// Constructor
    Distribution_MeanDev(const std::string& id, SUMOReal mean,
                         SUMOReal deviation);

    /// Destructor
    virtual ~Distribution_MeanDev();

    /// Returns the maximum value of this distribution
    SUMOReal getMax() const;

private:
    /// The distribution's mean
    SUMOReal myMeanValue;

    /// The distribution's standard deviation
    SUMOReal myDeviation;

};


#endif

/****************************************************************************/

