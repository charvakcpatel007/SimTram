/****************************************************************************/
/// @file    MSStripChanger.h
/// @author  Omair Mohammed Abdullah
/// @date    Wed, 27 Apr 2011
/// @version $Id: MSStripChanger.h 8593 2010-04-15 13:22:34Z behrisch $
///
// Performs lane changing of vehicles (strip by strip)
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
// Copyright 2001-2010 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef MSStripChanger_h
#define MSStripChanger_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include "MSStrip.h"
#include "MSLane.h"
#include "MSEdge.h"
#include <vector>
#include <utils/iodevices/OutputDevice.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSVehicle;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSStripChanger
 * @brief Performs lane changing of vehicles
 */
class MSStripChanger {
public:
    /// Constructor
    MSStripChanger(std::vector<MSStrip*>* strips);

    /// Destructor.
    ~MSStripChanger();

    /// Start lateral-movement-process for all vehicles on the edge's lane's strips.
    void laneChange(SUMOTime t);

public:
    /** Structure used for lane-change. For every lane you have to
        know four vehicles, the change-candidate veh and it's follower
        and leader. Further, information about the last vehicle that changed
        into this lane is needed */
    struct ChangeElem {
        /// the vehicle following the current vehicle
        MSVehicle*                follow;
        /// the vehicle in front of the current vehicle
        MSVehicle*                lead;
        /// the strip the vehicle is on
        MSStrip*                  strip;
        /// the lane the vehicle is on
        MSLane*                   lane;
        /// the regarded vehicle
        MSStrip::VehCont::reverse_iterator veh;
        /// last vehicle that changed into this strip
        MSVehicle*                hoppedVeh;
        /// the vehicle that really wants to change to this strip
        MSVehicle*                lastBlocked;

        SUMOReal dens;

    };

public:
    /** @brief The list of changers;
        For each lane, a ChangeElem is being build */
    typedef std::vector< ChangeElem > Changer;

    /// the iterator moving over the ChangeElems
    typedef Changer::iterator ChangerIt;

    /// the iterator moving over the ChangeElems
    typedef Changer::const_iterator ConstChangerIt;

protected:
    /// Initialize the changer before looping over all vehicles.
    void initChanger();

    /** @brief Check if there is a single change-candidate in the changer.
        Returns true if there is one. */
    bool vehInChanger() const throw() {
        // If there is at least one valid vehicle under the veh's in myChanger
        // return true.
        for (ConstChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
            if (veh(ce) != 0) {
                return true;
            }
        }
        return false;
    }

    /** Returns a pointer to the changer-element-iterator vehicle, or 0 if
        there is none. */
    MSVehicle* veh(ConstChangerIt ce) const throw() {
        // If ce has a valid vehicle, return it. Otherwise return 0.
        if (ce->veh != ce->strip->myVehicles.rend()) {
            return *(ce->veh);
        }
        return 0;
    }


    /** Find a new candidate and try to change it. */
    bool change(ChangerIt &left, ChangerIt &right);

    /** After the possible change, update the changer. */
    void updateChanger(bool vehHasChanged, const ChangerIt &left, const ChangerIt &right);

    /** During lane-change a temporary vehicle container is filled within
        the lanes (bad pratice to modify foreign members, I know). Swap
        this container with the real one. */
    void updateLanes(SUMOTime t);

    /** @brief Find current candidate.
        If there is none, myChanger.end() is returned. */
    ChangerIt findCandidate();

    int change2right(
        const std::pair<MSVehicle * const, SUMOReal> &leader,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &rLead,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &rFollow,
        const std::vector<MSVehicle::LaneQ> &preb,
        ChangerIt rightMostStrip) const throw();

    int change2left(
        const std::pair<MSVehicle * const, SUMOReal> &leader,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &lLead,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &lFollow,
        const std::vector<MSVehicle::LaneQ> &preb,
        ChangerIt leftMostStrip) const throw();

    void setOverlap(const std::pair<MSVehicle * const, SUMOReal> &neighLead,
                    const std::pair<MSVehicle * const, SUMOReal> &neighFollow,
                    /*const ChangerIt &target,*/ int &blocked) const throw();





    void setIsSafeChange(const std::pair<MSVehicle * const, SUMOReal> &neighLead,
                         const std::pair<MSVehicle * const, SUMOReal> &neighFollow,
                         const ChangerIt &target, int &blocked) const throw();

    /** Returns true, if candidate has an advantage by changing to the
        right. */
    int advan2right(
        const std::pair<MSVehicle * const, SUMOReal> &leader,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &rLead,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &rFollow,
        int blocked,
        const std::vector<MSVehicle::LaneQ> &preb, ChangerIt target) const throw();

    /** Returns true, if candidate has an advantage by changing to the
        left. */
    int advan2left(
        const std::pair<MSVehicle * const, SUMOReal> &leader,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &rLead,
        const std::vector<std::pair<MSVehicle *, SUMOReal> > &rFollow,
        int blocked,
        const std::vector<MSVehicle::LaneQ> &preb, ChangerIt target) const throw();

    /** Returns true if candidate overlaps with a vehicle, that
        already changed the lane.*/
    bool overlapWithHopped(ChangerIt target) const throw() {
        MSVehicle *v1 = target->hoppedVeh;
        MSVehicle *v2 = veh(myCandi);
        if (v1!=0 && v2!=0) {
            return MSVehicle::overlap(v1, v2);
        }
        return false;
    }

    std::pair<MSVehicle * const, SUMOReal> getRealThisLeader(const ChangerIt &target, ChangerIt, ChangerIt) const throw();

    std::pair<MSVehicle *, SUMOReal> getRealFollower(const ChangerIt &target) const throw();
    std::pair<MSVehicle *, SUMOReal> getRealRightFollower(ChangerIt) const throw();
    std::pair<MSVehicle *, SUMOReal> getRealLeftFollower(ChangerIt) const throw();

    std::pair<MSVehicle *, SUMOReal> getRealLeader(const ChangerIt &target) const throw();
    std::pair<MSVehicle *, SUMOReal> getRealRightLeader(ChangerIt) const throw();
    std::pair<MSVehicle *, SUMOReal> getRealLeftLeader(ChangerIt) const throw();

    ChangerIt getLeftMostStrip(ChangerIt candi);
    ChangerIt getRightMostStrip(ChangerIt candi);
protected:
    /// Container for ChangeElemements, one for every lane in the edge.
    Changer   myChanger;

    /** Change-candidate. Last of the vehicles in changer. Only this one
        will try to change. Every vehicle on the edge will be a candidate
        once in the change-process. */
    ChangerIt myCandi;

private:
    /// Default constructor.
    MSStripChanger();

    /// Copy constructor.
    MSStripChanger(const MSStripChanger&);

    /// Assignment operator.
    MSStripChanger& operator=(const MSStripChanger&);
};


#endif

/****************************************************************************/

