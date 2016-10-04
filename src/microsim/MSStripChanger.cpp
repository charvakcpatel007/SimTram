/****************************************************************************/
/// @file    MSStripChanger.cpp
/// @author  Omair Mohammed Abdullah
/// @date    Wed, 27 Apr 2011
/// @version $Id: MSStripChanger.cpp 8593 2010-04-15 13:22:34Z behrisch $
///
// Performs lane changing of vehicles
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

// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#define nullptr 0
#endif

#include "MSStripChanger.h"
#include "MSVehicle.h"
#include "MSVehicleType.h"
#include "MSVehicleTransfer.h"
#include "MSGlobals.h"
#include <cassert>
#include <iterator>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <microsim/MSAbstractLaneChangeModel.h>
#include <utils/common/MsgHandler.h>
#include "MSNet.h"
#include "MSLane.h"
#include "MSVehicleControl.h"
#include "MSStrip.h"

using namespace std;
#ifdef CHECK_MEMORY_LEAKS
#include <foreign/nvwa/debug_new.h>
#endif // CHECK_MEMORY_LEAKS


// ===========================================================================
// member method definitions
// ===========================================================================
MSStripChanger::MSStripChanger(std::vector<MSStrip*>* strips) {
    assert(strips->size() > 1);

    // Fill the changer with the strip-data.
    myChanger.reserve(strips->size());
    for (std::vector<MSStrip*>::iterator strip = strips->begin(); strip != strips->end(); ++strip) {
        ChangeElem ce;
        ce.follow    = 0;
        ce.lead      = 0;
        ce.strip     = *strip;
        ce.lane      = (*strip)->getLane();
        ce.veh       = (*strip)->myVehicles.rbegin();
        ce.hoppedVeh = 0;
        ce.lastBlocked = 0;
        myChanger.push_back(ce);
    }
}

//new modified
MSStripChanger::~MSStripChanger() {}


void
MSStripChanger::laneChange(SUMOTime t) {
    // This is what happens in one time step. After initialization of the
    // changer, each vehicle will try to change. After that the changer
    // needs an update to prevent multiple changes of one vehicle.
    // Finally, the change-result has to be given back to the lanes.
    initChanger();
    //std::cerr<<"\n\nOneIterOfLaneChange\n";
    for (ChangerIt it = myChanger.begin(); it != myChanger.end(); ++it) {
        //it->strip->printDebugMsg();
    }
    //std::cerr<<std::endl;
    while (vehInChanger()) {
        ChangerIt left, right;
        bool haveChanged = change(left, right);
        updateChanger(haveChanged, left, right);
    }   
    updateLanes(t);
}


void
MSStripChanger::initChanger() {
    // Prepare myChanger with a safe state.
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
        ce->lead = 0;
        ce->hoppedVeh = 0;
        ce->lastBlocked = 0;
        ce->dens = 0;
		/*if(ce->strip->getID() == "strip1_6_i_0" && ce->strip->myVehicles.size() >= 2)
		{
			std::string test = "TEST";
		}*/
        MSStrip::VehCont& vehicles = ce->strip->myVehicles;
        if (vehicles.empty()) {
            ce->veh  = vehicles.rend();
            ce->follow = 0;
            continue;
        }
        ce->veh  = vehicles.rbegin();
        if (vehicles.size() == 1) {
            ce->follow = 0;
            continue;
        }
        ce->follow = *(vehicles.rbegin() + 1);
    }
}


bool
MSStripChanger::change(ChangerIt& leftMostStrip, ChangerIt& rightMostStrip) {

    // Find change-candidate. If it is on an allowed lane, try to change
    // to the right (there is a rule in Germany that you have to change
    // to the right, unless you are overtaking). If change to the right
    // isn't possible, check if there is a possibility to overtake (on the
    // left.
    // If candidate isn't on an allowed lane, changing to an allowed has
    // priority.


	 myCandi = findCandidate();
	    MSVehicle* vehicle = veh(myCandi);
	    //std::cerr<<"\nChange called -- "<< myCandi->strip->getID()<<std::endl;


	    const std::vector<MSVehicle::LaneQ> &preb = vehicle->getBestLanes();
	    MSVehicle::StripCont enteredStrips;
	    //std::cerr<<"prebsz="<<preb.size()<<", changersz="<<myChanger.size()<<std::endl;
	    /*assert(preb.size()==myChanger.size());
	    for (int i=0; i<(int) myChanger.size(); ++i) {
	        ((std::vector<MSVehicle::LaneQ>&) preb)[i].occupied = myChanger[i].dens + preb[i].v;
	    }
	    */

	    vehicle->getLaneChangeModel().prepareStep();
	    leftMostStrip = getLeftMostStrip(myCandi);
	    rightMostStrip = getRightMostStrip(myCandi);
	    assert(leftMostStrip >= rightMostStrip);     // index of leftmost strip in greater in myChanger
	    // check whether the vehicle wants and is able to change to right lane
	    std::pair<MSVehicle * const, SUMOReal> leader = getRealThisLeader(myCandi, leftMostStrip, rightMostStrip);
	    std::vector<std::pair<MSVehicle*, SUMOReal> > rLeaders;
	    std::vector<std::pair<MSVehicle*, SUMOReal> > rFollowers;
	    std::vector<std::pair<MSVehicle*, SUMOReal> > lLeaders;
	    std::vector<std::pair<MSVehicle*, SUMOReal> > lFollowers;
       /*
	    std::cout<<"lane change debug"<<vehicle->getID()<<"\n";
	    if(leader.first!=0)
	    std::cout<<"leader"<<leader.first->getID()<<"\n";
       */
	    if(rightMostStrip != myChanger.begin()) //?? ACE ??!D! if rightmost strip of the candidate vehicle is the rightmost strip of the edge(first of myChanger) it wont have right leader and follower!!!
		{
			for (ChangerIt it=rightMostStrip-1; it >= myChanger.begin();--it) 
			{
				std::pair<MSVehicle *, SUMOReal> rLead = getRealRightLeader(it);
				rLeaders.push_back(rLead);
				std::pair<MSVehicle *, SUMOReal> rFollow = getRealRightFollower(it);
				rFollowers.push_back(rFollow);

				//std::cout<<"lane change debug first"<<vehicle->getID()<<"\n";
				if(it == myChanger.begin())
				{
					break;
				}
			}
		}
		/*original....commented
	    for (ChangerIt it=rightMostStrip-1; it != myChanger.begin()-1; --it) {
	        std::pair<MSVehicle *, SUMOReal> rLead = getRealRightLeader(it);
	        rLeaders.push_back(rLead);
	        std::pair<MSVehicle *, SUMOReal> rFollow = getRealRightFollower(it);
	        rFollowers.push_back(rFollow);
	    }*/

	    for (ChangerIt it=leftMostStrip+1; it != myChanger.end(); ++it) {
	        std::pair<MSVehicle *, SUMOReal> lLead = getRealLeftLeader(it);
	        lLeaders.push_back(lLead);
	        std::pair<MSVehicle *, SUMOReal> lFollow = getRealLeftFollower(it);
	        lFollowers.push_back(lFollow);

	    }


   /*

	    for (int i=0; i<lLeaders.size() ;i++)
	    {
	        if(lLeaders[i].first!=0)
	    	std::cout<<"left leaders"<<lLeaders[i].first->getID()<<"\n";

	    }
	    for (int i=0; i<lFollowers.size() ;i++)
	    	    {if(lFollowers[i].first!=0)
	    	    std::cout<<"left followers"<<lFollowers[i].first->getID()<<"\n";

	    	    }
	    for (int i=0; i<rLeaders.size() ;i++)
	    	    {if(rLeaders[i].first!=0)
	    	    std::cout<<"right leaders"<<rLeaders[i].first->getID()<<"\n";

	    	    }

	    for (int i=0; i<rFollowers.size() ;i++)
	   	    	    {if(rFollowers[i].first!=0)
	   	    	    std::cout<<"right followers"<<rFollowers[i].first->getID()<<"\n";

	   	    	    }

*/

	    /*
	    vehicle->printDebugMsg();
	        std::cerr<<"\tRL: "<<(rLead.first != 0 ? rLead.first->getID() : "0")
	                <<"\tRF: "<<(rFollow.first != 0 ? rFollow.first->getID() : "0")
	                <<"\tLL: "<<(lLead.first != 0 ? lLead.first->getID() : "0")
	                <<"\tLF: "<<(lFollow.first != 0 ? lFollow.first->getID() : "0")
	                <<"\tLd: "<<(leader.first != 0 ? leader.first->getID() : "0")<<"\n";
	    */


	    int state1 = (rightMostStrip == myChanger.begin())?0:change2right(leader, rLeaders, rFollowers, preb, rightMostStrip);//?? ACE ??!D! changed from int state1 = change2right(leader, rLeaders, rFollowers, preb, rightMostStrip);
	    bool changingAllowed =
	        (state1&(LCA_BLOCKEDBY_LEADER|LCA_BLOCKEDBY_FOLLOWER))==0;

	    if (state1 & LCA_BLOCKEDBY_LEADER)
	        ;//std::cerr<<"R:Blocked by leader\n";
	    if (state1 & LCA_BLOCKEDBY_FOLLOWER)
	        ;//std::cerr<<"R:Blocked by follower\n";
	    if ((state1 & LCA_SPEEDGAIN) == 0)
	        ;//std::cerr<<"R:Blocked by noSpeedGain\n";

	    if ((state1&LCA_URGENT)!=0||(state1&LCA_SPEEDGAIN)!=0) {
	        state1 |= LCA_RIGHT;
	    }
	    //std::cerr<<"Vehicle in change(): "<<vehicle->getID()<<std::endl;
	    // change if the vehicle wants to and is allowed to change
	    if ((state1&LCA_RIGHT)!=0&&changingAllowed) {
	#ifndef NO_TRACI
	        // inform lane change model about this change
	        vehicle->getLaneChangeModel().fulfillChangeRequest(REQUEST_RIGHT);
	        /*std::cout << "TraCI: lane changer fulfilled request for RIGHT |time " << MSNet::getInstance()->getCurrentTimeStep() << "s" << std::endl;*/
	#endif

	        //std::cerr<<"Ashutosh:"<<vehicle->getID()<<"state"<<state1<<"changingAllowed"<<changingAllowed;
	       // std::cerr<<"!!Vehicle:"<<vehicle->getID()<<" changing to right.\n";
	        // Order is important here, strips of vehicle are from left to right (increasing index),
	        // myChanger Elements are from right to left (increasing index)
	        enteredStrips.clear();
	        for (ChangerIt ci = rightMostStrip-1; ci != leftMostStrip; ++ci) {
	            enteredStrips.push_back(ci->strip);
	            //std::cerr<<"Entered: "<<ci->strip->getID()<<"\t";
	            ci->hoppedVeh = vehicle;
	            ci->strip->myTmpVehicles.push_front(vehicle);
	        }

	        leftMostStrip->strip->leftByStripChange(vehicle);             // handles aggregate vehLenSum
	        vehicle->enterStripsAtStripChange(enteredStrips); // updates myStrips of vehicle
	        (rightMostStrip - 1)->strip->enteredByStripChange(vehicle);    // handles aggregate vehLenSum
	            if (vehicle->getLane().getID() != vehicle->getMainStrip().getLane()->getID()) {
	            //std::cout<<"Lane changed from: "<<vehicle->getLane().getID()
	                 //   <<" to: "<<vehicle->getMainStrip().getLane()->getID()<<std::endl;
	            // Vehicle has moved its main strip to a different lane

	            vehicle->leaveLane(false);
	            vehicle->enterLaneAtLaneChange(vehicle->getMainStrip().getLane());
	        }
	        vehicle->myLastLaneChangeOffset = 0;
	        vehicle->getLaneChangeModel().changed();
	        (rightMostStrip - 1)->dens += (rightMostStrip - 1)->hoppedVeh->getVehicleType().getLength();
	        //std::cout<<"my strips"<<vehicle->getStripIDs()<<"\n";
	        return true;
	    }

	    if ((state1&LCA_RIGHT)!=0&&(state1&LCA_URGENT)!=0) {
	        (rightMostStrip - 1)->lastBlocked = vehicle;
	    }


	    // check whether the vehicle wants and is able to change to left lane
	    int state2 =
	        change2left(leader, lLeaders, lFollowers, preb, leftMostStrip);

	    if ((state2&LCA_URGENT)!=0||(state2&LCA_SPEEDGAIN)!=0) {
	        state2 |= LCA_LEFT;
	    }

	    if (state2 & LCA_BLOCKEDBY_LEADER)

	        ;//std::cerr<<"L:Blocked by leader\n";
	    if (state2 & LCA_BLOCKEDBY_FOLLOWER)
	        ;//std::cerr<<"L:Blocked by follower\n";
	    if ((state2 & LCA_SPEEDGAIN) == 0)
	        ;//std::cerr<<"L:Blocked by noSpeedGain\n";

	    changingAllowed =
	        (state2&(LCA_BLOCKEDBY_LEADER|LCA_BLOCKEDBY_FOLLOWER))==0;
	    vehicle->getLaneChangeModel().setState(state2|state1);
	    // change if the vehicle wants to and is allowed to change
	    if ((state2&LCA_LEFT)!=0&&changingAllowed) {
	#ifndef NO_TRACI
	        // inform lane change model about this change
	        vehicle->getLaneChangeModel().fulfillChangeRequest(REQUEST_LEFT);
	        /*std::cout << "TraCI: lane changer fulfilled request for LEFT |time " << MSNet::getInstance()->getCurrentTimeStep() << "s" << std::endl;*/
	#endif


	        //std::cerr<<" changing allowed = "<<changingAllowed<<" state 2 = "<<state2;
	        //std::cerr<<"!!Vehicle:"<<vehicle->getID()<<" changing to left.\n";
	        enteredStrips.clear();
	        // Order important, refer above
	        for (ChangerIt ci = rightMostStrip+1; ci != leftMostStrip+2; ++ci) {
	            enteredStrips.push_back(ci->strip);
	            //std::cerr<<"Entered: "<<ci->strip->getID()<<"\t";
	            ci->hoppedVeh = vehicle;
	            ci->strip->myTmpVehicles.push_front(vehicle);
	        }
	        rightMostStrip->strip->leftByStripChange(vehicle);
	        vehicle->enterStripsAtStripChange(enteredStrips);
	        (leftMostStrip + 1)->strip->enteredByStripChange(vehicle);
	        if (vehicle->getLane().getID() != vehicle->getMainStrip().getLane()->getID()) {
	            //std::cerr<<"Lane changed from: "<<vehicle->getLane().getID()
	            //        <<" to: "<<vehicle->getMainStrip().getLane()->getID()<<std::endl;
	            // Vehicle has moved its main strip to a different lane
	            vehicle->leaveLane(false);
	            vehicle->enterLaneAtLaneChange(vehicle->getMainStrip().getLane());
	        }
	        vehicle->myLastLaneChangeOffset = 0;
	        vehicle->getLaneChangeModel().changed();
	        (leftMostStrip + 1)->dens += (leftMostStrip + 1)->hoppedVeh->getVehicleType().getLength();
	        return true;
	    }
	    if ((state2&LCA_LEFT)!=0&&(state2&LCA_URGENT)!=0) {
	        (leftMostStrip + 1)->lastBlocked = vehicle;
	    }

	    if ((state1&(LCA_URGENT))!=0&&(state2&(LCA_URGENT))!=0) {
	        // ... wants to go to the left AND to the right
	        // just let them go to the right lane...
	        state2 = 0;
	        vehicle->getLaneChangeModel().setState(state1);
	    }
	    // check whether the vehicles should be swapped
	    if (false&&((state1&(LCA_URGENT))!=0||(state2&(LCA_URGENT))!=0)) {
	        // get the direction ...
	        ChangerIt target;
	        int dir;
	        if ((state1&(LCA_URGENT))!=0) {
	            // ... wants to go right
	            target = myCandi - 1;
	            dir = -1;
	        }
	        if ((state2&(LCA_URGENT))!=0) {
	            // ... wants to go left
	            target = myCandi + 1;
	            dir = 1;
	        }
	        MSVehicle *prohibitor = target->lead;
	        if (target->hoppedVeh!=0) {
	            SUMOReal hoppedPos = target->hoppedVeh->getPositionOnLane();
	            if (prohibitor==0||(
	                        hoppedPos>vehicle->getPositionOnLane() && prohibitor->getPositionOnLane()>hoppedPos)) {

	                prohibitor = 0;// !!! vehicles should not jump over more than one lanetarget->hoppedVeh;
	            }
	        }
	        if (prohibitor!=0
	                &&
	                ((prohibitor->getLaneChangeModel().getState()&(LCA_URGENT/*|LCA_SPEEDGAIN*/))!=0
	                 &&
	                 (prohibitor->getLaneChangeModel().getState()&(LCA_LEFT|LCA_RIGHT))
	                 !=
	                 (vehicle->getLaneChangeModel().getState()&(LCA_LEFT|LCA_RIGHT))
	                )
	           ) {

	            // check for position and speed
	            if (prohibitor->getVehicleType().getLength()-vehicle->getVehicleType().getLength()==0) {
	                // ok, may be swapped
	                // remove vehicle to swap with
	                MSLane::VehCont::iterator i =
	                    find(
	                        target->lane->myTmpVehicles.begin(),
	                        target->lane->myTmpVehicles.end(),
	                        prohibitor);
	                if (i!=target->lane->myTmpVehicles.end()) {
	                    MSVehicle *bla = *i;
	                    assert(bla==prohibitor);
	                    target->lane->myTmpVehicles.erase(i);
	                    // set this vehicle
	                    target->hoppedVeh = vehicle;
	                    target->lane->myTmpVehicles.push_front(vehicle);
	                    myCandi->hoppedVeh = prohibitor;
	                    myCandi->lane->myTmpVehicles.push_front(prohibitor);

	                    // leave lane and detectors
	                    vehicle->leaveLane(false);
	                    prohibitor->leaveLane(false);
	                    // patch position and speed
	                    SUMOReal p1 = vehicle->getPositionOnLane();
	                    vehicle->myState.myPos = prohibitor->myState.myPos;
	                    prohibitor->myState.myPos = p1;
	                    p1 = vehicle->getSpeed();
	                    vehicle->myState.mySpeed = prohibitor->myState.mySpeed;
	                    prohibitor->myState.mySpeed = p1;
	                    // enter lane and detectors
	                    vehicle->enterLaneAtLaneChange(target->lane);
	                    prohibitor->enterLaneAtLaneChange(myCandi->lane);
	                    // mark lane change
	                    vehicle->getLaneChangeModel().changed();
	                    vehicle->myLastLaneChangeOffset = 0;
	                    prohibitor->getLaneChangeModel().changed();
	                    prohibitor->myLastLaneChangeOffset = 0;
	                    (myCandi)->dens += prohibitor->getVehicleType().getLength();
	                    (target)->dens += vehicle->getVehicleType().getLength();
	                    return true;
	                }
	            }
	        }
	    }
	    // Candidate didn't change lane.
	    //std::cerr<<"veh_it: "<<veh(leftMostStrip)->getID()<<"\t"<<"veh_rm: "<<veh(rightMostStrip)->getID()<<std::endl;
	    for (ChangerIt it=rightMostStrip; it != leftMostStrip+1; ++it) {
	        it->strip->myTmpVehicles.push_front(veh(it));
	        it->dens += vehicle->getVehicleType().getLength();
	        //std::cerr<<"Strip: "<<it->strip->getID()<<", veh_it: "<<veh(it)->getID()<<std::endl;
	        assert(veh(it) == veh(rightMostStrip));
              
                 if(veh(it) != veh(rightMostStrip))
                  {
                   MSStrip::removeVehicle(veh(it));
                   cout<<"Remove this vehicle"<<"\n";
                  } 
		/* modified code               
		if(veh(it) == veh(rightMostStrip));
		//MSVehicleTransfer::myVehicles my;
               MSVehicleTransfer *MSVehicleTransfer::myInstance;
		MSVehicleTransfer::myVehicles.erase(veh);
               //my.erase(veh);*/

	    }
	    vehicle->myLastLaneChangeOffset += DELTA_T;
	    return false;

	    }



std::pair<MSVehicle * const, SUMOReal>
MSStripChanger::getRealThisLeader(const ChangerIt &target, ChangerIt leftMostStrip, ChangerIt rightMostStrip) const throw() {
    MSVehicle *vehicle = veh(target);
    assert(vehicle != 0);
    SUMOReal vehPos = vehicle->getPositionOnLane();
    
    // This is duplicated from getPred - needed because we also have hoppedVeh to consider
    MSVehicle* minPred = 0;
    SUMOReal minGap = 99999; // the maximum gap possible (a guess)
    for (ChangerIt it = rightMostStrip; it <= leftMostStrip; ++it) {
        //what about partial occupator?
        MSVehicle *curr = it->lead;

        if (curr == 0 && it->hoppedVeh == 0) continue;
        else if (curr == 0) curr = it->hoppedVeh;
        
        if (it->hoppedVeh && (it->hoppedVeh->getPositionOnLane() < curr->getPositionOnLane()))
            curr = it->hoppedVeh;
        
        SUMOReal gap = curr->getPositionOnLane() - curr->getVehicleType().getLength() - vehPos;
        if (gap < 0)
            gap = it->strip->getPartialOccupatorEnd();
        
        if (gap > 0 && gap < minGap) {
            minGap = gap;
            minPred = curr;
        }
    }
    if (minGap != 99999 && minPred != 0)
        return std::pair<MSVehicle *, SUMOReal>(minPred, minGap);
    return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);
    //TODO: fixit - handle partial occupators properly
    /*MSVehicle *vehicle = veh(target);
    MSVehicle *leader = vehicle->getPred();
    if (leader != 0) {
        assert(&leader->getLane() == &vehicle->getLane());
        SUMOReal gap = leader->getPositionOnLane() - leader->getVehicleType().getLength() - vehicle->getPositionOnLane();
        return std::pair<MSVehicle *, SUMOReal>(leader, gap);
    }
    return std::pair<MSVehicle *, SUMOReal>(0, -1);
    // get the leading vehicle on the lane to change to
    leader = target->lead;
    if (leader==0) {
        MSLane* targetLane = target->lane;
        MSVehicle *predP = targetLane->getPartialOccupator();
        if (predP!=0) {
            return std::pair<MSVehicle *, SUMOReal>(predP, targetLane->getPartialOccupatorEnd() - veh(myCandi)->getPositionOnLane());
        }
        const std::vector<MSLane*> &bestLaneConts = veh(myCandi)->getBestLanesContinuation();
        MSLinkCont::const_iterator link = targetLane->succLinkSec(*veh(myCandi), 1, *targetLane, bestLaneConts);
        if (targetLane->isLinkEnd(link)) {
            return std::pair<MSVehicle *, SUMOReal>(0, -1);
        }
        MSLane *nextLane = (*link)->getLane();
        if (nextLane==0) {
            return std::pair<MSVehicle *, SUMOReal>(0, -1);
        }
        leader = nextLane->getLastVehicle();
        if (leader==0) {
            return std::pair<MSVehicle *, SUMOReal>(0, -1);
        }
        SUMOReal gap =
            leader->getPositionOnLane()-leader->getVehicleType().getLength()
            +
            (myCandi->lane->getLength()-veh(myCandi)->getPositionOnLane());
        return std::pair<MSVehicle * const, SUMOReal>(leader, MAX2((SUMOReal) 0, gap));
    } else {
        MSVehicle *candi = veh(myCandi);
        SUMOReal gap = leader->getPositionOnLane()-leader->getVehicleType().getLength()-candi->getPositionOnLane();
        return std::pair<MSVehicle * const, SUMOReal>(leader, MAX2((SUMOReal) 0, gap));
    }
    */
}


std::pair<MSVehicle *, SUMOReal>
MSStripChanger::getRealLeader(const ChangerIt &target) const throw() {
    // get the leading vehicle on the lane to change to
    MSVehicle* neighLead = target->lead;
    // check whether the hopped vehicle got the leader
    if (target->hoppedVeh!=0) {
        SUMOReal hoppedPos = target->hoppedVeh->getPositionOnLane();
        if (hoppedPos>veh(myCandi)->getPositionOnLane() &&
                (neighLead==0 || neighLead->getPositionOnLane()>hoppedPos)) {

            neighLead = target->hoppedVeh;
        }
    }



    if (neighLead==0) {
        SUMOReal seen = myCandi->lane->getLength() - veh(myCandi)->getPositionOnLane();
        SUMOReal speed = veh(myCandi)->getSpeed();
        SUMOReal dist = veh(myCandi)->getCarFollowModel().brakeGap(speed);
        if (seen>dist) {

        	return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);
        }
        
        return std::pair<MSVehicle *, SUMOReal>(neighLead, myCandi->lane->getLength() - veh(myCandi)->getPositionOnLane());
        
        //TODO: fix along with strip partial occupators
        MSLane* targetLane = target->lane;
        MSVehicle *predP = targetLane->getPartialOccupator();
        if (predP!=0) {
            return std::pair<MSVehicle *, SUMOReal>(predP, targetLane->getPartialOccupatorEnd() - veh(myCandi)->getPositionOnLane());
        }
        const std::vector<MSLane*> &bestLaneConts = veh(myCandi)->getBestLanesContinuation(myCandi->lane);

        if (seen>dist) {
            return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);
        }
        return target->lane->getLeaderOnConsecutive(dist, seen, speed, *veh(myCandi), bestLaneConts);
    } else {
        MSVehicle *candi = veh(myCandi);
        return std::pair<MSVehicle *, SUMOReal>(neighLead,
                neighLead->getPositionOnLane()-neighLead->getVehicleType().getLength()-candi->getPositionOnLane());
    }
}


std::pair<MSVehicle *, SUMOReal>
MSStripChanger::getRealRightLeader(ChangerIt rightMostStrip) const throw() {
    // there is no right lane
    ChangerIt target = rightMostStrip  ; // -1 AB after integration jan 2012


    if (myCandi == myChanger.begin() || target+1 == myChanger.begin()) {

    	return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);
    }
    return getRealLeader(target);
}


std::pair<MSVehicle *, SUMOReal>
MSStripChanger::getRealLeftLeader(ChangerIt leftMostStrip) const throw() {
    // there is no left lane
    ChangerIt target = leftMostStrip ; // +1 AB after integration jan 2012
    if ((myCandi+1) == myChanger.end() || target == myChanger.end()) {
        return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);
    }
    return getRealLeader(target);
}


std::pair<MSVehicle *, SUMOReal>
MSStripChanger::getRealFollower(const ChangerIt &target) const throw() {
    MSVehicle* neighFollow = veh(target);
    // check whether the hopped vehicle got the follower
    if (target->hoppedVeh!=0) {
        SUMOReal hoppedPos = target->hoppedVeh->getPositionOnLane();
        if (hoppedPos<=veh(myCandi)->getPositionOnLane() &&
                (neighFollow==0 || neighFollow->getPositionOnLane()>hoppedPos)) {

            neighFollow = target->hoppedVeh;
        }
    }
    
    if (neighFollow==0) {
        return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);

        //TODO: fix later, like getLeader
        SUMOReal speed = target->lane->getMaxSpeed();
        // in order to look back, we'd need the minimum braking ability of vehicles in the net...
        // we'll assume it to be 4m/s^2
        // !!!revisit
        SUMOReal dist = speed * speed * SUMOReal(1./2.*4.) + SPEED2DIST(speed);
        dist = MIN2(dist, (SUMOReal) 500.);
        MSVehicle *candi = veh(myCandi);
        SUMOReal seen = candi->getPositionOnLane()-candi->getVehicleType().getLength();
        return target->lane->getFollowerOnConsecutive(dist, seen, candi->getSpeed(), candi->getPositionOnLane() - candi->getVehicleType().getLength());
    } else {
        MSVehicle *candi = veh(myCandi);
        return std::pair<MSVehicle *, SUMOReal>(neighFollow,
                candi->getPositionOnLane()-candi->getVehicleType().getLength()-neighFollow->getPositionOnLane());
    }
}


std::pair<MSVehicle *, SUMOReal>
MSStripChanger::getRealRightFollower(ChangerIt rightMostStrip) const throw() {
    // there is no right lane
    ChangerIt target = rightMostStrip ; //-1 AB after integration jan 2012

    if (myCandi == myChanger.begin() || target+1 == myChanger.begin()) {

    	return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);
    }
    return getRealFollower(target);
}


std::pair<MSVehicle *, SUMOReal>
MSStripChanger::getRealLeftFollower(ChangerIt leftMostStrip) const throw() {
    // there is no left lane
    ChangerIt target = leftMostStrip ; //+1 AB after integration jan 2012
    if ((myCandi+1) == myChanger.end() || target == myChanger.end()) {
        return std::pair<MSVehicle *, SUMOReal>(nullptr, -1);
    }
    return getRealFollower(target);
}



void
MSStripChanger::updateChanger(bool vehHasChanged, const ChangerIt &leftMostStrip, const ChangerIt &rightMostStrip) {
    assert(myCandi->veh != myCandi->strip->myVehicles.rend());

    // "Push" the vehicles to the back, i.e. follower becomes vehicle,
    // vehicle becomes leader, and leader becomes predecessor of vehicle,
    // if it exists.
    for (ChangerIt it = rightMostStrip; it <= leftMostStrip; ++it) {
        if (!vehHasChanged) {
            it->lead = veh(it);
        }
        
        //std::cerr<<"UpdateChanger for strip: "<<it->strip->getID()<<std::endl;
        //std::cerr<<"Veh is now1: "<<(*it->veh)->getID()<<", for strip: "<<it->strip->getID()<<std::endl;
        
        it->veh    = it->veh + 1;
        
        /*
        if (it->veh != it->strip->myVehicles.rend())
            std::cerr<<"Veh is now2: "<<(*it->veh)->getID()<<", for strip: "<<it->strip->getID()<<std::endl;
        else std::cerr<<"No more vehicles, for strip: "<<it->strip->getID()<<std::endl;
        */
        
        if (veh(it) == 0) {
            assert(it->follow == 0);
            // leader already 0.
            continue;
        }
        if (it->veh + 1 == it->strip->myVehicles.rend()) {
            it->follow = 0;
        } else {
            it->follow = *(it->veh + 1) ;
        }
    }
    return;
}


void
MSStripChanger::updateLanes(SUMOTime t) {

    // Update the lane's vehicle-container.
    // First: it is bad style to change other classes members, but for
    // this release, other attempts were too time-consuming. In a next
    // release we will change from this lane-centered design to a vehicle-
    // centered. This will solve many problems.
    // Second: this swap would be faster if vehicle-containers would have
    // been pointers, but then I had to change too much of the MSLane code.
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {

        ce->strip->swapAfterLaneChange(t);
    }
}


MSStripChanger::ChangerIt
MSStripChanger::findCandidate() {
    // Find the vehicle in myChanger with the smallest position. If there
    // is no vehicle in myChanger (shouldn't happen) , return
    // myChanger.end().
    ChangerIt max = myChanger.end();
    for (ChangerIt ce = myChanger.begin(); ce != myChanger.end(); ++ce) {
        if (veh(ce) == 0) {
            continue;
        }
        if (max == myChanger.end()) {
            max = ce;
            continue;
        }
        assert(veh(ce)  != 0);
        assert(veh(max) != 0);
        if (veh(max)->getPositionOnLane() < veh(ce)->getPositionOnLane()) {
            max = ce;
        }
    }
    assert(max != myChanger.end());
    assert(veh(max) != 0);
    return max;
}


int
MSStripChanger::change2right(const std::pair<MSVehicle * const, SUMOReal> &leader,
                            const std::vector<std::pair<MSVehicle *, SUMOReal> > &rLead,
                            const std::vector<std::pair<MSVehicle *, SUMOReal> > &rFollow,
                            const std::vector<MSVehicle::LaneQ> &preb, ChangerIt rightMostStrip) const throw() {
    // Try to change to the right-lane if there is one. If this lane isn't
    // an allowed one, cancel the try. Otherwise, check some conditions. If
    // they are simultaneously fulfilled, a change is possible.
    
    ChangerIt target = rightMostStrip -1;
    // no right lane -> exit
    /*?? ACE ?? !D! This is handled at the time/place when/where this function(change2right) is called
	// no right lane -> exit
    if (target+1 == myChanger.begin()) {
        return 0;
    }
	*/
    //std::cerr<<"Change2Right called.\n";
/*....................................................
    //ashutosh Temp lane changing
    MSVehicle* v = veh(myCandi);
    int c=0;
    if (veh(myCandi)->getPositionOnLane()>= 400){
    	const std::vector<MSVehicle::LaneQ> &preb = v->getBestLanes();
    	for (int i=0; i<(int) preb.size(); ++i) {
    		if (target->lane == preb[i].lane) {
    			c = 1;
    		}
        }
		if(c==0)
			return 0;
    }
//..................................................
*/
    if (!target->lane->allowsVehicleClass(veh(myCandi)->getVehicleType().getVehicleClass())) {
        return 0;
    }
    assert(!rLead.empty() && !rFollow.empty());
    int blocked = overlapWithHopped(target)
                  ? target->hoppedVeh->getPositionOnLane()<veh(myCandi)->getPositionOnLane()
                  ? LCA_BLOCKEDBY_FOLLOWER
                  : LCA_BLOCKEDBY_LEADER
                  : 0;
    setOverlap(rLead[0], rFollow[0], blocked);
    setIsSafeChange(rLead[0], rFollow[0], target, blocked);
    return blocked
           |
           advan2right(leader, rLead, rFollow,
                       blocked, preb, target);
}


int
MSStripChanger::change2left(const std::pair<MSVehicle * const, SUMOReal> &leader,
                           const std::vector<std::pair<MSVehicle *, SUMOReal> > &rLead,
                           const std::vector<std::pair<MSVehicle *, SUMOReal> > &rFollow,
                           const std::vector<MSVehicle::LaneQ> &preb, ChangerIt leftMostStrip) const throw() {
    // Try to change to the left-lane, if there is one. If this lane isn't
    // an allowed one, cancel the try. Otherwise,vSafe check some conditions.
    // If they are simultaneously fulfilled, a change is possible.
    
    // no left lane, overlapping or left lane not allowed -> exit
    ChangerIt target = leftMostStrip +1;
    if (target == myChanger.end()) {
        return 0;
    }
    //std::cerr<<"Change2Left called.\n";
    /*....................................................
        //ashutosh Temp lane changing
    int c =0;
     MSVehicle* v = veh(myCandi);
     if (veh(myCandi)->getPositionOnLane()>= 400){
        	const std::vector<MSVehicle::LaneQ> &preb = v->getBestLanes();
        	for (int i=0; i<(int) preb.size(); ++i) {
        		if (target->lane == preb[i].lane){
        			c==1;
        		}
        	}
        	if(c==0)
        	  return 0;
        }
    //..................................................
     */

    if (!target->lane->allowsVehicleClass(veh(myCandi)->getVehicleType().getVehicleClass())) {
        return 0;
    }
    assert(!rLead.empty() && !rFollow.empty());
    int blocked = overlapWithHopped(target)
                  ? target->hoppedVeh->getPositionOnLane()<veh(myCandi)->getPositionOnLane()
                  ? LCA_BLOCKEDBY_FOLLOWER
                  : LCA_BLOCKEDBY_LEADER
                  : 0;
    setOverlap(rLead[0], rFollow[0], blocked);
    setIsSafeChange(rLead[0], rFollow[0], target, blocked);
    return blocked
           |
           advan2left(leader, rLead, rFollow,
                      blocked, preb, target);
}


void
MSStripChanger::setOverlap(const std::pair<MSVehicle * const, SUMOReal> &rLead,
                          const std::pair<MSVehicle * const, SUMOReal> &rFollow,
                          int &blocked) const throw() {
    // check the follower only if not already known that...
    if ((blocked&LCA_BLOCKEDBY_FOLLOWER)==0) {
        if (rFollow.first!=0&&rFollow.second<0) {
            blocked |= (LCA_BLOCKEDBY_FOLLOWER|LCA_OVERLAPPING);
        }
    }
    // check the leader only if not already known that...
    if ((blocked&LCA_BLOCKEDBY_LEADER)==0) {
        if (rLead.first!=0&&rLead.second<0) {
            blocked |= (LCA_BLOCKEDBY_LEADER|LCA_OVERLAPPING);
        }
    }
}


void
MSStripChanger::setIsSafeChange(const std::pair<MSVehicle * const, SUMOReal> &neighLead,
                               const std::pair<MSVehicle * const, SUMOReal> &neighFollow,
                               const ChangerIt &target, int &blocked) const throw() {
    // Check if candidate's change to target-lane will be safe, i.e. is there
    // enough back-gap to the neighFollow to drive collision-free (if there is
    // no neighFollow, keep a safe-gap to the beginning of the lane) and is
    // there enough gap for the candidate to neighLead to drive collision-
    // free (if there is no neighLead, be sure that candidate is able to slow-
    // down towards the lane end).
    MSVehicle* vehicle     = veh(myCandi);

    // check back gap
    if ((blocked&LCA_BLOCKEDBY_FOLLOWER)==0) {
        if (neighFollow.first!=0) {

        	MSLane* targetLane = target->lane;
            // !!! actually: VSAFE needs the max speed of both tracks
            if (!neighFollow.first->getCarFollowModel().hasSafeGap(neighFollow.first->getSpeed(), neighFollow.second, vehicle->getSpeed(), targetLane->getMaxSpeed())) {
                blocked |= LCA_BLOCKEDBY_FOLLOWER;

            }
        }
    }

    // check front gap
    if ((blocked&LCA_BLOCKEDBY_LEADER)==0) {
        if (neighLead.first!=0) {

        	MSLane* targetLane = target->lane;
            // !!! actually: VSAFE needs the max speed of both tracks
            if (!vehicle->getCarFollowModel().hasSafeGap(vehicle->getSpeed(), neighLead.second, neighLead.first->getSpeed(), targetLane->getMaxSpeed())) {
                blocked |= LCA_BLOCKEDBY_LEADER;

            }
        }
    }
}


int
MSStripChanger::advan2right(const std::pair<MSVehicle * const, SUMOReal> &leader,
                           const std::vector<std::pair<MSVehicle *, SUMOReal> > &neighLead,
                           const std::vector<std::pair<MSVehicle *, SUMOReal> > &neighFollow,
                           int blocked,
                           const std::vector<MSVehicle::LaneQ> &preb,
                           ChangerIt target) const throw() {
    std::vector<MSStrip *> strips;
    for (ChangerIt it=target; it>=myChanger.begin(); --it) {//?? ACE ??!D! original :for (ChangerIt it=target; it!=myChanger.begin()-1; --it) {
        strips.push_back(it->strip);
		if(it == myChanger.begin())//?? ACE ??!D!
		{
			break;
		}
    }
    MSAbstractLaneChangeModel::MSLCMessager
    msg(leader.first, neighLead[0].first, neighFollow[0].first);

    return veh(myCandi)->getLaneChangeModel().wantsChangeToRight(
               msg, blocked,
               leader, neighLead, neighFollow, strips,
               preb,
               &(myCandi->lastBlocked));

    }


int
MSStripChanger::advan2left(const std::pair<MSVehicle * const, SUMOReal> &leader,
                          const std::vector<std::pair<MSVehicle *, SUMOReal> > &neighLead,
                          const std::vector<std::pair<MSVehicle *, SUMOReal> > &neighFollow,
                          int blocked,
                          const std::vector<MSVehicle::LaneQ> &preb,
                           ChangerIt target) const throw() {
    std::vector<MSStrip *> strips;
    for (ChangerIt it=target; it!=myChanger.end(); ++it) {
        strips.push_back(it->strip);
    }
    MSAbstractLaneChangeModel::MSLCMessager
    msg(leader.first, neighLead[0].first, neighFollow[0].first);
    //TODO: instead of getLane, it should be the lane of the next strip

    return veh(myCandi)->getLaneChangeModel().wantsChangeToLeft(
               msg, blocked,
               leader, neighLead, neighFollow, strips,
               preb,
               &(myCandi->lastBlocked));
}


MSStripChanger::ChangerIt
MSStripChanger::getLeftMostStrip(ChangerIt candi) {
    MSVehicle *vehicle = veh(candi);
    assert(vehicle != 0 && candi < myChanger.end() && candi >= myChanger.begin());
    ChangerIt ce = candi;
    
    for (; ce != myChanger.end(); ++ce) {
        if (ce->strip->getID() == vehicle->getLeftStrip()->getID())
            break;
    }
    assert (ce != myChanger.end());
    //std::cerr<<"LeftMostStrip: "<<ce->strip->getID()<<"::"<<ce->strip->getNumericalID()<<std::endl;
    return ce;
}

MSStripChanger::ChangerIt
MSStripChanger::getRightMostStrip(ChangerIt candi) {
    MSVehicle *vehicle = veh(candi);
    assert(vehicle != 0 && candi < myChanger.end() && candi >= myChanger.begin());
    ChangerIt ce = candi;
    
    for (; ce >= myChanger.begin(); --ce) {
        if (ce->strip->getID() == vehicle->getRightStrip()->getID())
            break;
    }
	assert(ce >= myChanger.begin());
    //assert(ce != myChanger.begin()-1);
    //std::cerr<<"RightMostStrip: "<<ce->strip->getID()<<"::"<<ce->strip->getNumericalID()<<std::endl;
    return ce;
}
/****************************************************************************/

