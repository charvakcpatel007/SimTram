/****************************************************************************/
/// @file    SUMOXMLDefinitions.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Clemens Honomichl
/// @author  Piotr Woznica
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @date    Sept 2002
/// @version $Id: SUMOXMLDefinitions.h 21062 2016-06-29 11:50:54Z palcraft $
///
// Definitions of elements and attributes known by SUMO
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
// Copyright (C) 2002-2016 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/
#ifndef SUMOXMLDefinitions_h
#define SUMOXMLDefinitions_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <utils/common/StringBijection.h>

// ===========================================================================
// definitions
// ===========================================================================
/**
 * @enum SumoXMLTag
 * @brief Numbers representing SUMO-XML - element names
 * @see GenericSAXHandler
 * @see SUMOSAXHandler
 */
enum SumoXMLTag {
    /** invalid tag */
    SUMO_TAG_NOTHING,
    /** root element of a network file */
    SUMO_TAG_NET,
    /** begin/end of the description of an edge */
    SUMO_TAG_EDGE,
    /** begin/end of the description of a single lane */
    SUMO_TAG_LANE,
    /** begin/end of the description of a neighboring lane */
    SUMO_TAG_NEIGH,
    /** begin/end of the description of a polygon */
    SUMO_TAG_POI,
    /** begin/end of the description of a polygon */
    SUMO_TAG_POLY,
    /** begin/end of the description of a junction */
    SUMO_TAG_JUNCTION,
    /** begin/end of the description of an edge restriction */
    SUMO_TAG_RESTRICTION,
    /** an e1 detector */
    SUMO_TAG_E1DETECTOR,
    SUMO_TAG_INDUCTION_LOOP,
    /** an e2 detector */
    SUMO_TAG_E2DETECTOR,
    SUMO_TAG_LANE_AREA_DETECTOR,
    /** an e3 detector */
    SUMO_TAG_E3DETECTOR,
    SUMO_TAG_ENTRY_EXIT_DETECTOR,
    /** an edge based mean data detector */
    SUMO_TAG_MEANDATA_EDGE,
    /** a lane based mean data detector */
    SUMO_TAG_MEANDATA_LANE,
    /** an e3 entry point*/
    SUMO_TAG_DET_ENTRY,
    /** an e3 exit point */
    SUMO_TAG_DET_EXIT,
    /// @brief An edge-following detector
    SUMO_TAG_EDGEFOLLOWDETECTOR,
    /// @brief An instantenous induction loop
    SUMO_TAG_INSTANT_INDUCTION_LOOP,
    /// @brief A variable speed sign
    SUMO_TAG_VSS,
    /// @brief A variable speed sign over lane
    SUMO_TAG_VSSLANE,
    /// @brief A calibrator
    SUMO_TAG_CALIBRATOR,
    /// @brief A rerouter
    SUMO_TAG_REROUTER,
    /// @brief A bus stop
    SUMO_TAG_BUS_STOP,
    /// @brief A train stop (alias for bus stop)
    SUMO_TAG_TRAIN_STOP,
    /// @brief An access point for a train stop
    SUMO_TAG_ACCESS,
    /// @brief A container stop
    SUMO_TAG_CONTAINER_STOP,
    /// @brief A Charging Station
    SUMO_TAG_CHARGING_STATION,
    /** a vtypeprobe detector */
    SUMO_TAG_VTYPEPROBE,
    /** a routeprobe detector */
    SUMO_TAG_ROUTEPROBE,
    /** root element of a route file */
    SUMO_TAG_ROUTES,
    /** description of a vehicle */
    SUMO_TAG_VEHICLE,
    /** description of a vehicle type */
    SUMO_TAG_VTYPE,
    /** begin/end of the description of a route */
    SUMO_TAG_ROUTE,
    /** description of a logic request within the junction */
    SUMO_TAG_REQUEST,
    /** a source */
    SUMO_TAG_SOURCE,
    /** a traffic assignment zone */
    SUMO_TAG_TAZ,
    /** a source within a district (connection road) */
    SUMO_TAG_TAZSOURCE,
    /** a sink within a district (connection road) */
    SUMO_TAG_TAZSINK,
    /** a traffic light logic */
    SUMO_TAG_TLLOGIC,
    /** a single phase description */
    SUMO_TAG_PHASE,
    /** a single trip definition (used by router) */
    SUMO_TAG_TRIP,
    /** a flow definition (used by router) */
    SUMO_TAG_FLOW,
    /** trigger: a step description */
    SUMO_TAG_STEP,
    /** an aggreagated-output interval */
    SUMO_TAG_INTERVAL,
    /// The definition of a periodic event
    SUMO_TAG_TIMEDEVENT,
    /// Incoming edge specification (jtrrouter)
    SUMO_TAG_FROMEDGE,
    /// Outgoing edge specification (jtrrouter)
    SUMO_TAG_TOEDGE,
    /// Sink(s) specification
    SUMO_TAG_SINK,
    SUMO_TAG_PARAM,
    SUMO_TAG_WAUT,
    SUMO_TAG_WAUT_SWITCH,
    SUMO_TAG_WAUT_JUNCTION,
    SUMO_TAG_SEGMENT,
    SUMO_TAG_DELETE,
    SUMO_TAG_STOP,
    SUMO_TAG_DEST_PROB_REROUTE,
    SUMO_TAG_CLOSING_REROUTE,
    SUMO_TAG_ROUTE_PROB_REROUTE,
    SUMO_TAG_POLYTYPE,
    SUMO_TAG_CONNECTION,
    SUMO_TAG_PROHIBITION,
    SUMO_TAG_SPLIT,
    SUMO_TAG_NODE,
    SUMO_TAG_TYPE,
    SUMO_TAG_DETECTOR_DEFINITION,
    SUMO_TAG_ROUTE_DISTRIBUTION,
    SUMO_TAG_VTYPE_DISTRIBUTION,
    SUMO_TAG_VAPORIZER,
    SUMO_TAG_ROUNDABOUT,
    SUMO_TAG_JOIN,
    SUMO_TAG_JOINEXCLUDE,
    SUMO_TAG_CROSSING,
    SUMO_TAG_CUSTOMSHAPE,

    SUMO_TAG_WAY,
    SUMO_TAG_ND,
    SUMO_TAG_TAG,
    SUMO_TAG_RELATION,
    SUMO_TAG_MEMBER,

    SUMO_TAG_VIEWSETTINGS,
    SUMO_TAG_VIEWSETTINGS_DECAL,
    SUMO_TAG_VIEWSETTINGS_LIGHT,
    SUMO_TAG_VIEWSETTINGS_SCHEME,
    SUMO_TAG_VIEWSETTINGS_OPENGL,
    SUMO_TAG_VIEWSETTINGS_BACKGROUND,
    SUMO_TAG_VIEWSETTINGS_EDGES,
    SUMO_TAG_VIEWSETTINGS_VEHICLES,
    SUMO_TAG_VIEWSETTINGS_PERSONS,
    SUMO_TAG_VIEWSETTINGS_CONTAINERS,
    SUMO_TAG_VIEWSETTINGS_JUNCTIONS,
    SUMO_TAG_VIEWSETTINGS_ADDITIONALS,
    SUMO_TAG_VIEWSETTINGS_POIS,
    SUMO_TAG_VIEWSETTINGS_POLYS,
    SUMO_TAG_VIEWSETTINGS_LEGEND,
    SUMO_TAG_VIEWSETTINGS_EVENT,
    SUMO_TAG_VIEWSETTINGS_EVENT_JAM_TIME,
    SUMO_TAG_INCLUDE,
    SUMO_TAG_DELAY,
    SUMO_TAG_VIEWPORT,
    SUMO_TAG_SNAPSHOT,
    SUMO_TAG_BREAKPOINTS_FILE,
    SUMO_TAG_LOCATION,
    SUMO_TAG_COLORSCHEME,
    SUMO_TAG_SCALINGSCHEME,
    SUMO_TAG_ENTRY,

    SUMO_TAG_CF_KRAUSS,
    SUMO_TAG_CF_KRAUSS_PLUS_SLOPE,
    SUMO_TAG_CF_KRAUSS_ORIG1,
    SUMO_TAG_CF_SMART_SK,
    SUMO_TAG_CF_DANIEL1,
    SUMO_TAG_CF_IDM,
    SUMO_TAG_CF_IDMM,
    SUMO_TAG_CF_PWAGNER2009,
    SUMO_TAG_CF_BKERNER,
    SUMO_TAG_CF_WIEDEMANN,

    SUMO_TAG_PERSON,
    SUMO_TAG_PERSONTRIP,
    SUMO_TAG_RIDE,
    SUMO_TAG_WALK,

    SUMO_TAG_CONTAINER,
    SUMO_TAG_TRANSPORT,
    SUMO_TAG_TRANSHIP,
    SUMO_TAG_TRAJECTORIES,
    SUMO_TAG_TIMESTEP,
    SUMO_TAG_TIMESLICE,
    SUMO_TAG_ACTORCONFIG,
    SUMO_TAG_MOTIONSTATE,
    SUMO_TAG_OD_PAIR,

    /**
     * ActivityGen Tags
     **/
    AGEN_TAG_GENERAL,
    /* streets object */
    AGEN_TAG_STREET,

    /* workingHours object */
    AGEN_TAG_WORKHOURS,
    AGEN_TAG_OPENING,
    AGEN_TAG_CLOSING,

    /* school object */
    AGEN_TAG_SCHOOLS,
    AGEN_TAG_SCHOOL,

    /* busStation and bus objects */
    AGEN_TAG_BUSSTATION,
    AGEN_TAG_BUSLINE,
    AGEN_TAG_STATIONS,
    AGEN_TAG_REV_STATIONS,
    AGEN_TAG_STATION,
    AGEN_TAG_FREQUENCY,

    /* population and children accompaniment brackets */
    AGEN_TAG_POPULATION,
    AGEN_TAG_BRACKET,
    //AGEN_TAG_CHILD_ACOMP,

    /* city entrances */
    AGEN_TAG_CITYGATES,
    AGEN_TAG_ENTRANCE,

    /* parameters */
    AGEN_TAG_PARAM

};


/**
 * @enum SumoXMLAttr
 * @brief Numbers representing SUMO-XML - attributes
 * @see GenericSAXHandler
 * @see SUMOSAXHandler
 */
enum SumoXMLAttr {
    /** invalid attribute */
    SUMO_ATTR_NOTHING,
    /* common attributes */
    SUMO_ATTR_ID,
    SUMO_ATTR_REFID,
    SUMO_ATTR_NAME,
    SUMO_ATTR_TYPE,
    SUMO_ATTR_PRIORITY,
    SUMO_ATTR_NUMLANES,
    SUMO_ATTR_SPEED,
    SUMO_ATTR_ONEWAY,
    SUMO_ATTR_WIDTH,
    SUMO_ATTR_SIDEWALKWIDTH,
    SUMO_ATTR_BIKELANEWIDTH,
    SUMO_ATTR_REMOVE,
    SUMO_ATTR_LENGTH,
    SUMO_ATTR_X,
    SUMO_ATTR_Y,
    SUMO_ATTR_Z,
    SUMO_ATTR_CENTER_X,
    SUMO_ATTR_CENTER_Y,
    SUMO_ATTR_CENTER_Z,
    /* sumo-junction attributes */
    SUMO_ATTR_KEY,
    SUMO_ATTR_REQUESTSIZE,
    SUMO_ATTR_REQUEST,
    SUMO_ATTR_RESPONSE,
    SUMO_ATTR_PROGRAMID,
    SUMO_ATTR_OFFSET,
    SUMO_ATTR_ENDOFFSET,
    SUMO_ATTR_INCLANES,
    SUMO_ATTR_INTLANES,
    /// the weight of a district's source or sink
    SUMO_ATTR_WEIGHT,
    SUMO_ATTR_NODE,
    SUMO_ATTR_EDGE,
    /// the edges of a route
    SUMO_ATTR_EDGES,
    /* vehicle attributes */
    SUMO_ATTR_DEPART,
    SUMO_ATTR_DEPARTLANE,
    SUMO_ATTR_DEPARTPOS,
    SUMO_ATTR_DEPARTPOS_LAT,
    SUMO_ATTR_DEPARTSPEED,
    SUMO_ATTR_ARRIVALLANE,
    SUMO_ATTR_ARRIVALPOS,
    SUMO_ATTR_ARRIVALSPEED,
    SUMO_ATTR_ROUTE,
    SUMO_ATTR_MAXSPEED,
    SUMO_ATTR_MAXSPEED_LAT,
    SUMO_ATTR_LATALIGNMENT,
    SUMO_ATTR_MINGAP_LAT,
    SUMO_ATTR_ACCEL,
    SUMO_ATTR_DECEL,
    SUMO_ATTR_VCLASS,
    SUMO_ATTR_REPNUMBER,
    SUMO_ATTR_SPEEDFACTOR,
    SUMO_ATTR_SPEEDDEV,
    SUMO_ATTR_LANE_CHANGE_MODEL,
    SUMO_ATTR_CAR_FOLLOW_MODEL,
    SUMO_ATTR_MINGAP,
    SUMO_ATTR_BOARDING_DURATION,
    SUMO_ATTR_LOADING_DURATION,
    /* Charging Station */
    SUMO_ATTR_CHARGINGPOWER,    // charge in W of the Charging Stations
    SUMO_ATTR_EFFICIENCY,       // Eficiency of the charge inCharging Stations
    SUMO_ATTR_CHARGEINTRANSIT,  // Allow/disallow charge in transit in Charging Stations
    SUMO_ATTR_CHARGEDELAY,      // Delay in the charge of charging stations
    /* Car following model attributes */
    SUMO_ATTR_SIGMA,    // used by: Krauss
    SUMO_ATTR_TAU,      // Krauss
    SUMO_ATTR_TMP1,
    SUMO_ATTR_TMP2,
    SUMO_ATTR_TMP3,
    SUMO_ATTR_TMP4,
    SUMO_ATTR_TMP5,
    /* Lane changing model attributes */
    SUMO_ATTR_LCA_STRATEGIC_PARAM,
    SUMO_ATTR_LCA_COOPERATIVE_PARAM,
    SUMO_ATTR_LCA_SPEEDGAIN_PARAM,
    SUMO_ATTR_LCA_KEEPRIGHT_PARAM,
    SUMO_ATTR_LCA_SUBLANE_PARAM,
    SUMO_ATTR_LCA_PUSHY,
    /* route alternatives / distribution attributes */
    SUMO_ATTR_LAST,
    SUMO_ATTR_COST,
    SUMO_ATTR_PROB,
    SUMO_ATTR_PROBS,
    SUMO_ATTR_ROUTES,
    SUMO_ATTR_VTYPES,
    /* trip definition attributes */
    SUMO_ATTR_LANE,
    SUMO_ATTR_LANES,
    SUMO_ATTR_FROM,
    SUMO_ATTR_TO,
    SUMO_ATTR_PERIOD,
    SUMO_ATTR_FROM_TAZ,
    SUMO_ATTR_TO_TAZ,
    SUMO_ATTR_REROUTE,
    SUMO_ATTR_PERSON_CAPACITY,
    SUMO_ATTR_CONTAINER_CAPACITY,
    SUMO_ATTR_PERSON_NUMBER,
    SUMO_ATTR_CONTAINER_NUMBER,
    SUMO_ATTR_MODES,
    /* source definitions */
    SUMO_ATTR_FUNCTION,
    SUMO_ATTR_POSITION,
    SUMO_ATTR_POSITION_LAT,
    SUMO_ATTR_FREQUENCY,
    SUMO_ATTR_STYLE,
    SUMO_ATTR_FILE,
    SUMO_ATTR_JUNCTION,
    SUMO_ATTR_NUMBER,
    SUMO_ATTR_DURATION,
    SUMO_ATTR_UNTIL,
    SUMO_ATTR_ROUTEPROBE,
    /* the edges crossed by a pedestrian crossing */
    SUMO_ATTR_CROSSING_EDGES,
    /** trigger: the time of the step */
    SUMO_ATTR_TIME,
    /** weights: time range begin */
    SUMO_ATTR_BEGIN,
    /** weights: time range end */
    SUMO_ATTR_END,
    /** link,node: the traffic light id responsible for this link */
    SUMO_ATTR_TLID,
    /** node: the type of traffic light */
    SUMO_ATTR_TLTYPE,
    /** link: the index of the link within the traffic light */
    SUMO_ATTR_TLLINKINDEX,
    /** edge: the shape in xml-definition */
    SUMO_ATTR_SHAPE,
    /// The information about how to spread the lanes from the given position
    SUMO_ATTR_SPREADTYPE,
    /// The turning radius at an intersection in m
    SUMO_ATTR_RADIUS,
    /// Whether vehicles must keep the junction clear
    SUMO_ATTR_KEEP_CLEAR,
    /// whether a given shape is user-defined
    SUMO_ATTR_CUSTOMSHAPE,
    /// A color information
    SUMO_ATTR_COLOR,
    /// The abstract direction of a link
    SUMO_ATTR_DIR,
    /// The state of a link
    SUMO_ATTR_STATE,
    /// A layer number
    SUMO_ATTR_LAYER,
    /// Fill the polygon
    SUMO_ATTR_FILL,
    SUMO_ATTR_PREFIX,
    SUMO_ATTR_DISCARD,

    SUMO_ATTR_FROM_LANE,
    SUMO_ATTR_TO_LANE,
    SUMO_ATTR_DEST,
    SUMO_ATTR_SOURCE,
    SUMO_ATTR_VIA,
    /// a list of node ids, used for controlling joining
    SUMO_ATTR_NODES,
    // Attributes for actuated traffic lights:
    /// minimum duration of a phase
    SUMO_ATTR_MINDURATION,
    /// maximum duration of a phase
    SUMO_ATTR_MAXDURATION,
    // Attributes for junction-internal lanes
    /// Information within the junction logic which internal lanes block external
    SUMO_ATTR_FOES,
    // Attributes for detectors
    /// Information whether the detector shall be continued on the folowing lanes
    SUMO_ATTR_CONT,
    SUMO_ATTR_CONTPOS,
    SUMO_ATTR_HALTING_TIME_THRESHOLD,
    SUMO_ATTR_HALTING_SPEED_THRESHOLD,
    SUMO_ATTR_JAM_DIST_THRESHOLD,

    SUMO_ATTR_WAUT_ID,
    SUMO_ATTR_JUNCTION_ID,
    SUMO_ATTR_PROCEDURE,
    SUMO_ATTR_SYNCHRON,
    SUMO_ATTR_REF_TIME,
    SUMO_ATTR_START_PROG,

    SUMO_ATTR_OFF,
    SUMO_ATTR_FRIENDLY_POS,
    SUMO_ATTR_SPLIT_VTYPE,
    SUMO_ATTR_UNCONTROLLED,
    SUMO_ATTR_PASS,
    SUMO_ATTR_BUS_STOP,
    SUMO_ATTR_CONTAINER_STOP,
    SUMO_ATTR_CHARGING_STATION,
    SUMO_ATTR_LINE,
    SUMO_ATTR_LINES,
    SUMO_ATTR_VALUE,
    SUMO_ATTR_PROHIBITOR,
    SUMO_ATTR_PROHIBITED,
    SUMO_ATTR_ALLOW,
    SUMO_ATTR_DISALLOW,
    SUMO_ATTR_PREFER,
    SUMO_ATTR_CONTROLLED_INNER,
    SUMO_ATTR_VEHSPERHOUR,
    SUMO_ATTR_OUTPUT,
    SUMO_ATTR_HEIGHT,
    SUMO_ATTR_GUISHAPE,
    SUMO_ATTR_OSGFILE,
    SUMO_ATTR_IMGFILE,
    SUMO_ATTR_ANGLE,
    SUMO_ATTR_EMISSIONCLASS,
    SUMO_ATTR_IMPATIENCE,
    SUMO_ATTR_STARTPOS,
    SUMO_ATTR_ENDPOS,
    SUMO_ATTR_TRIGGERED,
    SUMO_ATTR_CONTAINER_TRIGGERED,
    SUMO_ATTR_PARKING,
    SUMO_ATTR_EXPECTED,
    SUMO_ATTR_EXPECTED_CONTAINERS,
    SUMO_ATTR_INDEX,

    SUMO_ATTR_ENTERING,
    SUMO_ATTR_EXCLUDE_EMPTY,
    SUMO_ATTR_WITH_INTERNAL,
    SUMO_ATTR_TRACK_VEHICLES,
    SUMO_ATTR_MAX_TRAVELTIME,
    SUMO_ATTR_MIN_SAMPLES,

    SUMO_ATTR_LON,
    SUMO_ATTR_LAT,
    SUMO_ATTR_GEO,
    SUMO_ATTR_K,
    SUMO_ATTR_V,
    SUMO_ATTR_REF,
    SUMO_ATTR_HREF,
    SUMO_ATTR_ZOOM,
    SUMO_ATTR_INTERPOLATED,
    SUMO_ATTR_THRESHOLD,

    SUMO_ATTR_NET_OFFSET,
    SUMO_ATTR_CONV_BOUNDARY,
    SUMO_ATTR_ORIG_BOUNDARY,
    SUMO_ATTR_ORIG_PROJ,

    SUMO_ATTR_CF_PWAGNER2009_TAULAST,
    SUMO_ATTR_CF_PWAGNER2009_APPROB,
    SUMO_ATTR_CF_IDM_DELTA,
    SUMO_ATTR_CF_IDM_STEPPING,
    SUMO_ATTR_CF_IDMM_ADAPT_FACTOR,
    SUMO_ATTR_CF_IDMM_ADAPT_TIME,
    SUMO_ATTR_CF_KERNER_PHI,
    SUMO_ATTR_CF_WIEDEMANN_SECURITY,
    SUMO_ATTR_CF_WIEDEMANN_ESTIMATION,

    SUMO_ATTR_GENERATE_WALKS,
    SUMO_ATTR_ACTTYPE,
    SUMO_ATTR_SLOPE,
    SUMO_ATTR_VERSION,
    SUMO_ATTR_CORNERDETAIL,
    SUMO_ATTR_LINKDETAIL,
    SUMO_ATTR_LEFTHAND,
    SUMO_ATTR_COMMAND,

    SUMO_ATTR_ACTORCONFIG,
    SUMO_ATTR_VEHICLE,
    SUMO_ATTR_STARTTIME,
    SUMO_ATTR_VEHICLECLASS,
    SUMO_ATTR_FUEL,
    SUMO_ATTR_ACCELERATION,
    SUMO_ATTR_AMOUNT,
    SUMO_ATTR_ORIGIN,
    SUMO_ATTR_DESTINATION,


    /**
     * ActivityGen Tags
     **/
    /* general object */
    AGEN_ATTR_INHABITANTS,
    AGEN_ATTR_HOUSEHOLDS,
    AGEN_ATTR_CHILDREN,
    AGEN_ATTR_RETIREMENT,
    AGEN_ATTR_CARS,
    AGEN_ATTR_UNEMPLOYEMENT,
    AGEN_ATTR_MAX_FOOT_DIST,
    AGEN_ATTR_IN_TRAFFIC,
    AGEN_ATTR_OUT_TRAFFIC,

    /* streets object */
    //SUMO_ATTR_EDGE already defined
    AGEN_ATTR_POPULATION,
    AGEN_ATTR_OUT_WORKPOSITION,

    /* workHours object */
    AGEN_ATTR_HOUR,
    AGEN_ATTR_PROP,

    /* school object */
    //SUMO_ATTR_EDGE, SUMO_ATTR_POSITION, SUMO_ATTR_TYPE already defined
    AGEN_ATTR_CAPACITY,
    AGEN_ATTR_OPENING,
    AGEN_ATTR_CLOSING,

    /* busStation and Bus objects */
    // ID, EDGE, POSITION, REFID, BEGIN and END are already defined
    AGEN_ATTR_MAX_TRIP_DURATION,
    //AGEN_ATTR_ORDER,
    AGEN_ATTR_RATE,

    /* population and children accompaniment brackets */
    AGEN_ATTR_BEGINAGE,
    AGEN_ATTR_ENDAGE,
    AGEN_ATTR_PEOPLENBR,

    /* parameters */
    AGEN_ATTR_CARPREF,
    AGEN_ATTR_CITYSPEED,
    AGEN_ATTR_FREETIMERATE,
    AGEN_ATTR_UNI_RAND_TRAFFIC,
    AGEN_ATTR_DEP_VARIATION,

    /* city gates */
    AGEN_ATTR_INCOMING,
    AGEN_ATTR_OUTGOING,

    //@name Netedit Attributes (used as virtual property holders, must be in SumoXMLAttr)
    //
    //@{
    /* whether a feature has been loaded,guessed,modified or approved */
    GNE_ATTR_MODIFICATION_STATUS,
    /* first coordinate of edge shape */
    GNE_ATTR_SHAPE_START,
    /* last coordinate of edge shape */
    GNE_ATTR_SHAPE_END,
    /* block movement of a graphic element*/
    GNE_ATTR_BLOCK_MOVEMENT,
    /* parent of an additional element*/
    GNE_ATTR_PARENT,
    //@}

    SUMO_ATTR_TARGETLANE,
    SUMO_ATTR_CROSSING
};

/*
 * definitions of special SumoXML-attribute values.
 * Since these enums shall be used in switch statements we keep them separated
 * @{
 */

/**
 * @enum SumoXMLNodeType
 * @brief Numbers representing special SUMO-XML-attribute values
 * for representing node- (junction-) types used in netbuild/netimport and netload
 */
enum SumoXMLNodeType {
    NODETYPE_UNKNOWN, // terminator
    NODETYPE_TRAFFIC_LIGHT,
    NODETYPE_TRAFFIC_LIGHT_NOJUNCTION, // junction controlled only by traffic light but without other prohibitions,
    NODETYPE_TRAFFIC_LIGHT_RIGHT_ON_RED,
    NODETYPE_RAIL_SIGNAL,
    NODETYPE_RAIL_CROSSING,
    NODETYPE_PRIORITY,
    NODETYPE_PRIORITY_STOP, // like priority but all minor links have stop signs
    NODETYPE_RIGHT_BEFORE_LEFT,
    NODETYPE_ALLWAY_STOP,
    NODETYPE_ZIPPER,
    NODETYPE_DISTRICT,
    NODETYPE_NOJUNCTION,
    NODETYPE_INTERNAL,
    NODETYPE_DEAD_END,
    NODETYPE_DEAD_END_DEPRECATED
};


/**
 * @enum SumoXMLEdgeFunc
 * @brief Numbers representing special SUMO-XML-attribute values
 * for representing edge functions used in netbuild/netimport and netload
 */
enum SumoXMLEdgeFunc {
    EDGEFUNC_NORMAL,
    EDGEFUNC_CONNECTOR,
    EDGEFUNC_SINK,
    EDGEFUNC_SOURCE,
    EDGEFUNC_CROSSING,
    EDGEFUNC_WALKINGAREA,
    EDGEFUNC_INTERNAL
};


/**
 * @enum LaneSpreadFunction
 * @brief Numbers representing special SUMO-XML-attribute values
 * Information how the edge's lateral offset shall be computed
 * In dependence to this value, lanes will be spread to the right side or
 * to both sides from the given edge geometry (Also used when node
 * positions are used as edge geometry).
 */
enum LaneSpreadFunction {
    LANESPREAD_RIGHT,
    LANESPREAD_CENTER
};


/**
 * @enum LinkState
 * @brief The right-of-way state of a link between two lanes
 * used when constructing a NBTrafficLightLogic, in MSLink and GNEInternalLane
 *
 * This enumerations holds the possible right-of-way rules a link
 *  may have. Beyond the righ-of-way rules, this enumeration also
 *  holds the possible traffic light states.
 *
 *  enum values are assigned so that chars can be cast back to linkstates
 *  @todo fix redundancy
 */
enum LinkState {
    /// @brief The link has green light, may pass
    LINKSTATE_TL_GREEN_MAJOR = 'G',
    /// @brief The link has green light, has to brake
    LINKSTATE_TL_GREEN_MINOR = 'g',
    /// @brief The link has red light (must brake)
    LINKSTATE_TL_RED = 'r',
    /// @brief The link has red light (must brake) but indicates upcoming green
    LINKSTATE_TL_REDYELLOW = 'u',
    /// @brief The link has yellow light, may pass
    LINKSTATE_TL_YELLOW_MAJOR = 'Y',
    /// @brief The link has yellow light, has to brake anyway
    LINKSTATE_TL_YELLOW_MINOR = 'y',
    /// @brief The link is controlled by a tls which is off and blinks, has to brake
    LINKSTATE_TL_OFF_BLINKING = 'o',
    /// @brief The link is controlled by a tls which is off, not blinking, may pass
    LINKSTATE_TL_OFF_NOSIGNAL = 'O',
    /// @brief This is an uncontrolled, major link, may pass
    LINKSTATE_MAJOR = 'M',
    /// @brief This is an uncontrolled, minor link, has to brake
    LINKSTATE_MINOR = 'm',
    /// @brief This is an uncontrolled, right-before-left link
    LINKSTATE_EQUAL = '=',
    /// @brief This is an uncontrolled, minor link, has to stop
    LINKSTATE_STOP = 's',
    /// @brief This is an uncontrolled, all-way stop link.
    LINKSTATE_ALLWAY_STOP = 'w',
    /// @brief This is an uncontrolled, zipper-merge link
    LINKSTATE_ZIPPER = 'Z',
    /// @brief This is a dead end link
    LINKSTATE_DEADEND = '-'
};


/**
 * @enum LinkDirection
 * @brief The different directions a link between two lanes may take (or a
 * stream between two edges).
 * used in netbuild (formerly NBMMLDirection) and MSLink
 */
enum LinkDirection {
    /// The link is a straight direction
    LINKDIR_STRAIGHT = 0,
    /// The link is a 180 degree turn
    LINKDIR_TURN,
    /// The link is a 180 degree turn (left-hand network)
    LINKDIR_TURN_LEFTHAND,
    /// The link is a (hard) left direction
    LINKDIR_LEFT,
    /// The link is a (hard) right direction
    LINKDIR_RIGHT,
    /// The link is a partial left direction
    LINKDIR_PARTLEFT,
    /// The link is a partial right direction
    LINKDIR_PARTRIGHT,
    /// The link has no direction (is a dead end link)
    LINKDIR_NODIR
};


/**
 * @enum TrafficLightType
 */
enum TrafficLightType {
    TLTYPE_STATIC,
    TLTYPE_RAIL,
    TLTYPE_ACTUATED,
    TLTYPE_SOTL_PHASE,
    TLTYPE_SOTL_PLATOON,
    TLTYPE_SOTL_REQUEST,
    TLTYPE_SOTL_WAVE,
    TLTYPE_SOTL_MARCHING,
    TLTYPE_SWARM_BASED,
    TLTYPE_HILVL_DETERMINISTIC,
    TLTYPE_INVALID //< must be the last one
};

/** @enum LaneChangeAction
 * @brief The state of a vehicle's lane-change behavior
 */
enum LaneChangeAction {
    /// @name currently wanted lane-change action
    /// @{

    /// @brief No action desired
    LCA_NONE = 0,
    /// @brief Needs to stay on the current lane
    LCA_STAY = 1 << 0,
    /// @brief Wants go to the left
    LCA_LEFT = 1 << 1,
    /// @brief Wants go to the right
    LCA_RIGHT = 1 << 2,

    /// @brief The action is needed to follow the route (navigational lc)
    LCA_STRATEGIC = 1 << 3,
    /// @brief The action is done to help someone else
    LCA_COOPERATIVE = 1 << 4,
    /// @brief The action is due to the wish to be faster (tactical lc)
    LCA_SPEEDGAIN = 1 << 5,
    /// @brief The action is due to the default of keeping right "Rechtsfahrgebot"
    LCA_KEEPRIGHT = 1 << 6,
    /// @brief The action is due to a TraCI request
    LCA_TRACI = 1 << 7,

    /// @brief The action is urgent (to be defined by lc-model)
    LCA_URGENT = 1 << 8,

    /// @}

    /// @name External state
    /// @{

    /// @brief The vehicle is blocked by left leader
    LCA_BLOCKED_BY_LEFT_LEADER = 1 << 9,
    /// @brief The vehicle is blocked by left follower
    LCA_BLOCKED_BY_LEFT_FOLLOWER = 1 << 10,

    /// @brief The vehicle is blocked by right leader
    LCA_BLOCKED_BY_RIGHT_LEADER = 1 << 11,
    /// @brief The vehicle is blocked by right follower
    LCA_BLOCKED_BY_RIGHT_FOLLOWER = 1 << 12,

    // The vehicle is blocked being overlapping
    LCA_OVERLAPPING =  1 << 13,

    // The vehicle does not have enough space to complete a continuous lane
    // change before the next turning movement
    LCA_INSUFFICIENT_SPACE =  1 << 14,

    // used by the sublane model
    LCA_SUBLANE = 1 << 15,

    LCA_WANTS_LANECHANGE = LCA_LEFT | LCA_RIGHT,
    LCA_WANTS_LANECHANGE_OR_STAY = LCA_WANTS_LANECHANGE | LCA_STAY,

    LCA_BLOCKED_LEFT = LCA_BLOCKED_BY_LEFT_LEADER | LCA_BLOCKED_BY_LEFT_FOLLOWER,
    LCA_BLOCKED_RIGHT = LCA_BLOCKED_BY_RIGHT_LEADER | LCA_BLOCKED_BY_RIGHT_FOLLOWER,
    LCA_BLOCKED_BY_LEADER = LCA_BLOCKED_BY_LEFT_LEADER | LCA_BLOCKED_BY_RIGHT_LEADER,
    LCA_BLOCKED_BY_FOLLOWER = LCA_BLOCKED_BY_LEFT_FOLLOWER | LCA_BLOCKED_BY_RIGHT_FOLLOWER,
    LCA_BLOCKED = LCA_BLOCKED_LEFT | LCA_BLOCKED_RIGHT | LCA_INSUFFICIENT_SPACE,

    LCA_CHANGE_REASONS = (LCA_STRATEGIC | LCA_COOPERATIVE | LCA_SPEEDGAIN | LCA_KEEPRIGHT | LCA_SUBLANE)

                         // LCA_BLOCKED_BY_CURRENT_LEADER = 1 << 28
                         // LCA_BLOCKED_BY_CURRENT_FOLLOWER = 1 << 29
                         /// @}

};





/**
 * @enum LaneChangeModel
 */
enum LaneChangeModel {
    LCM_DK2008,
    LCM_LC2013,
    LCM_JE2013,
    LCM_SL2015,
    LCM_DEFAULT
};


//@}
//
/**
 * @enum LateralAlignment
 * @brief Numbers representing special SUMO-XML-attribute values
 * Information how vehicles align themselves within their lane by default
 */
enum LateralAlignment {
    LATALIGN_RIGHT,      // drive on the right side
    LATALIGN_CENTER,     // drive in the middle
    LATALIGN_ARBITRARY,  // maintain the current alignment
    LATALIGN_NICE,       // align with the closest sublane border
    LATALIGN_COMPACT,    // align with the rightmost sublane that allows keeping the current speed
    LATALIGN_LEFT        // drive on the left side
};

/**
 * @class SUMOXMLDefinitions
 * @brief class for maintaining associations between enums and xml-strings
 */

class SUMOXMLDefinitions {

public:
    /// The names of SUMO-XML elements (for passing to GenericSAXHandler)
    static StringBijection<int>::Entry tags[];

    /// The names of SUMO-XML attributes (for passing to GenericSAXHandler)
    static StringBijection<int>::Entry attrs[];

    /// The names of SUMO-XML elements for use in netbuild
    static StringBijection<int> Tags;

    /// The names of SUMO-XML attributes for use in netbuild
    static StringBijection<int> Attrs;

    /// @name Special values of SUMO-XML attributes
    //@{
    static StringBijection<SumoXMLNodeType> NodeTypes;

    static StringBijection<SumoXMLEdgeFunc> EdgeFunctions;

    static StringBijection<LaneSpreadFunction> LaneSpreadFunctions;

    static StringBijection<LinkState> LinkStates;

    static StringBijection<LinkDirection> LinkDirections;

    static StringBijection<TrafficLightType> TrafficLightTypes;

    static StringBijection<LaneChangeModel> LaneChangeModels;

    static StringBijection<SumoXMLTag> CarFollowModels;

    static StringBijection<LateralAlignment> LateralAlignments;

    static StringBijection<LaneChangeAction> LaneChangeActions;
    //@}

    /// @name Helper functions for ID-string manipulations
    //@{

    /// @brief return the junction id when given an edge of type internal, crossing or WalkingArea
    static std::string getJunctionIDFromInternalEdge(const std::string internalEdge);

    /// @brief return edge id when given the lane ID
    static std::string getEdgeIDFromLane(const std::string laneID);

    //@}

private:

    static StringBijection<SumoXMLNodeType>::Entry sumoNodeTypeValues[];

    static StringBijection<SumoXMLEdgeFunc>::Entry sumoEdgeFuncValues[];

    static StringBijection<LaneSpreadFunction>::Entry laneSpreadFunctionValues[];

    static StringBijection<LinkState>::Entry linkStateValues[];

    static StringBijection<LinkDirection>::Entry linkDirectionValues[];

    static StringBijection<TrafficLightType>::Entry trafficLightTypesVales[];

    static StringBijection<LaneChangeModel>::Entry laneChangeModelValues[];

    static StringBijection<SumoXMLTag>::Entry carFollowModelValues[];

    static StringBijection<LateralAlignment>::Entry lateralAlignmentValues[];

    static StringBijection<LaneChangeAction>::Entry laneChangeActionValues[];

};

#endif

/****************************************************************************/

