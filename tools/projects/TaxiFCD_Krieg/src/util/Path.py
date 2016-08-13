# -*- coding: Latin-1 -*-
"""
@file    Path.py
@author  Sascha Krieg
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2008-04-17
@version $Id: Path.py 20433 2016-04-13 08:00:14Z behrisch $

Contains paths which are needed frequently

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import

import os.path


def newPath(path, *paths):
    """Little helper to join several path elements and to convert windows backslash into a slash."""
    p = os.path.join(path, *paths)
    p = p.replace("\\", "/")
    return p
##Paths##


# main="F:/DLR/Projekte/Diplom/Daten/"
# main="D:/Krieg/Projekte/Diplom/Daten/"
main = "../../../../Projekte/Diplom/Daten"

fcd = newPath(main, "originalFCD/Proz-fcd_nuremberg_2007-07-18.dat")
simFcd = newPath(
    main, "simProzessiertFCD/nuremberg_sim_on_edges3Mit_t2Complete.dat")
vls = newPath(main, "originalFCD/Proz-fcd_nuremberg_VLS_2007-07-18.dat")
rawFcd = newPath(main, "originalFCD/Roh-nuremberg_2007-07-18.dat")
rawFcdVehIdList = newPath(main, "simRawFCD/rawFcdVehIdList.pickle")
edgeLengthDict = newPath(main, "taxiRouten/edgeLengthDict.pickle")
simRawFcd = newPath(
    main, "simRawFCD/simulatedRawFCD3Mit_t2Complete.out.dat")  # used input
taxiRoutes = newPath(main, "taxiRouten/taxiRoutes.rou.xml")
taxiRoutesComplete = newPath(main, "taxiRouten/t2Complete.rou.xml")
taxiRoutesDiffDepart = newPath(
    main, "taxiRouten/t2MitVerschiedenenLosfahrzeiten/t2Complete_")
net = newPath(
    main, "sumoNetzFilesNurnbergIIProjektion/nuernberg_vls_new.net.xml")
vtypeprobe = newPath(main, "simRawFCD/vtypeprobe3Mit_t2Complete.out.xml")


analysis = newPath(main, "auswertung/taxiAnalysisInformation.xml")
analysisWEE = newPath(
    main, "auswertung/taxiAnalysisInformationWithoutEmptyEdges.xml")
simulatedRawFCD = newPath(
    main, "simRawFCD/simulatedRawFCD.out.dat")  # generated output


taxisPerEdge = newPath(main, "mpl_dump_onNet__Files/taxisPerEdge.out.xml")
fcdVsCompleteRoute = newPath(
    main, "mpl_dump_onNet__Files/FCD_vs_completeRoute")
taxiVsFCDSpeed = newPath(
    main, "auswertung/geschwindigkeitsvergleich/taxiVsFCD.csv")
taxiVsFCDSpeedSelLanes = newPath(
    main, "auswertung/geschwindigkeitsvergleich/taxiVsFCD_SelectedLanes.txt")
normalTrafficRoutes = newPath(
    main, "sumoNetzFilesNurnbergIV/joined_a5f.rou.xml")
drivenEdges = newPath(main, "sumoNetzFilesNurnbergIV/drivenEdgesSet.pickle")
vOverTimeDir = newPath(
    main, "auswertung/Geschwindigkeitsganglinie/ZeitproKante/")
vOverRouteDir = newPath(main, "auswertung/GeschwindigkeitUeberRoute/")

# all paths for paramEffects
FQedgeDump = newPath(main, "fcdQualitaet/edgedumpFcdQuality_900_6Uhr.xml")
FQedgeDumpPickle = newPath(main, "fcdQualitaet/edgedumpFcdPickleDict.pickle")
FQvtype = newPath(main, "fcdQualitaet/vtypeprobeFcdQuality_1s_6Uhr.out.xml")
FQvtypePickle = newPath(main, "fcdQualitaet/vtypeprobePickleDict.pickle")
FQvehPickle = newPath(main, "fcdQualitaet/vehiclePickleList.pickle")
FQoutput = newPath(main, "fcdQualitaet/output/plotData/paramEffects_")
FQrawFCD = newPath(main, "fcdQualitaet/output/rawData/rawData_")
FQprocessedFCD = newPath(
    main, "fcdQualitaet/processedFCD/linkspeeds.NUREMBERG_FAKE_03.txt")
