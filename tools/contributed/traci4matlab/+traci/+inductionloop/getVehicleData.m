function vehicleData = getVehicleData(loopID)
%getVehicleData Get several data about passed vehicles.
%   vehicleData = getVehicleData(LOOPID) Returns a cell structure 
%   containing several information about vehicles which passed the detector.

%   Copyright 2015 Universidad Nacional de Colombia,
%   Politecnico Jaime Isaza Cadavid.
%   Authors: Andres Acosta, Jairo Espinosa, Jorge Espinosa.
%   $Id: getVehicleData.m 20 2015-03-02 16:52:32Z afacostag $

import traci.constants
vehicleData = traci.inductionloop.getUniversal(constants.LAST_STEP_VEHICLE_DATA, loopID);