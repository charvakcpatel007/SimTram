function speedDeviation = getSpeedDeviation(vehID)
%getSpeedDeviation Returns the maximum speed deviation of the vehicle type.
%   speedDeviation = getSpeedDeviation(VEHID) Returns the maximum speed 
%   deviation of the vehicle type.

%   Copyright 2015 Universidad Nacional de Colombia,
%   Politecnico Jaime Isaza Cadavid.
%   Authors: Andres Acosta, Jairo Espinosa, Jorge Espinosa.
%   $Id: getSpeedDeviation.m 22 2015-04-13 14:20:57Z afacostag $

import traci.constants
speedDeviation = traci.vehicle.getUniversal(constants.VAR_SPEED_DEVIATION, vehID);