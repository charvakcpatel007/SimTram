#!/usr/bin/env python
"""
@file    route_1htoDay.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    11.09.2009
@version $Id: route_1htoDay.py 20433 2016-04-13 08:00:14Z behrisch $

Uses "route_departOffset.py" for building 24 route files which describe
 a whole day assuming the given route files describes an hour.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import route_departOffset

if len(sys.argv) < 2:
    print("Usage: route_1htoDay.py <INPUT_FILE>")
    sys.exit()
for i in range(0, 24):
    out = sys.argv[1]
    out = out[:out.find(".")] + "_" + str(i) + out[out.find("."):]
    print("Building routes for hour " + str(i) + " into '" + out + "'...")
    route_departOffset.main(sys.argv[1], out, i * 3600)
