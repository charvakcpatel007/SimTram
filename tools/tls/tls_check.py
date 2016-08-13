#!/usr/bin/env python
"""
@file    tls_check.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2009-08-01
@version $Id: tls_check.py 20482 2016-04-18 20:49:42Z behrisch $

Verifies the traffic lights in the given network.
Currently verified:
- phase length matches controlled link number

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sumolib.net


if len(sys.argv) < 2:
    print("Call: tls_check.py <NET>", file=sys.stderr)
    sys.exit()

net1 = sumolib.net.readNet(sys.argv[1], withPrograms=True)

for tlsID in net1._id2tls:
    print("Checking tls '%s'" % tlsID)
    tls = net1._id2tls[tlsID]
    noConnections = tls._maxConnectionNo + 1
    for prog in tls._programs:
        print("   Checking program '%s'" % prog)
        prog = tls._programs[prog]
        for i, phase in enumerate(prog._phases):
            if len(phase[0]) != noConnections:
                print("      Error: phase %s describes %s signals instead of %s." % (
                    i, len(phase[0]), noConnections))
