#!/usr/bin/env python
"""
@file    xmlconnections_mapEdges.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2009-08-01
@version $Id: xmlconnections_mapEdges.py 20433 2016-04-13 08:00:14Z behrisch $

Reads edge id replacements from "edgemap.txt"; the format of this file is
 <OLD_EDGE_ID>-><NEW_EDGE_ID>
Reads the given connections file <CONNECTIONS> and replaces old edge names by new.
The result is written to <CONNECTIONS>.mod.xml

Call: xmlconnections_mapEdges.py <CONNECTIONS>

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

if len(sys.argv) < 2:
    print("Usage: " + sys.argv[0] + " <CONNECTIONS>")
    sys.exit()

# read map
mmap = {}
fdi = open("edgemap.txt")
for line in fdi:
    if line.find("->") < 0:
        continue
    (orig, dest) = line.strip().split("->")
    dest = dest.split(",")
    mmap[orig] = dest
fdi.close()

fdi = open(sys.argv[1])
fdo = open(sys.argv[1] + ".mod.xml", "w")
for line in fdi:
    for orig in mmap:
        line = line.replace(
            'from="' + orig + '"', 'from="' + mmap[orig][-1] + '"')
        line = line.replace('to="' + orig + '"', 'to="' + mmap[orig][0] + '"')
    fdo.write(line)
fdi.close()
fdo.close()
