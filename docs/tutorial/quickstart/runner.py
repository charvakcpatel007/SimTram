#!/usr/bin/env python
"""
@file    runner.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2011-09-25
@version $Id: runner.py 20433 2016-04-13 08:00:14Z behrisch $

This script is a test runner for the quickstart tutorial.

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
import subprocess
import sys
import time
import shutil
sys.path.append(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools"))
from sumolib import checkBinary


netconvertBinary = checkBinary('netconvert')
sumoBinary = checkBinary('sumo')
# build/check network
retcode = subprocess.call(
    [netconvertBinary, "-c", "data/quickstart.netccfg"], stdout=sys.stdout, stderr=sys.stderr)
try:
    shutil.copy("data/quickstart.net.xml", "net.net.xml")
except:
    print("Missing 'quickstart.net.xml'")
print(">> Netbuilding closed with status %s" % retcode)
sys.stdout.flush()
# run simulation
retcode = subprocess.call(
    [sumoBinary, "-c", "data/quickstart.sumocfg", "--no-step-log"], stdout=sys.stdout, stderr=sys.stderr)
print(">> Simulation closed with status %s" % retcode)
sys.stdout.flush()
