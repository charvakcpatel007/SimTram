#!/usr/bin/python
"""
@file    batch0103to0110.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2007
@version $Id: batch0103to0110.py 20433 2016-04-13 08:00:14Z behrisch $

Applies the transformation on all nets in the given folder or 
 - if no folder is given - in the base folder (../..).

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

import os
import os.path
import sys

r = "../../"
if len(sys.argv) > 1:
    r = sys.argv[1]
srcRoot = os.path.join(os.path.dirname(sys.argv[0]), r)
for root, dirs, files in os.walk(srcRoot):
    for name in files:
        if name.endswith(".net.xml") or name == "net.netconvert" or name == "net.netgen":
            p = os.path.join(root, name)
            print("Patching " + p + "...")
            os.system("0103to0110.py " + p)
            os.remove(p)
            os.rename(p + ".chg", p)
        for ignoreDir in ['.svn', 'foreign']:
            if ignoreDir in dirs:
                dirs.remove(ignoreDir)
