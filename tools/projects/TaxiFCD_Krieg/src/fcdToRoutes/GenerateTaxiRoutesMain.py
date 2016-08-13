#!/usr/bin/env python
# -*- coding: Latin-1 -*-
"""
@file    GenerateTaxiRoutesMain.py
@author  Sascha Krieg
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2008-04-17
@version $Id: GenerateTaxiRoutesMain.py 20433 2016-04-13 08:00:14Z behrisch $

Main of GenerateTaxiRoutes.

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

from .GenerateTaxiRoutes import *


def main():
    print("start program")
    readFCD()
    writeRoutes()
    print("end")

# start the program
main()
