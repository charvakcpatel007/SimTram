#!/usr/bin/env python
"""
@file    countConnectionsInDistricts.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2007-07-26
@version $Id: countConnectionsInDistricts.py 20433 2016-04-13 08:00:14Z behrisch $

<documentation missing>

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2007-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function
from xml.sax import saxutils, make_parser, handler
from optparse import OptionParser
import math


# written into the net. All members are "private".
class NetDistrictConnectionCountingHandler(handler.ContentHandler):

    def __init__(self):
        self._currentID = ""
        self._districtSinkNo = {}
        self._districtSourceNo = {}

    def startElement(self, name, attrs):
        if name == 'taz':
            self._currentID = attrs['id']
        elif name == 'tazSink':
            if self._currentID in self._districtSinkNo:
                self._districtSinkNo[self._currentID] = self._districtSinkNo[
                    self._currentID] + 1
            else:
                self._districtSinkNo[self._currentID] = 1
        elif name == 'tazSource':
            if self._currentID in self._districtSinkNo:
                self._districtSourceNo[self._currentID] = self._districtSourceNo[
                    self._currentID] + 1
            else:
                self._districtSourceNo[self._currentID] = 1

    def writeResults(self, output):
        fd = open(output, "w")
        for district in self._districtSourceNo:
            fd.write(district + ";" + str(self._districtSourceNo[district]) + ";" + str(
                self._districtSinkNo[district]) + "\n")
        fd.close()


optParser = OptionParser()
optParser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                     default=False, help="tell me what you are doing")
optParser.add_option("-n", "--net-file", dest="netfile",
                     help="read SUMO network(s) from FILE(s) (mandatory)", metavar="FILE")
optParser.add_option("-o", "--output", dest="output",
                     help="read SUMO network(s) from FILE(s) (mandatory)", metavar="FILE")
(options, args) = optParser.parse_args()

parser = make_parser()
reader = NetDistrictConnectionCountingHandler()
parser.setContentHandler(reader)
if options.verbose:
    print("Reading net '" + options.netfile + "'")
parser.parse(options.netfile)
reader.writeResults(options.output)
