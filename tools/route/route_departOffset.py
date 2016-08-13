#!/usr/bin/env python
"""
@file    route_departOffset.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    11.09.2009
@version $Id: route_departOffset.py 20482 2016-04-18 20:49:42Z behrisch $

Applies a given offset to the given route's departure time

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
import sys
import optparse
import array
from xml.sax import make_parser, handler


class RouteReader(handler.ContentHandler):

    def __init__(self, offset, out):
        self._offset = offset
        self._out = out

    def startElement(self, name, attrs):
        self._out.write('<' + name)
        for a in attrs.keys():
            val = attrs[a]
            if a == "depart":
                val = str(int(val) + self._offset)
            if a == "id":
                val = val + "_" + str(self._offset)
            self._out.write(' ' + a + '="' + val + '"')
        self._out.write('>')

    def endElement(self, name):
        self._out.write('</' + name + '>')

    def characters(self, content):
        self._out.write(content)


def main(infile, outfile, offset):
    out = open(outfile, "w")
    parser = make_parser()
    parser.setContentHandler(RouteReader(offset, out))
    parser.parse(infile)

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print(
            "Usage: route_departOffset.py <INPUT_FILE> <OUTPUT_FILE> <OFFSET>")
        sys.exit()
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]))
