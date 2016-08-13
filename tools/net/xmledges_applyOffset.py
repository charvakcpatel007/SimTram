#!/usr/bin/env python
"""
@file    xmledges_applyOffset.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2009-08-01
@version $Id: xmledges_applyOffset.py 20433 2016-04-13 08:00:14Z behrisch $

Applies a given offset to edges given in an xml-edge-file.
The results are written into <XMLEDGES>.mod.xml.
Call: xmledges_applyOffset.py <XMLEDGES> <X-OFFSET> <Y-OFFSET>

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
import string
import sys
from xml.sax import saxutils, make_parser, handler


class XMLNodesReader(handler.ContentHandler):

    def __init__(self, outFileName, xoff, yoff):
        self._out = open(outFileName, 'w')
        self._xoff = xoff
        self._yoff = yoff

    def endDocument(self):
        self._out.close()

    def startElement(self, name, attrs):
        self._out.write('<' + name)
        for (key, value) in attrs.items():
            if key == "shape":
                shape = value.split(" ")
                nshape = []
                for s in shape:
                    (x, y) = s.split(",")
                    nshape.append(
                        str(float(x) + self._xoff) + "," + str(float(y) + self._yoff))
                self._out.write(' %s="%s"' %
                                (key, saxutils.escape(" ".join(nshape))))
            else:
                self._out.write(' %s="%s"' % (key, saxutils.escape(value)))
        self._out.write('>')

    def endElement(self, name):
        self._out.write('</' + name + '>')

    def characters(self, content):
        self._out.write(saxutils.escape(content))

    def ignorableWhitespace(self, content):
        self._out.write(content)

    def processingInstruction(self, target, data):
        self._out.write('<?%s %s?>' % (target, data))


if len(sys.argv) < 4:
    print("Usage: " + sys.argv[0] + " <XMLEDGES> <X-OFFSET> <Y-OFFSET>")
    sys.exit()
parser = make_parser()
reader = XMLNodesReader(
    sys.argv[1] + ".mod.xml", float(sys.argv[2]), float(sys.argv[3]))
parser.setContentHandler(reader)
parser.parse(sys.argv[1])
