#!/usr/bin/env python
"""
@file    plot_net_trafficLights.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2013-10-28
@version $Id: plot_net_trafficLights.py 20433 2016-04-13 08:00:14Z behrisch $

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
import random
from xml.sax.handler import ContentHandler

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import sumolib
from sumolib.visualization import helpers

import matplotlib.pyplot as plt


def main(args=None):
    """The main function; parses options and plots"""
    # ---------- build and read options ----------
    from optparse import OptionParser
    optParser = OptionParser()
    optParser.add_option("-n", "--net", dest="net", metavar="FILE",
                         help="Defines the network to read")
    optParser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                         default=False, help="If set, the script says what it's doing")
    optParser.add_option("-w", "--width", dest="width",
                         type="float", default=20, help="Defines the width of the dots")
    optParser.add_option("-c", "--color", dest="color",
                         default='r', help="Defines the dot color")
    optParser.add_option("--edge-width", dest="defaultWidth",
                         type="float", default=1, help="Defines the edge width")
    optParser.add_option("--edge-color", dest="defaultColor",
                         default='k', help="Defines the edge color")
    # standard plot options
    helpers.addInteractionOptions(optParser)
    helpers.addPlotOptions(optParser)
    # parse
    options, remaining_args = optParser.parse_args(args=args)

    if options.net == None:
        print("Error: a network to load must be given.")
        return 1
    if options.verbose:
        print("Reading network from '%s'" % options.net)
    net = sumolib.net.readNet(options.net)

    tlsn = {}
    for tid in net._id2tls:
        t = net._id2tls[tid]
        tlsn[tid] = set()
        for c in t._connections:
            n = c[0].getEdge().getToNode()
            tlsn[tid].add(n)

    tlspX = []
    tlspY = []
    for tid in tlsn:
        x = 0
        y = 0
        n = 0
        for node in tlsn[tid]:
            x += node._coord[0]
            y += node._coord[1]
            n = n + 1
        x = x / n
        y = y / n
        tlspX.append(x)
        tlspY.append(y)

    fig, ax = helpers.openFigure(options)
    ax.set_aspect("equal", None, 'C')
    helpers.plotNet(net, {}, {}, options)
    plt.plot(tlspX, tlspY, options.color, linestyle='.',
             marker='o', markersize=options.width)
    options.nolegend = True
    helpers.closeFigure(fig, ax, options)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
