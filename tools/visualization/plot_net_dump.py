#!/usr/bin/env python
"""
@file    plot_net_dump.py
@author  Daniel Krajzewicz
@author  Laura Bieker
@date    2013-10-14
@version $Id: plot_net_dump.py 20724 2016-05-17 05:32:31Z namdre $

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
import matplotlib


class WeightsReader(ContentHandler):

    """Reads the dump file"""

    def __init__(self, value):
        self._edge2value = {}
        self._value = value
        self._intervals = []

    def startElement(self, name, attrs):
        if name == 'interval':
            self._time = float(attrs['begin'])
            self._edge2value[self._time] = {}
            self._intervals.append(self._time)
        if name == 'edge':
            id = attrs['id']
            if self._value in attrs:
                self._edge2value[self._time][id] = float(attrs[self._value])


def main(args=None):
    """The main function; parses options and plots"""
    # ---------- build and read options ----------
    from optparse import OptionParser
    optParser = OptionParser()
    optParser.add_option("-n", "--net", dest="net", metavar="FILE",
                         help="Defines the network to read")
    optParser.add_option("-i", "--dump-inputs", dest="dumps", metavar="FILE",
                         help="Defines the dump-output files to use as input")
    optParser.add_option("-m", "--measures", dest="measures",
                         default="speed,entered", help="Define which measure to plot")
    optParser.add_option("--min-width", dest="minWidth",
                         type="float", default=.5, help="Defines the minimum edge width")
    optParser.add_option("--max-width", dest="maxWidth",
                         type="float", default=3, help="Defines the maximum edge width")
    optParser.add_option("--log-colors", dest="logColors", action="store_true",
                         default=False, help="If set, colors are log-scaled")
    optParser.add_option("--log-widths", dest="logWidths", action="store_true",
                         default=False, help="If set, widths are log-scaled")
    optParser.add_option("--min-color-value", dest="colorMin",
                         type="float", default=None, help="If set, defines the minimum edge color value")
    optParser.add_option("--max-color-value", dest="colorMax",
                         type="float", default=None, help="If set, defines the maximum edge color value")
    optParser.add_option("--min-width-value", dest="widthMin",
                         type="float", default=None, help="If set, defines the minimum edge width value")
    optParser.add_option("--max-width-value", dest="widthMax",
                         type="float", default=None, help="If set, defines the maximum edge width value")
    optParser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                         default=False, help="If set, the script says what it's doing")
    # standard plot options
    helpers.addInteractionOptions(optParser)
    helpers.addPlotOptions(optParser)
    helpers.addNetOptions(optParser)
    # parse
    options, remaining_args = optParser.parse_args(args=args)

    if options.net == None:
        print("Error: a network to load must be given.")
        return 1
    if options.verbose:
        print("Reading network from '%s'" % options.net)
    net = sumolib.net.readNet(options.net)

    if options.measures == None:
        print("Error: a dump file must be given.")
        return 1

    times = []
    hc = None
    if options.dumps.split(",")[0] != "":
        if options.verbose:
            print("Reading colors from '%s'" % options.dumps.split(",")[0])
        hc = WeightsReader(options.measures.split(",")[0])
        sumolib.output.parse_sax(options.dumps.split(",")[0], hc)
        times = hc._edge2value

    hw = None
    if options.dumps.split(",")[1] != "":
        if options.verbose:
            print("Reading widths from '%s'" % options.dumps.split(",")[1])
        hw = WeightsReader(options.measures.split(",")[1])
        sumolib.output.parse_sax(options.dumps.split(",")[1], hw)
        times = hw._edge2value

    for t in times:
        if options.verbose:
            print("Processing timestep %s" % t)
        colors = {}
        maxColorValue = None
        minColorValue = None
        for e in net._id2edge:
            if hc and t in hc._edge2value and e in hc._edge2value[t]:
                if options.colorMax != None and hc._edge2value[t][e] > options.colorMax:
                    hc._edge2value[t][e] = options.colorMax
                if options.colorMin != None and hc._edge2value[t][e] < options.colorMin:
                    hc._edge2value[t][e] = options.colorMin
                if maxColorValue == None or maxColorValue < hc._edge2value[t][e]:
                    maxColorValue = hc._edge2value[t][e]
                if minColorValue == None or minColorValue > hc._edge2value[t][e]:
                    minColorValue = hc._edge2value[t][e]
                colors[e] = hc._edge2value[t][e]
        if options.colorMax != None:
            maxColorValue = options.colorMax
        if options.colorMin != None:
            minColorValue = options.colorMin
        if options.logColors:
            helpers.logNormalise(colors, maxColorValue)
        else:
            helpers.linNormalise(colors, minColorValue, maxColorValue)
        for e in colors:
            colors[e] = helpers.getColor(options, colors[e], 1.)
        if options.verbose:
            print("Color values are between %s and %s" %
                  (minColorValue, maxColorValue))

        widths = {}
        maxWidthValue = None
        minWidthValue = None
        for e in net._id2edge:
            if hw and t in hw._edge2value and e in hw._edge2value[t]:
                v = abs(hw._edge2value[t][e])
                if options.widthMax != None and v > options.widthMax:
                    v = options.widthMax
                if options.widthMin != None and v < options.widthMin:
                    v = options.widthMin
                if not maxWidthValue or maxWidthValue < v:
                    maxWidthValue = v
                if not minWidthValue or minWidthValue > v:
                    minWidthValue = v
                widths[e] = v
        if options.widthMax != None:
            maxWidthValue = options.widthMax
        if options.widthMin != None:
            minWidthValue = options.widthMin
        if options.logWidths:
            helpers.logNormalise(widths, options.colorMax)
        else:
            helpers.linNormalise(widths, minWidthValue, maxWidthValue)
        for e in widths:
            widths[e] = options.minWidth + widths[e] * \
                (options.maxWidth - options.minWidth)
        if options.verbose:
            print("Width values are between %s and %s" %
                  (minWidthValue, maxWidthValue))

        fig, ax = helpers.openFigure(options)
        ax.set_aspect("equal", None, 'C')
        helpers.plotNet(net, colors, widths, options)

        # drawing the legend, at least for the colors
        sm = plt.cm.ScalarMappable(cmap=matplotlib.cm.get_cmap(
            options.colormap), norm=matplotlib.colors.Normalize(vmin=minColorValue, vmax=maxColorValue))
        # "fake up the array of the scalar mappable. Urgh..." (pelson, http://stackoverflow.com/questions/8342549/matplotlib-add-colorbar-to-a-sequence-of-line-plots)
        sm._A = []
        plt.colorbar(sm)

        options.nolegend = True
        optName = None
        if options.output and options.output.find("%s") >= 0:
            m, s = divmod(int(t), 60)
            h, m = divmod(m, 60)
            timeStr = "%02d:%02d:%02d" % (h, m, s)
            ax.text(7300, 15500, timeStr, bbox={
                    'facecolor': 'white', 'pad': 10}, size=16)
            optName = options.output.replace("%s", str(t))
        helpers.closeFigure(fig, ax, options, False, optName)
        if optName == None:
            return 0
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
