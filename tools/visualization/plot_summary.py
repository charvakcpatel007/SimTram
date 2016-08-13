#!/usr/bin/env python
"""
@file    plot_summary.py
@author  Daniel Krajzewicz
@author  Laura Bieker
@date    2013-11-11
@version $Id: plot_summary.py 20433 2016-04-13 08:00:14Z behrisch $


This script plots a selected measure from a summary-output.
matplotlib (http://matplotlib.org/) has to be installed for this purpose


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2013-2016 DLR (http://www.dlr.de/) and contributors

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

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
import sumolib
from sumolib.visualization import helpers

import matplotlib.pyplot as plt


def readValues(files, verbose, measure):
    ret = {}
    for f in files:
        if verbose:
            print("Reading '%s'..." % f)
        ret[f] = sumolib.output.parse_sax__asList(f, "step", [measure])
    return ret


def main(args=None):
    """The main function; parses options and plots"""
    # ---------- build and read options ----------
    from optparse import OptionParser
    optParser = OptionParser()
    optParser.add_option("-i", "--summary-inputs", dest="summary", metavar="FILE",
                         help="Defines the summary-output files to use as input")
    optParser.add_option("-v", "--verbose", dest="verbose", action="store_true",
                         default=False, help="If set, the script says what it's doing")
    optParser.add_option("-m", "--measure", dest="measure",
                         default="running", help="Define which measure to plot")
    # standard plot options
    helpers.addInteractionOptions(optParser)
    helpers.addPlotOptions(optParser)
    # parse
    options, remaining_args = optParser.parse_args(args=args)

    if options.summary == None:
        print("Error: at least one summary file must be given")
        sys.exit(1)

    minV = 0
    maxV = 0
    files = options.summary.split(",")
    nums = readValues(files, options.verbose, options.measure)
    for f in files:
        maxV = max(maxV, len(nums[f]))
    ts = range(minV, maxV + 1)

    fig, ax = helpers.openFigure(options)
    for i, f in enumerate(files):
        v = sumolib.output.toList(nums[f], options.measure)
        c = helpers.getColor(options, i, len(files))
        l = helpers.getLabel(f, i, options)
        plt.plot(ts[0:len(v)], v, label=l, color=c)
    helpers.closeFigure(fig, ax, options)

if __name__ == "__main__":
    sys.exit(main(sys.argv))
