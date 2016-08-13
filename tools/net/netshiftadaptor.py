"""
@file    netshiftadaptor.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2008-09-01
@version $Id: netshiftadaptor.py 20433 2016-04-13 08:00:14Z behrisch $


This class performs a network reprojection
 using barycentric cordinates of two triangles
 which share the same nodes of two networks.

This means: the class is initialised with two
 networks and two lists of node ids (should be
 exactly three). The according nodes should be 
 the "same" nodes in both networks.
When "reproject" is called, all nodes' position
 of the second network are reprojected so that
 they match positions within the first network.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""


class NetShiftAdaptor:

    def __init__(self, net1, net2, nodes1, nodes2):
        self._net1 = net1
        self._net2 = net2
        self._nodes1 = nodes1
        self._nodes2 = nodes2
        if len(nodes1) != 3 or len(nodes2) != 3:
            raise "Both node lists must contain exactly 3 node ids"

    def reproject(self, verbose=False):
        x11 = self._net1._id2node[self._nodes1[0]]._coord[0]
        y11 = self._net1._id2node[self._nodes1[0]]._coord[1]
        x12 = self._net1._id2node[self._nodes1[1]]._coord[0]
        y12 = self._net1._id2node[self._nodes1[1]]._coord[1]
        x13 = self._net1._id2node[self._nodes1[2]]._coord[0]
        y13 = self._net1._id2node[self._nodes1[2]]._coord[1]
        x21 = self._net2._id2node[self._nodes2[0]]._coord[0]
        y21 = self._net2._id2node[self._nodes2[0]]._coord[1]
        x22 = self._net2._id2node[self._nodes2[1]]._coord[0]
        y22 = self._net2._id2node[self._nodes2[1]]._coord[1]
        x23 = self._net2._id2node[self._nodes2[2]]._coord[0]
        y23 = self._net2._id2node[self._nodes2[2]]._coord[1]
        b0 = (x22 - x21) * (y23 - y21) - (x23 - x21) * (y22 - y21)
        for n in self._net2._nodes:
            x0 = n._coord[0]
            y0 = n._coord[1]
            b1 = ((x22 - x0) * (y23 - y0) - (x23 - x0) * (y22 - y0)) / b0
            b2 = ((x23 - x0) * (y21 - y0) - (x21 - x0) * (y23 - y0)) / b0
            b3 = ((x21 - x0) * (y22 - y0) - (x22 - x0) * (y21 - y0)) / b0
            n._coord = (
                b1 * x11 + b2 * x12 + b3 * x13, b1 * y11 + b2 * y12 + b3 * y13)
        for e in self._net2._edges:
            for l in e._lanes:
                shape = []
                for p in l._shape:
                    x0 = p[0]
                    y0 = p[1]
                    b1 = (
                        (x22 - x0) * (y23 - y0) - (x23 - x0) * (y22 - y0)) / b0
                    b2 = (
                        (x23 - x0) * (y21 - y0) - (x21 - x0) * (y23 - y0)) / b0
                    b3 = (
                        (x21 - x0) * (y22 - y0) - (x22 - x0) * (y21 - y0)) / b0
                    x = (b1 * x11 + b2 * x12 + b3 * x13)
                    y = (b1 * y11 + b2 * y12 + b3 * y13)
                    shape.append((x, y))
                l._shape = shape
            e.rebuildShape()
