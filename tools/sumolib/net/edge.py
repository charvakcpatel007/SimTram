"""
@file    edge.py
@author  Daniel Krajzewicz
@author  Laura Bieker
@author  Karol Stosiek
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2011-11-28
@version $Id: edge.py 20687 2016-05-10 11:27:00Z behrisch $

This file contains a Python-representation of a single edge.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2011-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from .connection import Connection


class Edge:

    """ Edges from a sumo network """

    def __init__(self, id, fromN, toN, prio, function, name):
        self._id = id
        self._from = fromN
        self._to = toN
        self._priority = prio
        if fromN:
            fromN.addOutgoing(self)
        if toN:
            toN.addIncoming(self)
        self._lanes = []
        self._speed = None
        self._length = None
        self._incoming = {}
        self._outgoing = {}
        self._shape = None
        self._cachedShapeWithJunctions = None
        self._function = function
        self._tls = None
        self._name = name

    def getName(self):
        return self._name

    def isSpecial(self):
        return self._function != ""

    def getFunction(self):
        return self._function

    def getPriority(self):
        return self._priority

    def getTLS(self):
        return self._tls

    def addLane(self, lane):
        self._lanes.append(lane)
        self._speed = lane.getSpeed()
        self._length = lane.getLength()

    def addOutgoing(self, conn):
        if conn._to not in self._outgoing:
            self._outgoing[conn._to] = []
        self._outgoing[conn._to].append(conn)

    def _addIncoming(self, conn):
        if conn._from not in self._incoming:
            self._incoming[conn._from] = []
        self._incoming[conn._from].append(conn)

    def setShape(self, shape):
        self._shape = shape
        self._cachedShapeWithJunctions = None

    def getID(self):
        return self._id

    def getIncoming(self):
        return self._incoming

    def getOutgoing(self):
        return self._outgoing

    def getShape(self, includeJunctions=False):
        if not self._shape:
            if self._cachedShapeWithJunctions == None:
                self._cachedShapeWithJunctions = [
                    self._from._coord, self._to._coord]
            return self._cachedShapeWithJunctions
        if includeJunctions:
            if self._cachedShapeWithJunctions == None:
                if self._from._coord != self._shape[0]:
                    self._cachedShapeWithJunctions = [
                        self._from._coord] + self._shape
                else:
                    self._cachedShapeWithJunctions = list(self._shape)
                if self._to._coord != self._shape[-1]:
                    self._cachedShapeWithJunctions += [self._to._coord]
            return self._cachedShapeWithJunctions
        return self._shape

    def getBoundingBox(self, includeJunctions=True):
        s = self.getShape(includeJunctions)
        xmin = s[0][0]
        xmax = s[0][0]
        ymin = s[0][1]
        ymax = s[0][1]
        for p in s[1:]:
            xmin = min(xmin, p[0])
            xmax = max(xmax, p[0])
            ymin = min(ymin, p[1])
            ymax = max(ymax, p[1])
        assert(xmin != xmax or ymin != ymax)
        return (xmin, ymin, xmax, ymax)

    def getClosestLanePosDist(self, point, perpendicular=False):
        minDist = 1e400
        minIdx = None
        minPos = None
        for i, l in enumerate(self._lanes):
            pos, dist = l.getClosestLanePosAndDist(point, perpendicular)
            if dist < minDist:
                minDist = dist
                minIdx = i
                minPos = pos
        return minIdx, minPos, minDist

    def getSpeed(self):
        return self._speed

    def getLaneNumber(self):
        return len(self._lanes)

    def getLane(self, idx):
        return self._lanes[idx]

    def getLanes(self):
        return self._lanes

    def rebuildShape(self):
        noShapes = len(self._lanes)
        if noShapes % 2 == 1:
            self.setShape(self._lanes[int(noShapes / 2)]._shape)
        else:
            shape = []
            minLen = -1
            for l in self._lanes:
                if minLen == -1 or minLen > len(l.getShape()):
                    minLen = len(l._shape)
            for i in range(0, minLen):
                x = 0.
                y = 0.
                for j in range(0, len(self._lanes)):
                    x = x + self._lanes[j]._shape[i][0]
                    y = y + self._lanes[j]._shape[i][1]
                x = x / float(len(self._lanes))
                y = y / float(len(self._lanes))
                shape.append([x, y])
            self.setShape(shape)

    def getLength(self):
        return self._lanes[0].getLength()

    def setTLS(self, tls):
        self._tls = tls

    def getFromNode(self):
        return self._from

    def getToNode(self):
        return self._to

    def is_fringe(self, connections=None):
        """true if this edge has no incoming or no outgoing connections (except turnarounds)
           If connections is given, only those connections are considered"""
        if connections is None:
            return self.is_fringe(self._incoming) or self.is_fringe(self._outgoing)
        else:
            cons = sum([c for c in connections.values()], [])
            return len([c for c in cons if c._direction != Connection.LINKDIR_TURN]) == 0

    def allows(self, vClass):
        """true if this edge has a lane which allows the given vehicle class"""
        for lane in self._lanes:
            if vClass in lane._allowed:
                return True
        return False

    def __repr__(self):
        if self.getFunction() == '':
            return '<edge id="%s" from="%s" to="%s"/>' % (self._id, self._from.getID(), self._to.getID())
        else:
            return '<edge id="%s" function="%s"/>' % (self._id, self.getFunction())
