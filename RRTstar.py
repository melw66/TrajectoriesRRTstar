#!/usr/bin/env python

import math

class RRTstar():
    def __init__(self):
        self.vertices = []
        self.edges = []
        self.costs = {}
        self.parent = {}
        self.children = {}

    def addVertex(self, x_pos, z_pos):
        self.vertices.append((x_pos, z_pos))
        self.costs[(x_pos, z_pos)] = 0
        self.parent[(x_pos, z_pos)] = None
        self.children[(x_pos, z_pos)] = []

    def addEdge(self, from_vertex, to_vertex):
        self.edges.append((from_vertex, to_vertex))
        distance = math.sqrt((from_vertex[0] - to_vertex[0])**2 + (from_vertex[1] - to_vertex[1])**2)
        self.costs[to_vertex] = self.costs[from_vertex] + distance
        self.parent[to_vertex] = from_vertex
        self.children[from_vertex].append(to_vertex)

    def removeVertex(self, x_pos, z_pos):
        if (x_pos, z_pos) in self.vertices:
            self.vertices.remove((x_pos, z_pos))
            self.costs.remove((x_pos, z_pos))
            parent = self.parent[(x_pos, z_pos)]
            if parent != None:
                self.children[parent].remove((x_pos, z_pos))
            for child in self.children[(x_pos, z_pos)]:
                self.parent[child] = None

    def removeEdge(self, from_vertex, to_vertex):
        if (from_vertex, to_vertex) in self.edges:
            self.edges.remove((from_vertex, to_vertex))
            self.costs[to_vertex] = 0
            self.parent[to_vertex] = None
            self.children[from_vertex].remove(to_vertex)

    def getVertices(self):
        return self.vertices

    def getEdges(self):
        return self.edges

    def getCost(self, vertex):
        return self.costs[vertex]

    def getParent(self, vertex):
        return self.parent[vertex]
