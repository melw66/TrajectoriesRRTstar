#!/usr/bin/env python

from RRTstar import RRTstar

import random
import math
import shapely.geometry
from operator import sub

class GenerateRRTstar():
    def __init__(self, x_range, bounds_array, obstacle_array, radius, unit_distance):
        self.x_range = x_range
        self.bounds_array = bounds_array
        self.obstacle_array = obstacle_array
        self.radius = radius
        self.unit_distance = unit_distance
        self.probs_array = self.generateProbs()

    # generates probabilities that a point generated uniformly at random in the free
    # space will be in each region (in form: [0, prob1, prob1 + prob2, prob1 + prob2 + prob3, ...])
    def generateProbs(self):
        probs_array = [0]
        sum = 0
        for region in self.bounds_array:
            x_range = region[0][1] - region[0][0]
            z_range = region[1][1] - region[1][0]
            sum += x_range * z_range
            probs_array.append(sum)
        for i in range(1, len(probs_array)):
            probs_array[i] = probs_array[i] / sum
        return probs_array

    # uniformly at random generates a valid (free) point in the specific environment
    def randomState(self):
        region = random.random()
        for i in range(len(self.probs_array) - 1):
            if self.probs_array[i] <= region < self.probs_array[i + 1]:
                x_value = random.uniform(self.bounds_array[i][0][0], self.bounds_array[i][0][1])
                z_value = random.uniform(self.bounds_array[i][1][0], self.bounds_array[i][1][1])
                return x_value, z_value

    # calculates 2D Euclidean distance
    def distance(self, vertex_a, vertex_b):
        return math.sqrt((vertex_a[0] - vertex_b[0])**2 + (vertex_a[1] - vertex_b[1])**2)

    # returns the nearest vertex in tree to a point not in the tree
    def nearestNeighborTuple(self, random_state, tree):
        min_distance = float("inf")
        nearest_neighbor = tree.getVertices()[0]
        for vertex in tree.getVertices():
            if self.distance(random_state, vertex) < min_distance:
                min_distance = self.distance(random_state, vertex)
                nearest_neighbor = vertex
        return nearest_neighbor, min_distance

    # given a random state and a start point, generates the point along vector from
    # start point to random state that is unit distance from the start point
    def newState(self, random_state, nearest_tuple):
        if nearest_tuple[1] <= self.unit_distance:
            return random_state
        vector = tuple(map(sub, random_state, nearest_tuple[0]))
        scale = self.unit_distance / nearest_tuple[1]
        new_state = (nearest_tuple[0][0] + vector[0] * scale, nearest_tuple[0][1] + vector[1] * scale)
        return new_state

    # returns true if a line between 2 points doesn't pass through an obstacle
    def obstacleFree(self, nearest_neighbor, new_state):
        line = shapely.geometry.LineString([list(nearest_neighbor), list(new_state)])
        for obstacle in self.obstacle_array:
            if line.intersects(obstacle):
                return False
        return True

    # returns all vertices in region within unit distance from a point
    def checkNeighborhood(self, new_state, tree):
        neighbors = []
        for vertex in tree.getVertices():
            if self.distance(new_state, vertex) <= self.unit_distance:
                neighbors.append(vertex)
        return neighbors

    # generates tree
    def createRRTstar(self, x_init, num_iters):
        new_tree = RRTstar()
        new_tree.addVertex(x_init[0], x_init[1])

        for i in range(num_iters):
            x_rand = self.randomState()
            x_near_tuple = self.nearestNeighborTuple(x_rand, new_tree)
            x_new = self.newState(x_rand, x_near_tuple)
            if self.obstacleFree(x_near_tuple[0], x_new):
                neighbors = self.checkNeighborhood(x_new, new_tree)
                new_tree.addVertex(x_new[0], x_new[1])
                x_min = x_near_tuple[0]
                cost_min = new_tree.getCost(x_near_tuple[0]) + self.distance(x_min, x_new)
                for vertex in neighbors:
                    if self.obstacleFree(vertex, x_new):
                        if (new_tree.getCost(vertex) + self.distance(vertex, x_new)) < cost_min:
                            x_min = vertex
                            cost_min = new_tree.getCost(vertex) + self.distance(vertex, x_new)
                new_tree.addEdge(x_min, x_new)
                for vertex in neighbors:
                    if self.obstacleFree(x_new, vertex):
                        if (new_tree.getCost(x_new) + self.distance(x_new, vertex)) < new_tree.getCost(vertex):
                            parent = new_tree.getParent(vertex)
                            new_tree.removeEdge(parent, vertex)
                            new_tree.addEdge(x_new, vertex)
        return new_tree
