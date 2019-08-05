#!/usr/bin/env python

from RRTstar import RRTstar
from GenerateRRTstar import GenerateRRTstar

import shapely.geometry
import matplotlib.pyplot as plt
import descartes
import numpy as np

# create bounds array
bounds_array = [[(0.313, 0.613), (0.313, 1.339)],
                [(0.613, 1.013), (0.313, 1.687)],
                [(1.013, 1.888), (0.313, 5.087)],
                [(1.888, 3.288), (0.313, 2.518)],
                [(1.888, 3.288), (4.018, 5.087)],
                [(3.288, 3.713), (0.313, 5.087)],
                [(3.713, 4.387), (0.313, 5.787)],
                [(4.387, 4.387), (0.313, 5.787)]]

# create obstacle array
whole_floor = shapely.geometry.Polygon([[0, 0], [0, 6.1], [4.7, 6.1], [4.7, 0]])
large_free = shapely.geometry.Polygon([[0.313, 0.313], [0.313, 1.339], [0.613, 1.339], [0.613, 1.687], [1.013, 1.687], [1.013, 5.087], [3.713, 5.087], [3.713, 5.787], [4.387, 5.787], [4.387, 0.313]])
clipped_obstacle = whole_floor.difference(large_free)
table = shapely.geometry.Polygon([[1.888, 2.518], [1.888, 4.018], [3.288, 4.018], [3.288, 2.518]])
obstacle_array = [clipped_obstacle, table]

# create RRT*
my_generator = GenerateRRTstar((0.313, 4.387), bounds_array, obstacle_array, 0.25, 0.25)
random_init = my_generator.randomState()
tree = my_generator.createRRTstar(random_init, 800)

# create plot
fig = plt.figure()
ax = fig.add_subplot(111)

# plot the obstacles in the environment
for obstacle in obstacle_array:
    ax.add_patch(descartes.PolygonPatch(obstacle, fc='blue', alpha=0.5))

# plot the vertices and edges of the generated RRT*
for i in range(1, len(tree.getVertices())):
    ax.plot(tree.getVertices()[i][0], tree.getVertices()[i][1], 'bo')
for edge in tree.getEdges():
    line = shapely.geometry.LineString([list(edge[0]), list(edge[1])])
    ax.plot(*np.array(line).T, color='green', linewidth=1, solid_capstyle='round')

# plot the start point in red
ax.plot(random_init[0], random_init[1], 'ro')

# show plot
ax.axis('equal')
plt.show()
