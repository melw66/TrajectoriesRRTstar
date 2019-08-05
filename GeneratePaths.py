#!/usr/bin/env python

from RRTstar import RRTstar
from GenerateRRTstar import GenerateRRTstar

import random
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

"""
Generates 100 start points and generates 10 paths per start point (total of 1000 paths)
"""

my_generator = GenerateRRTstar((0.313, 4.387), bounds_array, obstacle_array, 0.25, 0.25)

paths = []

for i in range(100):
    random_init = my_generator.randomState()
    tree = my_generator.createRRTstar(random_init, 1000)
    for j in range(10):
        current_path = []
        random_index = random.randint(300, len(tree.getVertices()) - 1)
        random_end = tree.getVertices()[random_index]
        vertex = random_end
        while tree.getParent(vertex) != None:
            current_path.append(vertex)
            vertex = tree.getParent(vertex)
        current_path.append(vertex)
        paths.append(current_path)

return paths
