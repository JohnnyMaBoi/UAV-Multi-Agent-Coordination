import numpy as np

class Obstacle:
    def __init__(self, vertices):
        self.vertices = vertices
    

class Map:
    def __init__(self, dimensions, obstacles, drone_dim):
        self.dimensions = dimensions
        self.obstacles = obstacles
