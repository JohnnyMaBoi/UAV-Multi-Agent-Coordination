import numpy as np

class Obstacle:
    def __init__(self, vertices):
        self.vertices = vertices
    

class Map:
    def __init__(self, dimensions, obstacles, drone_dim):
        self.dimensions = dimensions
        self.obstacles = obstacles

class Node:
    """
    Object representing a node in the map, which is a unit square the
    size of the Crazyflie drone.

    Attributes:
        coords (2 double tuple): Coordinates of the node in the map
        is_obstacle (bool): Whether the node is an obstacle
        parent (Node): Parent of this node for A*
        heuristic (double): Heuristic value to evaluate Node viability for A*
    """      
    def __init__(self, coords = (0.0, 0.0)):
        """ Creates a new Node with coordinates """
        self._coords = coords
        self._is_obstacle = False
        self.parent = Node()
        self.heuristic = 0
    
    @property
    def coords(self):
        return self._coords
    
    @property
    def is_obstacle(self):
        return self._is_obstacle
    
    @property
    def heuristic(self):
        return self.heuristic

    def get_parent(self):
        return self.parent

    def set_obstacle(self, val):
        self._is_obstacle = val
    
    def set_heuristic(self, val):
        self.heuristic = val
    
    def set_parent(self, val):
        self.parent = val
