import numpy as np


class Obstacle:
    def __init__(self, vertices):
        """
        vertices are inputted as a list of  (x, y) tuples in meters.

        This class only works for rectangular obstacles that are in line with x and y axes
        """
        self.vertices = vertices
        self.top = max([vertex[1] for vertex in vertices])
        self.bottom = min([vertex[1] for vertex in vertices])
        self.left = min([vertex[0] for vertex in vertices])
        self.right = min([vertex[0] for vertex in vertices])


class Map:
    # here we give our start and end, and this class will calculate
    #  obstacles is a list of obstacle objects
    def __init__(self, origin, dim_x, dim_y, obstacles, drone_dim):
        self.origin = origin
        self.dim_x = dim_x
        self.dim_y = dim_y

        self.extents = [
            (0, self.dim_y),
            (self.dim_x, self.dim_y),
            (0, 0),
            (self.dim_x, 0)
        ]

        self.array = [[Node(coords=(x,y)) for x in range(int(dim_x / drone_dim))] for y in range(int(dim_y / drone_dim))]

        # Fill nodes that are obstacles here
    
    def a_star(self, start, end):
        pass

class Node:
    """
    Object representing a node in the map, which is a unit square the
    size of the Crazyflie drone.

    Attributes:
        _coords (2 double tuple): Coordinates of the node in the map
        _is_obstacle (bool): Whether the node is an obstacle
        parent (Node): Parent of this node for A*
        _cost (double): Cost value for distance from start to node position
        _heuristic (double): Heuristic value for distance to end from node position
        _f_cost (double): Total cost of node
    """      
    def __init__(self, coords = (0.0, 0.0)):
        """ Creates a new Node with coordinates """
        self._coords = coords
        self._is_obstacle = False
        self.parent = None
        self._cost = 0
        self._heuristic = 0
        self._f_cost = self.heuristic + self.cost
    
    @property
    def coords(self):
        return self._coords
    
    @property
    def is_obstacle(self):
        return self._is_obstacle
    
    @property
    def cost(self):
        return self._cost

    @property
    def heuristic(self):
        return self._heuristic

    @property
    def f_cost(self):
        self._f_cost = self._cost + self._heuristic
        return self._f_cost

    def get_parent(self):
        return self.parent

    def set_obstacle(self, val):
        self._is_obstacle = val
    
    def set_heuristic(self, val):
        self._heuristic = val

    def set_cost(self, val):
        self._cost = val
    
    def set_parent(self, val):
        self.parent = val

    def __sort__(self, other_node):
        return (self.f_cost < other_node.f_cost)
