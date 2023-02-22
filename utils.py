import numpy as np

vertices1 = [(0, 0.3), (0, 0.605), (1.22, 0.605), (1.22, 0.3)]
vertices2 = [(0, -0.3), (0, -0.605), (1.22, -0.605), (1.22, -0.3)]


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


class Node:
    def __init__(self, position, is_obstacle, parent=None, cost=0, heuristic=0):
        self.position = position
        self.is_obstacle = is_obstacle
        self.parent = parent
        self.cost = cost  # cost to reach node from starting point
        self.heuristic = heuristic  # cost to reach end from current position
        self.f_cost = self.heuristic + self.cost

    def __sort__(self, other_node):
        return (
            self.f_cost < other_node.f_cost
        )  # returns true if the node is less costly than another node


class Map:
    # here we give our start and end, and this class will calculate
    #  obstacles is a list of obstacle objects
    def __init__(self, bottomleft, dim_x, dim_y, obstacles, drone_dim):
        self.bottomleft = bottomleft
        self.dim_x = dim_x
        self.dim_y = dim_y

        self.extents = [
            (self.bottomleft[0], self.bottomleft[1] + self.dim_y),
            (self.bottomleft[0] + self.dim_x, self.bottomleft[1] + self.dim_y),
            (self.bottomleft[0], self.bottomleft[1])(
                self.bottomleft[0] + self.dim_x, self.bottomleft[1]
            ),
        ]

        self.array = np.zeros(((dim_x / drone_dim), (dim_y / drone_dim)))

    # function that does a*, spits out waypoints and path
    #
