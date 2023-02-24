import matplotlib.pyplot as plt

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
        self.right = max([vertex[0] for vertex in vertices])


class Map:
    # here we give our start and end, and this class will calculate
    #  obstacles is a list of obstacle objects
    def __init__(self, origin=(0,0), dim_x=0, dim_y=0, obstacles=[], drone_dim=0.1):
        self.origin = origin
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.drone_dim = drone_dim
        self.obstacles = obstacles

        self.extents = [
            (0, self.dim_y),
            (self.dim_x, self.dim_y),
            (0, 0),
            (self.dim_x, 0)
        ]

        self.array = [[Node(coords=(x,y)) for x in range(int(dim_x / drone_dim))] for y in range(int(dim_y / drone_dim))]

        self.create_obstacles(obstacles)
    
    def create_obstacles(self, obstacles):
        for o in obstacles:
            top = int((o.top + self.origin[1])/self.drone_dim)
            bottom = int((o.bottom + self.origin[1])/self.drone_dim)
            left = int((o.left + self.origin[0])/self.drone_dim)
            right = int((o.right + self.origin[0])/self.drone_dim)
            # print(f"{top=}, {bottom=}, {left=}, {right=}")
            for y in range(bottom, top+1):
                for x in range(left, right+1):
                    self.array[y][x].set_obstacle(True)

    def visualize_map(self):
        obstacles_x =  []
        obstacles_y = []
        for y in range(len(self.array)):
            for x in range(len(self.array[0])):
                if self.array[y][x].is_obstacle:
                    obstacles_x.append(x)
                    obstacles_y.append(y)

        plt.scatter(obstacles_x, obstacles_y)
        plt.xlim([0, len(self.array[0])])
        plt.ylim([0, len(self.array)])
        plt.show()
    
    def a_star(self, start, end):
        return [(0.0, 0.0, 0.0, 0.0)]

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
