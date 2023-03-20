import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
        self.right = max([vertex[0] for vertex in vertices])



class Map:
    # here we give our start and end, and this class will calculate
    #  obstacles is a list of obstacle objects
    # prev_map is a way that map can take previous map objects and incorporate the intersections into a new map
    def __init__(self, origin=(0, 0), dim_x=0, dim_y=0, obstacles=[], drone_dim=0.1, prev_map = None):
        self.origin = origin
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.drone_dim = drone_dim
        self.obstacles = obstacles
        self.prev_map = prev_map

        fig = plt.figure()
        self.ax = fig.add_subplot(projection='3d')

        self.extents = [
            (0, self.dim_y),
            (self.dim_x, self.dim_y),
            (0, 0),
            (self.dim_x, 0),
        ]

        # creating map based on the length of x and y edges of the map - x=0 y=0 does NOT represent origin. to get origin use map.origin
        # because drone_dim is .1 meter, division makes each meter 10 nodes
        self.array = [
            [Node(coords=(x, y)) for x in range(int(dim_x / drone_dim))]
            for y in range(int(dim_y / drone_dim))
        ]

        self.create_obstacles(obstacles)

        # if there is a previous map inputted, grab its drone intersections so that you can populate the new map with higher priority drones
        if self.prev_map is not None:
            for y in range(len(self.prev_map.array)):
                for x in range(len(self.prev_map.array[0])):
                    if self.prev_map.array[y][x].intersections is not None:
                        self.array[y][x].set_intersection(self.prev_map.array[y][x].intersections)

    def create_obstacles(self, obstacles):
        for o in obstacles:
            # note that we are finding obstacle coords based on origin set above!!
            top = int((o.top + self.origin[1]) / self.drone_dim)
            bottom = int((o.bottom + self.origin[1]) / self.drone_dim)
            left = int((o.left + self.origin[0]) / self.drone_dim)
            right = int((o.right + self.origin[0]) / self.drone_dim)
            # print(f"{top=}, {bottom=}, {left=}, {right=}")

            # adjusting node obstacle property for all coords in obstacle
            for y in range(bottom, top + 1):
                for x in range(left, right + 1):
                    self.array[y][x].set_obstacle(True)

    def get_neighbors(self, node):
        # grabbing position of node (meters)
        x_pos = node.coords[0]
        y_pos = node.coords[1]

        # initializing number of nodes in x and y dimensions of map
        array_y = len(self.array)
        array_x = len(self.array[0])

        # Creating neighbor nodes based on no diagonal just horiz/vert movement
        # because this is relative location of nodes, we don't need to ref origin
        top = None if y_pos + 1 >= array_y else self.array[y_pos + 1][x_pos]
        bottom = None if y_pos - 1 < 0 else self.array[y_pos - 1][x_pos]
        left = None if x_pos - 1 < 0 else self.array[y_pos][x_pos - 1]
        right = None if x_pos + 1 >= array_x else self.array[y_pos][x_pos + 1]
        neighbors = [top, bottom, left, right]
        neighbors_filtered = [
            n for n in neighbors if n is not None and not n.is_obstacle
        ]
        return neighbors_filtered

    def sequence_to_path(self, seq):
        path = []
        # seq is a list of tuples indicating drone position in "coords" frame, not true origin relative to lighthouse frame
        for s in seq:
            if s is not None:
                path.append(
                    (
                        (s[0] * self.drone_dim) + 0.5 * self.drone_dim - self.origin[0],
                        (s[1] * self.drone_dim) + 0.5 * self.drone_dim - self.origin[1],
                    )
                )
        return path

    def visualize_map(self, path=None, waypoint_names=None):
        obstacles_x = []
        obstacles_y = []
        for y in range(len(self.array)):
            for x in range(len(self.array[0])):
                if self.array[y][x].is_obstacle:
                    obstacles_x.append(x)
                    obstacles_y.append(y)

        for z in np.linspace(0, 0.5, 20):
            self.ax.scatter(obstacles_x, obstacles_y, [z]*len(obstacles_x), color="black")

        if path.all():
            self.ax.scatter(path[:,0], path[:,1], path[:,2])

        self.ax.set_xlim([0, len(self.array[0])])
        self.ax.set_ylim([0, len(self.array)])
        self.ax.set_zlim([0, 2])
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        title = "Planned Crazyflie Path"
        if waypoint_names is not None:
            title += f"\nTakeoff Loc {waypoint_names[0]}, Hover @ {waypoint_names[1]}, Drop Loc {waypoint_names[2]}"
        self.ax.set_title(title)

        # plt.show()

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

    def __init__(self, coords=(0.0, 0.0)):
        """Creates a new Node with coordinates"""
        self._coords = coords
        self._is_obstacle = False
        self.parent = None
        self._cost = 0
        self._heuristic = 0
        self._f_cost = self.heuristic + self.cost
        self._intersections = []
          # empty list where drone intersections at time indeces will be recorded

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

    @property
    def intersections(self):
        return self.intersections
        # use this property to access points where drones intersect a node
        # will return a list of tuples [("droneID", time_idx), ("ID", idx)...]

    def get_parent(self):
        return self.parent

    def set_obstacle(self, val):
        self._is_obstacle = val

    def set_heuristic(self, val):
        self._heuristic = val
        self._f_cost = self._cost + self._heuristic

    def set_cost(self, val):
        self._cost = val
        self._f_cost = self._cost + self._heuristic

    def set_parent(self, val):
        self.parent = val

    def set_intersections(self, val):
        self.intersections.extend(val)
        # when appending intersections they are in the format ("drone ID", time idx)
        # where time idx is an int representing the step of A* that this intersection is

    def __sort__(self, other_node):
        return self.f_cost < other_node.f_cost

    def __repr__(self):
        return f"F_cost: {self.f_cost}"

    def __eq__(self, n):
        return self.coords == n.coords
