import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from fly_sequence import (
    reset_estimator,
    run_sequence,
    start_position_printing,
    get_pose,
)
from map_objects import Obstacle, Map, Node
from a_star import Astar

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default="radio://0/90/2M/E7E7E7E7E7")

sequence = []

obstacles = [
    [(0, 0.3), (0, 0.605), (1.22, 0.605), (1.22, 0.3)],
    [(0, -0.3), (0, -0.605), (1.22, -0.605), (1.22, -0.3)],
]

drop_locations = {
    "A": (-0.6, -0.6),
    "B": (-0.6, -0.3),
    "C": (-0.6, 0),
    "D": (-0.6, 0.3),
    "E": (-0.6, 0.6),
}

start_locations = {"1": (1.8, -0.3), "2": (1.8, 0.6)}

hover_locations = {
    "Bin 1a": (0.15, 0.25),
    "Bin 2a": (0.455, 0.25),
    "Bin 3a": (0.76, 0.25),
    "Bin 4a": (1.065, 0.25),
    "Bin 1b": (0.15, -0.25),
    "Bin 2b": (0.455, -0.25),
    "Bin 3b": (0.76, -0.25),
    "Bin 4b": (1.065, -0.25),
}

hover_height = 0.45

# Lighthouse origin relative to bottom left of map
map_origin = (1, 1)

map_dim_x = 3.5
map_dim_y = 2

if __name__ == "__main__":

    # obstacle_list = [Obstacle(obstacles[i]) for i in range(len(obstacles))]
    # map = Map(origin=map_origin, dim_x=map_dim_x, dim_y=map_dim_y, obstacles=obstacle_list, drone_dim=0.1)
    # map.visualize_map()

    cflib.crtp.init_drivers()

    obstacle_list = [Obstacle(obstacles[i]) for i in range(len(obstacles))]
    map = Map(
        origin=map_origin,
        dim_x=map_dim_x,
        dim_y=map_dim_y,
        obstacles=obstacle_list,
        drone_dim=0.1,
    )
    # map.visualize_map()

    # Select start, hover, & end points
    start = start_locations["1"]
    hover_point = hover_locations["Bin 2a"]
    end = drop_locations["B"]

    astar_to_hover = Astar(map=map, start=start, target=hover_point)
    seq_to_hover = astar_to_hover.a_star_search()
    print(seq_to_hover)
    # visualize a* sequence:
    map.visualize_map(path=seq_to_hover)

    # with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

    #     """
    #     Pseudocode:
    #     Add takeoff to sequence
    #     Run A* between start and hover location, set z pos to hover height, add to sequence
    #     Run A* between hover and drop location, set z pos to hover height (add landing), add to sequence
    #     Add landing to sequence
    #     Run sequence
    #     """

    #     obstacle_list = [Obstacle(obstacles[i]) for i in range(len(obstacles))]
    #     map = Map(origin=map_origin, dim_x=map_dim_x, dim_y=map_dim_y, obstacles=obstacle_list, drone_dim=0.1)
    #     # map.visualize_map()

    #     # Select start, hover, & end points
    #     start = start_locations['1']
    #     hover_point = hover_locations['Bin 2a']
    #     end = drop_locations['B']

    #     # Add takeoff to sequence
    #     x, y, z = get_pose(scf)
    #     sequence.append((x, y, hover_height/2, 0.0))
    #     sequence.append((start[0], start[1], hover_height/2, 0.0))
    #     sequence.append((start[0], start[1], hover_height, 0.0))

    #     astar_to_hover = Astar(map=map, start=start, target=hover_point)
    #     # visualize a* sequence:
    #     map.visualize_map(path=astar_to_hover.a_star_search())

    #     # Add first flight sequence
    #     seq_to_hover = astar_to_hover.a_star_search()
    #     for i in range(len(seq_to_hover)):
    #         seq_to_hover[i][2] = hover_height

    #     sequence.extend(seq_to_hover)

    #     astar_to_drop = Astar(map=map, start=hover_point, target=end)
    #     # Add second flight sequence
    #     seq_to_drop = astar_to_drop.a_star_search()
    #     for i in range(len(seq_to_drop)):
    #         seq_to_drop[i][2] = hover_height

    #     sequence.extend(seq_to_drop)

    #     # Add landing to flight sequence
    #     sequence.append((end[0], end[1], hover_height, 0.0))
    #     sequence.append((end[0], end[1], hover_height/2, 0.0))
    #     sequence.append((end[0], end[1], 0.0, 0.0))

    #     # reset_estimator(scf)
    #     # # start_position_printing(scf)
    #     # run_sequence(scf, sequence, 0.5)
