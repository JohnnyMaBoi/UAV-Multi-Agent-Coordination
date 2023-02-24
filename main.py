import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from fly_sequence import reset_estimator, run_sequence, start_position_printing, get_pose
from utils import Obstacle, Map, Node

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E7E7')

sequence = []

obstacles = [
    [(0, .3), (0, .605), (1.22, .605), (1.22, .3)],
    [(0, -.3), (0, -.605), (1.22, -.605), (1.22, -.3)]
]

drop_locations = {
    'A': (-.6, -.6), 
    'B': (-.6, -.3),
    'C': (-.6, 0), 
    'D': (-.6, .3), 
    'E': (-.6, .6)
}

start_locations = {
    '1': (1.8, -.3),
    '2': (1.8, .6)
}

hover_locations = {
    'Bin 1a': (.15, .25),
    'Bin 2a': (.455, .25), 
    'Bin 3a': (.76, .25), 
    'Bin 4a': (1.065, .25),
    'Bin 1b': (.15, -.25), 
    'Bin 2b': (.455, -.25), 
    'Bin 3b': (.76, -.25), 
    'Bin 4b': (1.065, -.25)
}

hover_height = 0.45

# Lighthouse origin relative to bottom left of map
map_origin = (1, 1)

map_dim_x = 3.5
map_dim_y = 2

if __name__ == '__main__':

    # obstacle_list = [Obstacle(obstacles[i]) for i in range(len(obstacles))]
    # map = Map(origin=map_origin, dim_x=map_dim_x, dim_y=map_dim_y, obstacles=obstacle_list, drone_dim=0.1)
    # map.visualize_map()

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        """
        Pseudocode:
        Add takeoff to sequence
        Run A* between start and hover location, set z pos to hover height, add to sequence
        Run A* between hover and drop location, set z pos to hover height (add landing), add to sequence
        Add landing to sequence
        Run sequence
        """

        obstacle_list = [Obstacle(obstacles[i]) for i in range(len(obstacles))]
        map = Map(origin=map_origin, dim_x=map_dim_x, dim_y=map_dim_y, obstacles=obstacle_list, drone_dim=0.1)
        # map.visualize_map()
        
        # Select start, hover, & end points
        start = start_locations['1']
        hover_point = hover_locations['Bin 2a']
        end = drop_locations['B']

        # Add takeoff to sequence
        x, y, z = get_pose(scf)
        sequence.append((x, y, hover_height/2, 0.0))
        sequence.append((start[0], start[1], hover_height/2, 0.0))
        sequence.append((start[0], start[1], hover_height, 0.0))

        # Add first flight sequence
        seq_to_hover = map.a_star(start, hover_point)
        for i in range(len(seq_to_hover)):
            seq_to_hover[i][2] = hover_height

        sequence.extend(seq_to_hover)

        # Add second flight sequence
        seq_to_drop = map.a_star(hover_point, end)
        for i in range(len(seq_to_drop)):
            seq_to_drop[i][2] = hover_height
        
        sequence.extend(seq_to_drop)

        # Add landing to flight sequence
        sequence.append((end[0], end[1], hover_height, 0.0))
        sequence.append((end[0], end[1], hover_height/2, 0.0))
        sequence.append((end[0], end[1], 0.0, 0.0))

        # reset_estimator(scf)
        # # start_position_printing(scf)
        # run_sequence(scf, sequence, 0.5)