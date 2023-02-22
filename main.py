import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from fly_sequence import reset_estimator, run_sequence, start_position_printing

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/60/2M/E7E7E7E7E7')

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
    1: (1.8, -.3),
    2: (1.8, .6)
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

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        """
        Pseudocode:
        Create Obstacles from obstacle list
        Create map
        Run A* between start and hover location, set z pos to hover height, add to sequence
        Run A* between hover and drop location, set z pos to hover height (add landing), add to sequence
        Run sequence
        """

        reset_estimator(scf)
        # start_position_printing(scf)
        run_sequence(scf, sequence, 0.5)