import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from fly_sequence import reset_estimator, run_sequence, start_position_printing, set_initial_position

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/60/2M/E7E7E7E7E7')

sequence = [
    (0.0, 0.0, 0.4, 0),
    (0.0, 0.0, 1.2, 0),
    (0.2, -0.2, 1.2, 0),
    (0.2, 0.2, 1.2, 0),
    (-0.2, 0.2, 1.2, 0),
    (-0.2, -0.2, 1.2, 0),
    (0.0, 0.0, 1.2, 0),
    (0.0, 0.0, 0.4, 0),
]

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # reset_estimator(scf)
        # run_sequence(scf, sequence, initial_yaw)
        start_position_printing(scf)