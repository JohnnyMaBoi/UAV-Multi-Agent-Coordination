import os
import cflib.crtp
import matplotlib.pyplot as plt
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from fly_sequence import reset_estimator, run_sequence, get_pose
from map_objects import Obstacle, Map
from a_star import Astar

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default="radio://0/60/2M/E7E7E7E7E7")

# initializing the points that will be commanded
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

path_log = open(
    os.path.join(os.path.realpath(os.path.dirname(__file__)), "drone_path_log.csv"),
    "w+",
)
path_log.write("x,y,z\n")
path_log.flush()
a_star_log = open(
    os.path.join(os.path.realpath(os.path.dirname(__file__)), "a_star_path_log.csv"),
    "w+",
)
a_star_log.write("x,y,z\n")
a_star_log.flush()


def start_position_printing(scf, file=None):
    log_conf = LogConfig(name="Position", period_in_ms=500)
    log_conf.add_variable("kalman.stateX", "float")
    log_conf.add_variable("kalman.stateY", "float")
    log_conf.add_variable("kalman.stateZ", "float")

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def position_callback(timestamp, data, logconf):
    x = data["kalman.stateX"]
    y = data["kalman.stateY"]
    z = data["kalman.stateZ"]
    print("pos: ({}, {}, {})".format(x, y, z))
    path_log.write(f"{x},{y},{z}\n")
    path_log.flush()


if __name__ == "__main__":

    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:

        """
        Pseudocode:
        Add takeoff to sequence
        Run A* between start and hover location, set z pos to hover height, add to sequence
        Run A* between hover and drop location, set z pos to hover height (add landing), add to sequence
        Add landing to sequence
        Run sequence
        """

        starting_point = "2"
        hover_bin = "Bin 1a"
        drop_location = "B"

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
        start = start_locations[starting_point]
        start_map_frame = (start[0] + map_origin[0], start[1] + map_origin[1])
        hover_point = hover_locations[hover_bin]
        hover_point_map_frame = (
            hover_point[0] + map_origin[0],
            hover_point[1] + map_origin[1],
        )
        end = drop_locations[drop_location]
        end_map_frame = (end[0] + map_origin[0], end[1] + map_origin[1])

        # Add takeoff to sequence
        # x, y, z = start[0], start[1], 0.0
        x, y, z = get_pose(scf)
        sequence.append((x, y, hover_height / 2, 0.0))
        sequence.append((start[0], start[1], hover_height / 2, 0.0))
        sequence.append((start[0], start[1], hover_height, 0.0))

        astar_to_hover = Astar(
            map=map, start=start_map_frame, target=hover_point_map_frame
        )
        seq_to_hover = astar_to_hover.a_star_search()
        print("Generated hover sequence")

        # Add first flight sequence
        path_to_hover = map.sequence_to_path(seq=seq_to_hover)
        for i, p in enumerate(path_to_hover):
            path_to_hover[i] = (p[0], p[1], hover_height, 0.0)

        sequence.extend(path_to_hover)

        # initializing the A* object to be searched
        astar_to_drop = Astar(
            map=map, start=hover_point_map_frame, target=end_map_frame
        )
        seq_to_drop = astar_to_drop.a_star_search()
        print("Generated drop sequence")

        sequence.extend([(hover_point[0], hover_point[1], hover_height, 0.0)] * 20)

        # Add second flight sequence
        path_to_drop = map.sequence_to_path(seq=seq_to_drop)
        for i, p in enumerate(path_to_drop):
            path_to_drop[i] = (p[0], p[1], hover_height, 0.0)

        sequence.extend(path_to_drop)

        # Add landing to flight sequence
        sequence.append((end[0], end[1], hover_height, 0.0))
        sequence.append((end[0], end[1], hover_height / 2, 0.0))
        sequence.append((end[0], end[1], 0.0, 0.0))

        # print(sequence)

        a_star_seq = seq_to_hover + seq_to_drop
        map.visualize_map(
            path=a_star_seq, waypoint_names=[starting_point, hover_bin, drop_location]
        )

        for s in sequence:
            a_star_log.write(f"{s[0]},{s[1]},{s[2]}\n")
            a_star_log.flush()

        fig = plt.figure()
        ax = plt.axes(projection="3d")
        ax.scatter3D(
            [s[0] for s in sequence], [s[1] for s in sequence], [s[2] for s in sequence]
        )
        ax.set_xlim([-1, 2.5])
        ax.set_ylim([-1, 1])
        ax.set_zlim([0, 1])
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title(
            f"Planned Crazyflie Path\nTakeoff Loc {starting_point}, Hover @ {hover_bin}, Drop Loc {drop_location}"
        )
        plt.show()

        reset_estimator(scf)
        start_position_printing(scf)
        run_sequence(scf, sequence, 0.5)
