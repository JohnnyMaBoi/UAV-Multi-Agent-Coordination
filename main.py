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

# URI to the Crazyflie to connect to COMMENTED 3/9/23
# uri = uri_helper.uri_from_env(default="radio://0/60/2M/E7E7E7E7E7")

# initializing the points that will be commanded

obstacles = [
    [(0, 0.3), (0, 0.605), (1.22, 0.605), (1.22, 0.3)],
    [(0, -0.3), (0, -0.605), (1.22, -0.605), (1.22, -0.3)],
]

drop_loc = {
    "D1": (1.8, .6),
    "D2": (1.8, -0.3)
}

start_loc = {"S1": (-.6, 0.9), "S2": (-.6, .3), "S3":(-.6, -.3)}

pick_loc = {
    "P1": (.4, .7),
    "P2": (.3, -.2),
    "P3": (1.0, .2),
    "P4": (.2, -.7),
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

# this may cause problems because now the file is just a path generator
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


def framed_pos(position):
    """function to create the framed pos values taken by A*"""
    origin_tup = (1, 1)
    framed_position = (
        position[0]+origin_tup[0],
        position[1]+origin_tup[1]
    )
    return framed_position

# if __name__ == "__main__":
# COMMENTED 3/9/23 above/below commenting out code to run crazyflie
    # cflib.crtp.init_drivers()

    # with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:


def create_multi_capable_path(droneID, preprocessed_tasks, processed_map=None):
    """
    run A* alg for a drone that will output a new map with drone intersections and a commandable path

    args: 
        droneID (str): name that you are calling the drone in your code for consistency. Used 
            mostly in the node drone intersection tuple
        map (map class): map that A* is run in, has all attributes of map class
        preprocessed_tasks (list): list of the tasks expected of this drone, in order.
            note: this should BEGIN with the starting location of the drone, i.e. 1 

    returns: 
        map (map class): same map as original with new updated intersections. Will be passed in to the next
            time this func runs as the processed_map variable
        sequence (list?): list of points commanded to the drone in (x,y,z) tuples. Advances with timesteps
    """
        
    # Note: this script does NOT touch down or change altitude at all exc for very beginning and very end. 
    #     Didn't have time to do anything other than basic pathing

        # initializing sequence that will be sent
    sequence = []

    
    # starting_point = "2"
    # hover_bin = "Bin 1a"
    # drop_loc = "B"

    obstacle_list = [Obstacle(obstacles[i]) for i in range(len(obstacles))]
    map = Map(
        origin=map_origin,
        dim_x=map_dim_x,
        dim_y=map_dim_y,
        obstacles=obstacle_list,
        drone_dim=0.1,
        prev_map = processed_map
    )

    # debugging statements for map
    # print(f"the size of the map array is{(map.array[2][13])}")
    # map.visualize_map()

    # creating list to store the meters lighthouse-frame coords of each task in task list
    task_loc = []
    for strings in preprocessed_tasks:
        print(str(strings))
        if str(strings) in start_loc:
            task_loc.append((start_loc[strings]))
        if str(strings) in pick_loc:
            task_loc.append((pick_loc[strings]))
        if str(strings) in drop_loc:
            task_loc.append((drop_loc[strings]))
    
    print(f"the task_loc is {task_loc}")
    print(f"len of task loc for all except start is {len(task_loc[1:][:])}")

    # COMMENTED 3/9/23 moving towards func format, no more start and end
        # Select start, hover, & end points
        # start = start_loc[preprocesed_tasks[0]]
        # start_map_frame = (start[0] + map_origin[0], start[1] + map_origin[1])
        # hover_point = pick_loc[hover_bin]
        # hover_point_map_frame = (
        #     hover_point[0] + map_origin[0],
        #     hover_point[1] + map_origin[1],
        # )
        # end = drop_loc[preprocessed_tasks[-1]]
        # end_map_frame = (end[0] + map_origin[0], end[1] + map_origin[1])

        # Add takeoff to sequence
        # x, y, z = start[0], start[1], 0.0

        # COMMENTED 3/9/23 might cause problems, commenting any ref to crazyflie pose estimation for now.
        # x, y, z = get_pose(scf)
        # sequence.append((x, y, hover_height / 2, 0.0))


        # performing a* on all points that are not the first point in the task list bc its the start
        # needs to initialize and then populate astar with functions Astar and Astar.a_star_search
    
    # test the Astar
    # test_astar = Astar(
    #                 map=map, start=framed_pos((0,0)),
    #                 target=framed_pos(0,1))
    # print(f"test generated astar between known pts {test_astar.a_star_search()}")

    seq_to_hover = []
    for idx, locations in enumerate(task_loc[1:][:]):
        print(f"index of the task loc enumeration is {idx} which leads to {framed_pos(task_loc[idx][:])} when framed")
        if idx < len(task_loc[1:])-1:
            
            next_goal_astar = Astar(
                    map=map, start=framed_pos(task_loc[idx][:]),
                    target=framed_pos(task_loc[idx+1][:])
                    # map=map, start=start_map_frame, target=hover_point_map_frame
                )
            print(next_goal_astar.a_star_search())
                # !!! Kind of unsure about this datatype!
            seq_to_hover.append(next_goal_astar.a_star_search())
        else:
            break
    
    if len(seq_to_hover)>1:
        print("Generated hover sequence")
    else:
        print("Failed to generate hover sequence")

    # Add hover flying sequence(accomplish goals)
    path_to_hover = map.sequence_to_path(seq=seq_to_hover)
    for i, p in enumerate(path_to_hover):
        path_to_hover[i] = (p[0], p[1], hover_height, 0.0)

    sequence.append(path_to_hover)

        # COMMENTED BEFORE PREZ not working
        # Add takeoff and landing to flight sequence
        # sequence.insert(0, (path_to_hover[0][0], path_to_hover[0][1], hover_height / 2, 0.0))
        # sequence.insert(1, (path_to_hover[0][0], path_to_hover[0][1], hover_height, 0.0))

        # sequence.append((path_to_hover[-1][0], path_to_hover[-1][1], hover_height, 0.0))
        # sequence.append((path_to_hover[-1][0], path_to_hover[-1][1], hover_height / 2, 0.0))
        # sequence.append((path_to_hover[-1][0], path_to_hover[-1][1], 0.0, 0.0))


        # initializing the A* object to be searched
        # astar_to_drop = Astar(
        #     map=map, start=hover_point_map_frame, target=end_map_frame
        # )
        # seq_to_drop = astar_to_drop.a_star_search()
        # print("Generated drop sequence")

    # delay so that the drone stays at a certain point
        # sequence.extend([(hover_point[0], hover_point[1], hover_height, 0.0)] * 20)

        # # Add second flight sequence
        # path_to_drop = map.sequence_to_path(seq=seq_to_drop)
        # for i, p in enumerate(path_to_drop):
        #     path_to_drop[i] = (p[0], p[1], hover_height, 0.0)

        # sequence.extend(path_to_drop)


        # print(sequence)

    a_star_seq = seq_to_hover
    
    # map.visualize_map(
    #     # path=a_star_seq, waypoint_names=preprocessed_tasks
    # )

    # for s in sequence:
    #     a_star_log.write(f"{s[0]},{s[1]},{s[2]}\n")
    #     a_star_log.flush()

    # fig = plt.figure()
    # ax = plt.axes(projection="3d")
    # ax.scatter3D(
    #     [s[0] for s in sequence], [s[1] for s in sequence], [s[2] for s in sequence]
    # )
    # ax.set_xlim([-1, 2.5])
    # ax.set_ylim([-1, 1])
    # ax.set_zlim([0, 1])
    # ax.set_xlabel("x")
    # ax.set_ylabel("y")
    # ax.set_zlabel("z")
    # ax.set_title(
    #     f"Planned Crazyflie Path\nTakeoff Loc {preprocessed_tasks[0]}, Hover @ {preprocessed_tasks[1:-2]}, Drop Loc {preprocessed_tasks[-1]}"
    # )
    plt.show()
    return sequence, map

task_strings = ["S1","P1","D1"]
create_multi_capable_path("kevin", task_strings)


# COMMENTED 3/9/23 commenting out code to run crazyflie
        # reset_estimator(scf)
        # start_position_printing(scf)
        # run_sequence(scf, sequence, 0.5)
