import os
import cflib.crtp
import numpy as np
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

# if __name__ == "__main__":
# COMMENTED 3/9/23 above/below commenting out code to run crazyflie
    # cflib.crtp.init_drivers()

    # with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
# initializing the points that will be commanded


drop_loc = {
    "D1": (1.8, .6),
    "D2": (1.8, -0.3)
}

start_loc = {
    "S1": (-.6, 0.9),
    "S2": (-.6, .3),
    "S3":(-.6, -.3)
}

pick_loc = {
    "P1": (.4, .7),
    "P2": (.3, -.2),
    "P3": (1.0, .2),
    "P4": (.2, -.7),
}

class MultiPlanner():
    def __init__(self, drop_loc, start_loc, pick_loc):
        # Initialize Point of interest dictionaries
        self.drop_loc = drop_loc
        self.start_loc = start_loc
        self.pick_loc = pick_loc
        
        self.min_hover_height = 0.45

        # Hover buffer for drones
        self.hover_height = 0.1

        # z_level counter, increments with each run of create_multi_capable_path
        self.z_level = 0

        # Lighthouse origin relative to bottom left of map
        self.map_origin = (1, 1)

        map_dim_x = 3.5

        map_dim_y = 2    

        obstacles = [
            [(0, 0.3), (0, 0.605), (1.22, 0.605), (1.22, 0.3)],
            [(0, -0.3), (0, -0.605), (1.22, -0.605), (1.22, -0.3)]
        ]

        obstacle_list = [Obstacle(ob) for ob in obstacles]

        self.map = Map(
            origin=self.map_origin,
            dim_x=map_dim_x,
            dim_y=map_dim_y,
            obstacles=obstacle_list,
            drone_dim=0.1,
            prev_map = None
        )

    def framed_pos(self, position):
        """function to create the framed pos values taken by A*"""
        framed_position = (
            position[0]+self.map_origin[0],
            position[1]+self.map_origin[1]
        )
        return framed_position    

    def create_multi_capable_path(self, droneID, preprocessed_tasks, processed_map=None):
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

        
        # starting_point = "2"
        # hover_bin = "Bin 1a"
        # drop_loc = "B"



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
        for idx, locations in enumerate(task_loc[:-1]):
            # print(f"index of the task loc enumeration is {idx} which leads to {framed_pos(task_loc[idx])} when framed")

            print(f"Finding path between {self.framed_pos(task_loc[idx])} and {self.framed_pos(task_loc[idx+1])}")

            next_goal_astar = Astar(
                    map=self.map, start=self.framed_pos(task_loc[idx]),
                    target=self.framed_pos(task_loc[idx+1])
                    # map=map, start=start_map_frame, target=hover_point_map_frame
                )

            new_traj = next_goal_astar.a_star_search()
            
            if new_traj:
                print("adding traj: ", new_traj)
                seq_to_hover.extend(new_traj)
            
        if len(seq_to_hover)>1:
            print("Generated hover sequence")
        else:
            print("Failed to generate hover sequence")

        # Add hover flying sequence(accomplish goals)
        path_to_hover = np.array(self.map.sequence_to_path(seq=seq_to_hover))

        height = self.min_hover_height + self.hover_height * self.z_level
        self.z_level += 1
        
        HeightYaw_matrix = np.vstack([[height, 0.0]] * path_to_hover.shape[0])
        
        path_to_hover = np.append(path_to_hover, HeightYaw_matrix, axis=1)


        # Add Plotting sequence
        path_to_plot = np.array(seq_to_hover)

        Height_matrix = np.vstack([height] * path_to_plot.shape[0])

        path_to_plot = np.append(path_to_plot, Height_matrix, axis=1)

        print(path_to_plot)

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

        a_star_seq = path_to_plot

        world_seq = path_to_hover
        
        return world_seq, a_star_seq
    

# Initialize Multi Agent Planner Node with key locations and map
Planner = MultiPlanner(drop_loc, start_loc, pick_loc)

# Define tasks and agents. # of tasks must equal # of agents
task_strings = [["S1","P1","D1"], ["S2","P2","D2"]]
agents = ["kevin", "bob", 'charley']

# initializing sequence that will be sent
sequence = []
for i in range(len(task_strings)):
    seq, plot_seq = Planner.create_multi_capable_path(agents[i], task_strings[i])
    sequence.append(seq)

    Planner.map.visualize_map(
        path=plot_seq, waypoint_names=task_strings[i]
    )

plt.show()
print("Total hover sequence: ", sequence)

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
    #     f"Planned Crazyflie Path\nTakeoff Loc {task[0]}, Hover @ {task[1:-2]}, Drop Loc {task[-1]}"
    # )
    # plt.show()



# COMMENTED 3/9/23 commenting out code to run crazyflie
        # reset_estimator(scf)
        # start_position_printing(scf)
        # run_sequence(scf, sequence, 0.5)
