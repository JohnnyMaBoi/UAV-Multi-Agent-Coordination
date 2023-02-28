import os
import matplotlib.pyplot as plt
import pandas as pd

if __name__ == "__main__":
    drone_path = pd.read_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)), "drone_path_log.csv"))
    a_star_path = pd.read_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)), "a_star_path_log.csv"))
    
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.scatter3D(a_star_path['x'], a_star_path['y'], a_star_path['z'])
    ax.scatter3D(drone_path['x'], drone_path['y'], drone_path['z'])
    ax.set_xlim([-1, 2.5])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_aspect('equal')
    ax.legend(['Planned Path', 'Actual Path'])
    ax.set_title("Crazyflie Planned Trajectory vs. Actual Trajectory")
    plt.show()
    