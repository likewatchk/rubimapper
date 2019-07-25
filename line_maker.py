import os
import model_predictive_trajectory_generator as planner
import motion_model
import pandas as pd
from matplotlib import pyplot as plt
import numpy as np
import math

table_path = os.path.dirname(os.path.abspath(__file__)) + "/lookuptable.csv"

def search_nearest_one_from_lookuptable(tx, ty, tyaw, lookup_table):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookup_table):

        dx = tx - table[0]
        dy = ty - table[1]
        dyaw = tyaw - table[2]
        d = math.sqrt(dx ** 2 + dy ** 2 + dyaw ** 2)
        if d <= mind:
            minid = i
            mind = d

    return lookup_table[minid]


def get_lookup_table():
    data = pd.read_csv(table_path)

    return np.array(data)


def generate_path(target_states, k0):
    # x, y, yaw, s, km, kf

    lookup_table = get_lookup_table()
    result = []

    for state in target_states:
        bestp = search_nearest_one_from_lookuptable(
            state[0], state[1], state[2], lookup_table)
        print("x,y,yaw,s,km,kf")
        print("bestp : ", bestp)
        target = motion_model.State(x=state[0], y=state[1], yaw=state[2])  # State class constructor
        init_p = np.array(
            [math.sqrt(state[0] ** 2 + state[1] ** 2), bestp[4], bestp[5]]).reshape(3, 1)  # 거리, s, km
        print("init_p : ", init_p)

        x, y, yaw, p = planner.optimize_trajectory(target, k0, init_p)

        if x is not None:
            print("find good path")
            result.append(
                [x[-1], y[-1], yaw[-1], float(p[0]), float(p[1]), float(p[2])])

    print("finish path generation")
    return result


def sampling_test_state():
    k0 = 0.0
    states = [[30, 10, np.deg2rad(50)]]
    result = generate_path(states, k0)
    xc, yc, yawc = motion_model.generate_trajectory(
        result[0][3], result[0][4], result[0][5], k0)
    if planner.show_animation:
        plt.plot(xc, yc, 'bo')
        plt.grid(True)
        plt.axis("equal")
        plt.show()

