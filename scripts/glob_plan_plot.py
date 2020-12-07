#!/usr/bin/env python


'''
Program to generate plots from rosbag2 data

plots generated:
-trajectory 
- x position in time
- y position in time
- theta angle in time
- absolute position error in time
- x and y position errors in time
- theta error in time
'''
import matplotlib.pyplot as plt
from os import listdir, walk, getcwd
from os.path import isfile, join
import numpy as np
import math
import sys
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset

import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    path = '../bags/global_plan/path1'
    dirs = next(walk(path))[1]
    dirs.sort()
    print(dirs)

    list_x = list()
    list_y = list()
    distances = []
    n_poses=[]
    for idx,dir in enumerate(dirs):

        mypath = f'{path}/{dir}'

        bag_files = [f for f in listdir(mypath) if f.endswith(".db3")]
        print(bag_files)

        for bag_file in bag_files:
            Dist = 0
            y_poses = list()
            x_poses = list()
            conn = sqlite3.connect(f'{mypath}/{bag_file}')
            c = conn.cursor()

            c.execute('SELECT * FROM messages')
            message_data = c.fetchall()

            message = message_data[0][3]
            msg_type = get_message('nav_msgs/msg/Path')



            x = deserialize_message(message, msg_type)
            for i,msg in enumerate(x.poses[:-1]):
                dx =x.poses[i+1].pose.position.x - msg.pose.position.x
                dy =x.poses[i+1].pose.position.y - msg.pose.position.y
                Dist = Dist + math.sqrt(dx*dx + dy*dy)
                x_poses.append(msg.pose.position.x)
                y_poses.append(msg.pose.position.y)
            x_poses.append(x.poses[-1].pose.position.x)
            y_poses.append(x.poses[-1].pose.position.y)

            list_x.append(x_poses)
            list_y.append(y_poses)
            n_poses.append(len(x.poses))
            distances.append(Dist)

    print(n_poses)
    print(distances)
    fig, ax = plt.subplots()
    plt.title(f'trajektoria')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    
    ax.plot(list_x[0],list_y[0], label='Djikstra')
    ax.plot(list_x[1],list_y[1], label='A*')


    # fig.legend()
    ax.legend(loc = 0)
    plt.grid()
    plt.savefig(f'{mypath}/trajectory.pdf')


    plt.close('all')

if __name__ == "__main__":
    main()
