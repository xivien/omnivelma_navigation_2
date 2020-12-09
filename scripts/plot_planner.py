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
import statistics
from os import listdir, walk
from os.path import isfile, join
import numpy as np
import math
import sys
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset

import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def get_pose(msg_types, topic_names, message, msg_type_id):
    message_type = msg_types[msg_type_id]
    topic_name = topic_names[msg_type_id]

    if topic_name.find('error') != -1:
        pos = message.pose.position
        theta = message.pose.orientation.z
        if abs(theta) > 1:
            theta = 0.0
        return pos, theta

    if message_type == 'nav_msgs/msg/Odometry':
        pos = message.pose.pose.position
        th = message.pose.pose.orientation
        theta = 2*math.acos(th.z)

    elif message_type == 'geometry_msgs/msg/PoseStamped':
        pos = message.pose.position
        th = message.pose.orientation
        theta = 2*math.acos(th.z)

    elif message_type == 'geometry_msgs/msg/PoseWithCovarianceStamped':
        pos = message.pose.pose.position
        th = message.pose.pose.orientation
        theta = 2*math.acos(th.z)

    elif message_type == 'geometry_msgs/msg/Twist':
        pos = message.linear
        theta = message.angular.z

    return pos, theta


def main():
    path = '../bags/TEB'
    dirs = next(walk(path))[1]
    dirs.sort()
    print(dirs)
    iter =0
    avg_x_vel = []
    avg_y_vel = []
    avg_th_vel = []
    list_x = list()
    list_y = list()
    plot_number=0
    for dir in dirs:
        mypath = f'{path}/{dir}'

        bag_files = [f for f in listdir(mypath) if f.endswith(".db3")]

        for bag_file in bag_files:
            plot_number+=1
            conn = sqlite3.connect(f'{mypath}/{bag_file}')
            c = conn.cursor()

            c.execute('SELECT * FROM messages')
            message_data = c.fetchall()

            c.execute('SELECT * FROM topics')
            topic_data = c.fetchall()
            msg_types = list()
            topic_names = list()
            for row in topic_data:
                msg_types.append(row[2])
                topic_names.append(row[1])

            time0 =message_data[0][2]

            dfs = {k: {} for k in topic_names}
            for df in dfs:
                dfs[df] = {k: [] for k in ['x', 'y', 'th', 'time']}

            for row in message_data:
                id = row[1] - 1
                msg_type = get_message(msg_types[id])
                x = deserialize_message(row[3], msg_type)
                pos, th = get_pose(msg_types, topic_names, x, id)
                dfs[topic_names[id]]['x'].append(pos.x)
                dfs[topic_names[id]]['y'].append(pos.y)
                dfs[topic_names[id]]['th'].append(th)
                dfs[topic_names[id]]['time'].append((row[2] - time0)/1e9)
            list_x.append(dfs['/amcl/filtered'])
            list_y.append(dfs['/cmd_vel'])
            iter+=1



    fig, ax = plt.subplots()
    plt.title(f'Trajektoria')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    ax.plot(list_x[0]['x'],list_x[0]['y'], label='DWB')
    ax.plot(list_x[1]['x'],list_x[1]['y'], label='TEB')
    # fig.legend()
    ax.legend(loc = 4)
    plt.grid()
    axins = zoomed_inset_axes(ax, 5, loc=2, borderpad=3.0)
    axins.plot(list_x[0]['x'],list_x[0]['y'], label='DWB')
    axins.plot(list_x[1]['x'],list_x[1]['y'], label='TEB')
    x1, x2, y1, y2 = 36, 40, 29, 33 # specify the limits
    axins.set_xlim(x1, x2) # apply the x-limits
    axins.set_ylim(y1, y2) # apply the y-limits
    axins.grid()

    plt.yticks(visible=True)
    plt.xticks(visible=True)
    mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.5")
    plt.savefig(f'{mypath}/trajectory.pdf', bbox_inches='tight')




    plt.figure(2)
    plt.grid()
    plt.plot(list_y[0]['time'],list_y[0]['x'], label='DWB')
    plt.plot(list_y[1]['time'],list_y[1]['x'], label='TEB')
    plt.xlabel('czas [s]')
    plt.ylabel('v_x [m/s]')
    plt.legend()
    plt.title(f'Zadana prędkość liniowa x')
    plt.savefig(f'{mypath}/x.pdf', bbox_inches='tight')




    plt.figure(3)
    plt.grid()
    plt.plot(list_y[0]['time'],list_y[0]['y'], label='DWB')
    plt.plot(list_y[1]['time'],list_y[1]['y'], label='TEB')
    plt.xlabel('czas [s]')
    plt.ylabel('v_y [m/s]')
    plt.legend()
    plt.title(f'Zadana prędkość liniowa y')
    plt.savefig(f'{mypath}/y.pdf', bbox_inches='tight')


    plt.figure(4)
    plt.grid()
    plt.plot(list_y[0]['time'],list_y[0]['th'], label='DWB')
    plt.plot(list_y[1]['time'],list_y[1]['th'], label='TEB')
    plt.xlabel('czas [s]')
    plt.ylabel('omega [rad/s]')
    plt.legend()
    plt.title(f'Zadana prędkość kątowa')
    plt.savefig(f'{mypath}/theta.pdf', bbox_inches='tight')

    plt.close('all')

    avg_x_TEB = statistics.mean(np.absolute(np.asarray(list_y[1]['x'])))
    avg_y_TEB = statistics.mean(np.absolute(np.asarray(list_y[1]['y'])))
    avg_th_TEB = statistics.mean(np.absolute(np.asarray(list_y[1]['th'])))
    time_TEB  = (list_y[1]['time'][-1] - list_y[1]['time'][0])

    avg_x_DWB = statistics.mean(np.absolute(np.asarray(list_y[0]['x'])))
    avg_y_DWB = statistics.mean(np.absolute(np.asarray(list_y[0]['y'])))
    avg_th_DWB = statistics.mean(np.absolute(np.asarray(list_y[0]['th'])))
    time_DWB = (list_y[0]['time'][-1]  - list_y[0]['time'][0])

    print(f"time TEB: {time_TEB} time DWB: {time_DWB}")
    print(f"x TEB: {avg_x_TEB} x DWB: {avg_x_DWB}")
    print(f"y TEB: {avg_y_TEB} y DWB: {avg_y_DWB}")
    print(f"th TEB: {avg_th_TEB} th DWB: {avg_th_DWB}")
    

if __name__ == "__main__":
    main()
