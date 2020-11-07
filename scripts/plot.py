 #!/usr/bin/env python
import matplotlib.pyplot as plt
import pandas as pd
from os import listdir, walk
from os.path import isfile, join
import numpy as np
import math
import sys
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
import tf2_ros

import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def get_pose(msg_types, message, msg_type_id):
        message_type = msg_types[msg_type_id]
        if message_type == 'nav_msgs/msg/Odometry':
            pos = message.pose.pose.position
            th = message.pose.pose.orientation
            theta = 2*math.acos(th.z)

        elif message_type == 'geometry_msgs/msg/PoseStamped' :
            pos = message.pose.position
            th = message.pose.orientation
            theta = 2*math.acos(th.z)
        elif message_type == 'geometry_msgs/msg/PoseWithCovarianceStamped':
            pos = message.pose.pose.position
            th = message.pose.pose.orientation
            theta = 2*math.acos(th.z)

        # correct for changing theta
        if(theta < math.pi):
            theta = math.pi*2 - theta
        return pos, theta

def main():
    conn = sqlite3.connect('rosbag2_2020_11_07-12_47_01_0.db3')
    c = conn.cursor()

    c.execute('SELECT * FROM messages')
    message_data = c.fetchall()

    c.execute('SELECT * FROM topics')
    topic_data = c.fetchall()
    msg_types = list()
    for row in topic_data:
        msg_types.append(row[2])

    print(msg_types)

    time0 = message_data[0][2]

    dfs = []

    for i in msg_types:
        dfs.append({k: [] for k in ['x','y','th', 'time']})

    for row in message_data:
        id = row[1] -1
        msg_type = get_message(msg_types[id])
        x = deserialize_message(row[3], msg_type)
        pos, th = get_pose(msg_types, x,id)
        dfs[id]['x'].append(pos.x)
        dfs[id]['y'].append(pos.y)
        dfs[id]['th'].append(th)
        dfs[id]['time'].append((row[2] - time0)/1e9)

        if id == 

    # fig, ax = plt.subplots()
    
    # plt.title(f'trajektoria')
    # plt.xlabel('x [m]')
    # plt.ylabel('y [m]')
    # ax.plot(dfs[2]['x'],dfs[2]['y'], label='perfect localization')
    # ax.plot(dfs[1]['x'],dfs[1]['y'], label='filtered localization')
    # ax.plot(dfs[0]['x'],dfs[0]['y'], label='real localization')
    # # ax.plot(dfs[3]['x'],dfs[3]['y'], label='AMCL localization')
    # fig.legend()
    # plt.grid()
    # axins = zoomed_inset_axes(ax, 3, loc=10)
    # axins.plot(dfs[2]['x'],dfs[2]['y'], label='perfect localization')
    # axins.plot(dfs[1]['x'],dfs[1]['y'], label='filtered localization')
    # axins.plot(dfs[0]['x'],dfs[0]['y'], label='real localization')
    # # axins.plot(dfs[3]['x'],dfs[3]['y'], label='AMCL localization')
    # x1, x2, y1, y2 = -0.15, 0.15, -0.15, 0.15 # specify the limits
    # axins.set_xlim(x1, x2) # apply the x-limits
    # axins.set_ylim(y1, y2) # apply the y-limits
    
    # plt.yticks(visible=False)
    # plt.xticks(visible=False)
    # mark_inset(ax, axins, loc1=1, loc2=3, fc="none", ec="0.5")
    # # plt.savefig(f'{mypath}/trajectory.pdf')

    plt.figure(2)
    plt.grid()
    plt.plot(dfs[2]['time'],dfs[2]['x'], label='perfect localization')
    plt.plot(dfs[1]['time'],dfs[1]['x'], label='filtered localization')
    plt.plot(dfs[0]['time'],dfs[0]['x'], label='real localization')
    plt.plot(dfs[3]['time'],dfs[3]['x'], label='AMCL localization')
    plt.xlabel('time [s]')
    plt.ylabel('x [m]')
    plt.legend()
    plt.title(f'x position value')
    plt.savefig(f'x.pdf')


    plt.figure(3)
    plt.grid()
    plt.plot(dfs[2]['time'],dfs[2]['y'], label='perfect localization')
    plt.plot(dfs[1]['time'],dfs[1]['y'], label='filtered localization')
    plt.plot(dfs[0]['time'],dfs[0]['y'], label='real localization')
    plt.plot(dfs[3]['time'],dfs[3]['y'], label='AMCL localization')
    plt.xlabel('time [s]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.title(f'y position value')
    plt.savefig(f'y.pdf')

    plt.figure(4)
    plt.grid()
    plt.plot(dfs[2]['time'],dfs[2]['th'], label='perfect localization')
    plt.plot(dfs[1]['time'],dfs[1]['th'], label='filtered localization')
    plt.plot(dfs[0]['time'],dfs[0]['th'], label='real localization')
    plt.plot(dfs[3]['time'],dfs[3]['th'], label='AMCL localization')
    plt.xlabel('time [s]')
    plt.ylabel('theta [rad]')
    plt.legend()
    plt.title(f'theta angle value')
    plt.savefig(f'theta.pdf')


#     dirs = next(walk('.'))[1]
#     dirs.sort()
#     print(dirs)
#     for dir in dirs:
#         mypath = f'./{dir}'

#         onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
#         print(onlyfiles)
#         onlyfiles.sort()

#         dfs= []

#         ## ['_slash_error.csv', '_slash_gazebo_odom.csv','_slash_odometry.csv']
#         for file in onlyfiles:
#             dfs.append(pd.read_csv(f'{mypath}/{file}'))

#         for df in dfs:
#             df.rosbagTimestamp = (df.rosbagTimestamp - df.rosbagTimestamp[0])/1e9
            
#         fig, ax = plt.subplots()
        
#         plt.title(f'trajektoria')
#         plt.xlabel('x [m]')
#         plt.ylabel('y [m]')
#         ax.plot(dfs[2].x,dfs[2].y, label='real localization')
#         ax.plot(dfs[1].x,dfs[1].y, label='perfect localization')
#         fig.legend()
#         plt.grid()
#         axins = zoomed_inset_axes(ax, 3, loc=10)
#         axins.plot(dfs[2].x,dfs[2].y, label='real localization')
#         axins.plot(dfs[1].x,dfs[1].y, label='perfect localization')
#         x1, x2, y1, y2 = -0.15, 0.15, -0.15, 0.15 # specify the limits
#         axins.set_xlim(x1, x2) # apply the x-limits
#         axins.set_ylim(y1, y2) # apply the y-limits
        
#         plt.yticks(visible=False)
#         plt.xticks(visible=False)
#         mark_inset(ax, axins, loc1=1, loc2=3, fc="none", ec="0.5")
#         plt.savefig(f'{mypath}/trajectory.pdf')

#         plt.figure(2)
#         plt.grid()
#         plt.plot(dfs[2].rosbagTimestamp,dfs[2].x, label='real localization')
#         plt.plot(dfs[1].rosbagTimestamp,dfs[1].x, label='perfect localization')
#         plt.xlabel('time [s]')
#         plt.ylabel('x [m]')
#         plt.legend()
#         plt.title(f'x position value')
#         plt.savefig(f'{mypath}/x.pdf')

#         plt.figure(3)
#         plt.grid()
#         plt.plot(dfs[2].rosbagTimestamp,dfs[2].y, label='real localization')
#         plt.plot(dfs[1].rosbagTimestamp,dfs[1].y, label='perfect localization')
#         plt.xlabel('time [s]')
#         plt.ylabel('y [m]')
#         plt.legend()
#         plt.title(f'y position value')
#         plt.savefig(f'{mypath}/y.pdf')

#         plt.figure(4)
#         plt.grid()
#         plt.plot(dfs[2].rosbagTimestamp,dfs[2].theta, label='real localization')
#         plt.plot(dfs[1].rosbagTimestamp,dfs[1].theta, label='perfect localization')
#         plt.xlabel('time [s]')
#         plt.ylabel('theta [rad]')
#         plt.legend()
#         plt.title(f'theta angle value')
#         plt.savefig(f'{mypath}/theta.pdf')

#         plt.figure(5)
#         plt.grid()
#         x_error = np.asarray(dfs[0].x)
#         y_error = np.asarray(dfs[0].y)
#         error = np.sqrt(np.add(np.power(x_error,2),np.power(y_error,2)))
#         plt.plot(dfs[0].rosbagTimestamp, error, label = 'error')
#         plt.xlabel('time [s]')
#         plt.ylabel('error [m]')
#         plt.legend()
#         plt.title(f'absolute position error \n final error: {error[-1]:.4}')
#         plt.savefig(f'{mypath}/error_abs.pdf')

#         plt.figure(6)
#         plt.grid()
#         plt.plot(dfs[0].rosbagTimestamp,dfs[0].x, label = 'error_x')
#         plt.plot(dfs[0].rosbagTimestamp,dfs[0].y, label = 'error_y')
#         plt.xlabel('time [s]')
#         plt.ylabel('error [m]')
#         plt.legend()
#         plt.title(f'x and y position errors \n final errors: x:{dfs[0].x.iloc[-1]:.4} y:{dfs[0].y.iloc[-1]:.4}')
#         plt.savefig(f'{mypath}/error.pdf')

#         plt.figure(7)
#         plt.grid()
#         plt.plot(dfs[0].rosbagTimestamp,dfs[0].theta, label='error')
#         plt.xlabel('time [s]')
#         plt.ylabel('error [rad]')
#         plt.legend()
#         plt.title(f'theta angle error \n final error: {dfs[0].theta.iloc[-1]:.4}')
#         plt.savefig(f'{mypath}/error_theta.pdf')

#         plt.close('all')

if __name__ == "__main__":
    main()