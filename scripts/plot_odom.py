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

    # correct for changing theta
    if(theta < math.pi):
        theta = math.pi*2 - theta
    return pos, theta


def main():
    path = '../bags/test'
    dirs = next(walk(path))[1]
    dirs.sort()
    print(dirs)

    avg_MSE_odom = 0
    avg_MSE_odom_filtered = 0
    avg_MSE_th = 0
    avg_MSE_th_filtered = 0

    plot_number=0
    for dir in dirs:
        mypath = f'{path}/{dir}'

        bag_files = [f for f in listdir(mypath) if f.endswith(".db3")]
        # print(bag_files)

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

            # print(msg_types)

            time0 = message_data[0][2]

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
                if th ==0:
                    dfs[topic_names[id]]['th'].append(dfs[topic_names[id]]['th'][-1])
                else:
                    dfs[topic_names[id]]['th'].append(th)
                dfs[topic_names[id]]['time'].append((row[2] - time0)/1e9)


            fig, ax = plt.subplots()
            plt.title(f'Trajektoria')
            plt.xlabel('x [m]')
            plt.ylabel('y [m]')
            ax.plot(dfs['/odom/noisy']['x'],dfs['/odom/noisy']['y'], label='odometria')
            ax.plot(dfs['/odometry/filtered']['x'],dfs['/odometry/filtered']['y'], label='odometria przefiltrowana')
            ax.plot(dfs['/omnivelma/pose']['x'],dfs['/omnivelma/pose']['y'], label='pozycja w symulatorze')
            # fig.legend()
            ax.legend(loc = 3)
            plt.grid()
            axins = zoomed_inset_axes(ax, 3, loc=1)
            axins.plot(dfs['/odom/noisy']['x'],dfs['/odom/noisy']['y'], label='odometria')
            axins.plot(dfs['/odometry/filtered']['x'],dfs['/odometry/filtered']['y'], label='odometria przefiltrowana')
            axins.plot(dfs['/omnivelma/pose']['x'],dfs['/omnivelma/pose']['y'], label='pozycja w symulatorze')
            x1, x2, y1, y2 = -0.5, 0.5, -0.5, 0.5 # specify the limits
            axins.set_xlim(x1, x2) # apply the x-limits
            axins.set_ylim(y1, y2) # apply the y-limits
            axins.grid()

            plt.yticks(visible=True)
            plt.xticks(visible=True)
            mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.5")
            plt.savefig(f'{mypath}/trajectory.pdf', bbox_inches='tight')




            plt.figure(2)
            plt.grid()
            plt.plot(dfs['/odom/noisy']['time'],dfs['/odom/noisy']['x'], label='odometria')
            plt.plot(dfs['/odometry/filtered']['time'],dfs['/odometry/filtered']['x'], label='odometria przefiltrowana')
            plt.plot(dfs['/omnivelma/pose']['time'],dfs['/omnivelma/pose']['x'], label='pozycja w symulatorze')
            plt.xlabel('czas [s]')
            plt.ylabel('x [m]')
            plt.legend()
            plt.title(f'Wartość położenia w osi x')
            plt.savefig(f'{mypath}/x.pdf', bbox_inches='tight')




            plt.figure(3)
            plt.grid()
            plt.plot(dfs['/odom/noisy']['time'],dfs['/odom/noisy']['y'], label='odometria')
            plt.plot(dfs['/odometry/filtered']['time'],dfs['/odometry/filtered']['y'], label='odometria przefiltrowana')
            plt.plot(dfs['/omnivelma/pose']['time'],dfs['/omnivelma/pose']['y'], label='pozycja w symulatorze')
            plt.xlabel('czas [s]')
            plt.ylabel('y [m]')
            plt.legend()
            plt.title(f'Wartość położenia w osi y')
            plt.savefig(f'{mypath}/y.pdf', bbox_inches='tight')





            plt.figure(4)
            plt.grid()
            plt.plot(dfs['/odom/noisy']['time'],dfs['/odom/noisy']['th'], label='odometria')
            plt.plot(dfs['/odometry/filtered']['time'],dfs['/odometry/filtered']['th'], label='odometria przefiltrowana')
            plt.plot(dfs['/omnivelma/pose']['time'],dfs['/omnivelma/pose']['th'], label='pozycja w symulatorze')
            plt.xlabel('czas [s]')
            plt.ylabel('kąt [rad]')
            plt.legend()
            plt.title(f'Wartość orientacji')
            plt.savefig(f'{mypath}/theta.pdf', bbox_inches='tight')




            plt.figure(5)
            plt.grid()

            odom_x_error = np.asarray(dfs['/error/odom']['x'])
            odom_y_error = np.asarray(dfs['/error/odom']['y'])
            odom_th_error = np.asarray(dfs['/error/odom']['th'])
            odom_error = np.sqrt(np.add(np.power(odom_x_error,2),np.power(odom_y_error,2)))

            odom_filtered_x_error = np.asarray(dfs['/error/odom_filtered']['x'])
            odom_filtered_y_error = np.asarray(dfs['/error/odom_filtered']['y'])
            odom_filtered_th_error = np.asarray(dfs['/error/odom_filtered']['th'])
            odom_filtered_error = np.sqrt(np.add(np.power(odom_filtered_x_error,2),np.power(odom_filtered_y_error,2)))

            MSE_odom = np.mean(np.power(odom_error,2))
            MSE_Th_odom = np.mean(np.power(odom_th_error,2))
            plt.plot(dfs['/error/odom']['time'], odom_error, label = 'błąd odometrii')


            MSE_odom_filtered = np.mean(np.power(odom_filtered_error,2))
            MSE_Th_odom_filtered = np.mean(np.power(odom_filtered_th_error,2))
            plt.plot(dfs['/error/odom_filtered']['time'], odom_filtered_error, label = 'błąd przefiltrowanej odometrii')


            avg_MSE_odom+=MSE_odom
            avg_MSE_odom_filtered+=MSE_odom_filtered
            avg_MSE_th+=MSE_Th_odom
            avg_MSE_th_filtered+=MSE_Th_odom_filtered
            print(f'MSE odom pos: {MSE_odom:.4} , theta: {MSE_Th_odom:.4}')
            print(f'MSE odom filtered pos: {MSE_odom_filtered:.4} , theta: {MSE_Th_odom_filtered:.4}')

            plt.xlabel('czas [s]')
            plt.ylabel('błąd [m]')
            plt.legend()
            plt.title(f'Absolutny błąd położenia')
            plt.savefig(f'{mypath}/error_abs.pdf',bbox_inches='tight')




            plt.figure(6)
            fig, axs = plt.subplots(2, sharex=True, sharey=False)
            axs[0].plot(dfs['/error/odom']['time'],dfs['/error/odom']['x'], label = 'odometria')
            axs[1].plot(dfs['/error/odom']['time'],dfs['/error/odom']['y'], label = 'odometria')
            axs[0].plot(dfs['/error/odom_filtered']['time'],dfs['/error/odom_filtered']['x'], label = 'przefiltrowana odometria')
            axs[1].plot(dfs['/error/odom_filtered']['time'],dfs['/error/odom_filtered']['y'], label = 'przefiltrowana odometria')

            plt.xlabel('time [s]')

            for ax in axs.flat:
                ax.set(ylabel='błąd [m]')
                ax.grid()
                ax.legend()
            
            axs[0].set_title('Błąd x')
            axs[1].set_title('Błąd y')

            plt.savefig(f'{mypath}/error.pdf',bbox_inches='tight')



            plt.figure(10)
            plt.plot(dfs['/error/odom']['time'],dfs['/error/odom']['th'], label = 'odometria')
            plt.plot(dfs['/error/odom_filtered']['time'],dfs['/error/odom_filtered']['th'], label = 'przefiltrowana odometria')
            plt.xlabel('time [s]')

            plt.grid()
            plt.legend()
            plt.title(f'Błąd orientacji')
            plt.savefig(f'{mypath}/error_theta.pdf',bbox_inches='tight')

            plt.close('all')
    
    avg_MSE_odom/=plot_number
    avg_MSE_odom_filtered/=plot_number
    avg_MSE_th/=plot_number
    avg_MSE_th_filtered/=plot_number
    print('avg_MSE_odom avg_MSE_odom_filtered avg_MSE_th avg_MSE_th_filtered')
    print(f'{avg_MSE_odom} {avg_MSE_odom_filtered} {avg_MSE_th} {avg_MSE_th_filtered}')

if __name__ == "__main__":
    main()
