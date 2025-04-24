# !/usr/bin/env python

# python3 /home/lin/develop/ros/soslab_ws/src/slam/RCS-SLAM/ajh_swarm_slam/test/scripts/plot_timetsmap.py

import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

truth_path = '/home/lin/develop/data/swarm/roomba/truth/'
odom_path = '/home/lin/develop/data/swarm/roomba/odom/'
robot_num = 15
robots_truth = []
robots_odom = []
fig_count = 0

# =================================================================== #
# Parse the truth data
# =================================================================== #

for i in range(1, robot_num+1):
    ### Get the file
    filepath = truth_path + 'Robot'+ str(i) + '_filtered.csv'
    is_exist = os.path.isfile(filepath)
    if is_exist==False:
        print(f'file: {filepath}, exist: {"True" if is_exist else "False"}')
        continue

    ### Parse the data
    df = pd.read_csv(filepath, usecols=["Time", "pose.position.x", "pose.position.y"])
    timestamp =  list(df["Time"].astype(float))
    pos_x = list(df["pose.position.x"].astype(float))
    pos_y = list(df["pose.position.y"].astype(float))
    robot = [timestamp, pos_x, pos_y]
    robots_truth.append(robot)

# =================================================================== #
# Parse the odom data
# =================================================================== #
for i in range(1, robot_num+1):
    ### Get the file
    filepath = odom_path + 'Robot'+ str(i) + '_filtered.csv'
    is_exist = os.path.isfile(filepath)
    if is_exist==False:
        print(f'file: {filepath}, exist: {"True" if is_exist else "False"}')
        continue

    ### Parse the data
    df = pd.read_csv(filepath, usecols=["Time", "pose.x", "pose.y"])
    timestamp =  list(df["Time"].astype(float))
    pos_x = list(df["pose.x"].astype(float))
    pos_y = list(df["pose.y"].astype(float))
    robot = [timestamp, pos_x, pos_y]
    robots_odom.append(robot)    

# =================================================================== #
# Plot the time for truth
# =================================================================== #    
for i in range(len(robots_truth)):
    print(f'Turth robot:{i+1}, data size:{len(robots_truth[i][0])}, \
            time: [{robots_truth[i][0][0]}, {robots_truth[i][0][-1]}], \
            total duration: {robots_truth[i][0][-1] - robots_truth[i][0][0]}' )

    ### prepare the step and time
    step=[]
    timestamp = []
    for j in range(1, len(robots_truth[i][0])):
        timestamp.append(robots_truth[i][0][j] - robots_truth[i][0][j-1])
        step.append(j)

    ### Prepare the fig
    fig_count += 1
    fig = plt.figure(fig_count, constrained_layout=True)
    ax = fig.add_subplot(111)
    plot_truth = ax.scatter(step, timestamp)            
    ax.set_xlabel('timestamp', fontsize=15)
    ax.set_ylabel('timeoffset', fontsize=15)
    fig.suptitle(f'Truth timeoffset: robot-{i+1}', fontweight='bold', fontsize=20)

# =================================================================== #
# Plot the time for odom
# =================================================================== #    
for i in range(len(robots_odom)):
    print(f'Odom robot:{i+1}, data size:{len(robots_odom[i][0])}, \
            time: [{robots_odom[i][0][0]}, {robots_odom[i][0][-1]}], \
            total duration: {robots_odom[i][0][-1] - robots_odom[i][0][0]}' )

    ### prepare the step and time
    step = []
    timestamp = []
    for j in range(1, len(robots_odom[i][0])):
        timestamp.append(robots_odom[i][0][j] - robots_odom[i][0][j-1])
        step.append(j)

    # ### Prepare the fig
    # fig_count += 1
    # fig = plt.figure(fig_count, constrained_layout=True)
    # ax = fig.add_subplot(111)
    # plot_truth = ax.scatter(step, timestamp)            
    # ax.set_xlabel('timestamp', fontsize=15)
    # ax.set_ylabel('timeoffset', fontsize=15)
    # ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%.3f'))
    # fig.suptitle(f'Odom timeoffset: robot-{i+1}', fontweight='bold', fontsize=20)


plt.show()

