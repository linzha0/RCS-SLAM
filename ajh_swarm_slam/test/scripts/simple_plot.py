# !/usr/bin/env python3

# python3 /home/lin/develop/ros/soslab_ws/src/slam/RCS-SLAM/ajh_swarm_slam/test/scripts/simple_plot.py

import matplotlib.pyplot as plt
import numpy as np

############################################################
# one inside odom: noise-odomy:1.0, truth-y:0.7
############################################################

steps = [1, 2, 3, 4, 5, 6]
noise_odom_y = [0.8, 1.2, 1.1, 1.0, 1.1, 1.2]
range_aided_y = [0.799704093481, 0.928041154628, 0.803180352434, 0.700924017619, 0.8007376801, 0.900737313969]
old_direct_comm_y = [0.79988688573, 1.09807274263, 0.994089582922, 0.883934583153, 0.973776332008, 1.03706965147]
new_direct_comm_y = [0.799879048962, 1.09101052743, 0.972893016489, 0.854771734771, 0.936647938979, 1.01852288624]
deltas = list(np.array(old_direct_comm_y) - np.array(new_direct_comm_y))

fig = plt.figure(1,constrained_layout=True)
ax = fig.add_subplot(111)
plot_1 = ax.plot(steps, noise_odom_y, color="black", label="noise_odom_y", linestyle='dashed')
plot_2 = ax.plot(steps, range_aided_y, color="green", label="range_aided_y", linestyle='dashed')
plot_3 = ax.plot(steps, old_direct_comm_y, color="red", label="old_direct_comm_y", linestyle='dashed')
plot_4 = ax.plot(steps, new_direct_comm_y, color="blue", label="new_direct_comm_y", linestyle='dashed')
for i, delta in enumerate(deltas):
    plt.text(1+i * 1, 0.5, "{:.3g}".format(delta), fontsize=14, ha='center', va='center')
    
# ax.locator_params(nbins=6)
ax.set_xlabel('step', fontsize=15)
ax.set_ylabel('odometry y-axis', fontsize=15)
ax.legend(fontsize=15)
ax.tick_params(axis='both', which='major', labelsize=15)
plt.yticks(np.arange(0.6, 1.2, 0.1))
plt.title("noise-odomy:1.0, truth-y:0.7", fontweight='bold', fontsize=24)
plt.grid()
plt.tight_layout()
# plt.savefig('filename_1.png', dpi=300)

############################################################
# one inside odom: noise-odomy:0.9, truth-y:0.6
############################################################

steps = [1, 2, 3, 4, 5, 6]
noise_odom_y = [0.8, 1.2, 1.1, 0.9, 1.1, 1.2]
range_aided_y = [0.799704093481, 0.928041154627, 0.803180352423, 0.600924017603, 0.800737680089, 0.900737313968]
old_direct_comm_y = [0.79988688573, 1.09807274263, 0.994089582928, 0.783934583243, 0.973776332, 1.03706965146]
new_direct_comm_y = [0.799879048962,1.09101052743, 0.972893016494, 0.754771734869, 0.936647938967, 1.01852288623]
deltas = list(np.array(old_direct_comm_y) - np.array(new_direct_comm_y))

fig = plt.figure(2,constrained_layout=True)
ax = fig.add_subplot(111)
plot_1 = ax.plot(steps, noise_odom_y, color="black", label="noise_odom_y", linestyle='dashed')
plot_2 = ax.plot(steps, range_aided_y, color="green", label="range_aided_y", linestyle='dashed')
plot_3 = ax.plot(steps, old_direct_comm_y, color="red", label="old_direct_comm_y", linestyle='dashed')
plot_4 = ax.plot(steps, new_direct_comm_y, color="blue", label="new_direct_comm_y", linestyle='dashed')
for i, delta in enumerate(deltas):
    plt.text(1+i * 1, 0.5, "{:.3g}".format(delta), fontsize=14, ha='center', va='center')

# ax.locator_params(nbins=6)
ax.set_xlabel('step', fontsize=15)
ax.set_ylabel('odometry y-axis', fontsize=15)
ax.legend(fontsize=15)
ax.tick_params(axis='both', which='major', labelsize=15)
plt.yticks(np.arange(0.6, 1.2, 0.1))
plt.title("noise-odomy:0.9, truth-y:0.6", fontweight='bold', fontsize=24)
plt.grid()
plt.tight_layout()
# plt.savefig('filename_1.png', dpi=300)

plt.show()
