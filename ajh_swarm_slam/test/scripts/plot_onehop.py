# !/usr/bin/env python3

# python3 /home/lin/develop/ros/soslab_ws/src/slam/RCS-SLAM/ajh_swarm_slam/test/scripts/plot_onehop.py

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
############################################################
# only direct comm
############################################################
# ### old method -1
# # steps = [1, 2, 3, 4, 5, 6]
# # robot1_y = [0.00017941782078, 0.0618622446324, 0.0854175599776, 0.0943941464868, 0.0977661086383, 0.0989044837423]
# # noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# # robot2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1]
# # noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# # robot3_y = [2.19982058218, 2.13813775537, 2.11458244002, 2.10560585351, 2.10223389136, 2.10109551626]
# # noise3_y = [2.2, 2.2, 2.2, 2.2, 2.2, 2.2] 

# ### old method -2
# steps = [1, 2, 3, 4, 5, 6]
# robot1_y = [0.0001514727267, 0.0366826551696, 0.046471787217, 0.0490944186129, 0.0497950887622, 0.0499749485212]
# noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# robot2_y = [1.09995955995, 1.06342837751, 1.05363924546, 1.05101661406, 1.05031594391, 1.05013608415]
# noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# robot3_y = [2.19988896733, 1.91988896733, 1.63988896733, 1.35988896733, 1.07988896733, 0.799888967325]
# noise3_y = [2.2, 1.92, 1.64, 1.36, 1.08, 0.8] 

# ### new method-1
# # steps = [1, 2, 3, 4, 5, 6]
# # robot1_y = [0.000179417820536, 0.0618622445895, 0.085417559897, 0.0943941463835, 0.0977661085296, 0.0989044836499]
# # noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# # robot2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1 ]
# # noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# # robot3_y = [2.19982058218, 2.13813775541, 2.1145824401, 2.10560585362, 2.10223389147, 2.10109551635]
# # noise3_y = [2.2, 2.2, 2.2, 2.2, 2.2, 2.2] 

# # ### new method-2
# # steps = [1, 2, 3, 4, 5, 6]
# # robot1_y = [0.00017941782078, 0.0618622446324, 0.0854175599776, 
# #             0.0943941464868, 0.0977661086383, 0.0989044837423]
# # noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# # robot2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1]
# # noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# # robot3_y = [2.19982058218, 2.13813775537, 2.11458244002, 
# #             2.10560585351, 2.10223389136, 2.10109551626]
# # noise3_y = [2.2, 1.92, 1.64, 1.36, 1.08, 0.8] 

# ### new method-2 ?
# # steps = [1, 2, 3, 4, 5, 6]
# # robot1_y = [0.000151472726662, 0.0366826551634, 0.0464717872064, 
# #             0.0490944186004, 0.0497950887495, 0.0499749485103]
# # noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# # robot2_y = [1.09995955995, 1.06342837751, 1.05363924547, 
# #             1.05101661407, 1.05031594393, 1.05013608416]
# # noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# # robot3_y = [2.19988896733, 1.91988896733, 1.63988896733, 
# #             1.35988896733, 1.07988896733, 0.799888967325]
# # noise3_y = [2.2, 1.92, 1.64, 1.36, 1.08, 0.8] 


# fig = plt.figure(1, constrained_layout=True)
# ax = fig.add_subplot(111)
# plot_1 = ax.plot(steps, robot1_y, color="blue", label="robot_1_y")
# plot_2 = ax.plot(steps, robot2_y, color="green", label="robot_2_y")
# plot_3 = ax.plot(steps, robot3_y, color="red",   label="robot_3_y")
# plot_4 = ax.plot(steps, noise1_y, color="black",  linestyle='dashed')
# plot_5 = ax.plot(steps, noise2_y, color="black",  linestyle='dashed')
# plot_6 = ax.plot(steps, noise3_y, color="black",  linestyle='dashed')

# ax.set_xlabel('step', fontsize=15)
# ax.set_ylabel('odometry y-axis', fontsize=15)
# ax.legend(fontsize=15)
# ax.tick_params(axis='both', which='major', labelsize=15)
# ax.set_yticks(np.arange(0.0, 3.3, 1.1))
# ax.set_yticks(np.arange(0.0, 3.3, 0.1), minor=True)
# ax.grid(which='minor', alpha=0.2)
# ax.grid(which='major', alpha=0.5)
# plt.title("only direct comm", fontweight='bold', fontsize=24)
# plt.tight_layout()

############################################################
#  direct comm + one-hop
############################################################

### old method -2
# steps = [1, 2, 3, 4, 5, 6]
# robot1_y = [0.000356626967282, 0.0223831355706, 0.0176600276771, 
#             0.00557260151401, -0.00919647371127, -0.0273111076508]
# noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# robot2_y = [1.09994375142, 1.04911898865, 1.02501188858, 
#             1.00824514144, 0.994144066481, 0.983380561787]
# noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# robot3_y = [2.19969962161, 1.94849787787, 1.69732809001, 
#             1.44618226931, 1.19505242686, 0.943930573609]
# noise3_y = [2.2, 1.92, 1.64, 1.36, 1.08, 0.8] 

# ### new method-1
# steps = [1, 2, 3, 4, 5, 6]
# robot1_y = [0.000419455176057, 0.0792150840928, 0.0956616902953, 0.0990945331846, 0.0998111974508, 0.0999614864641]
# noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# robot2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1 ]
# noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# robot3_y = [2.19958054482, 2.12078491591, 2.1043383097, 2.10090546682, 2.10018880255, 2.10003851354]
# noise3_y = [2.2, 2.2, 2.2, 2.2, 2.2, 2.2] 

### new method-2
# steps = [1, 2, 3, 4, 5, 6]
# robot1_y = [0.000356626966714, 0.0223831350589, 0.0176600266372, 
#             0.00557259993509, -0.00919647584504, -0.0273111104007]
# noise1_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
# robot2_y = [1.09994375142, 1.04911898822, 1.02501188771, 
#             1.00824514012, 0.994144064702, 0.983380559612]
# noise2_y = [1.1, 1.1, 1.1, 1.1, 1.1, 1.1] 
# robot3_y = [2.19969962161, 1.94849787813, 1.69732808966, 
#             1.44618226768, 1.1950524235, 0.943930568273]
# noise3_y = [2.2, 1.92, 1.64, 1.36, 1.08, 0.8]
# agent 1~3:
# raw range:
# 2.2, 1.92, 1.64, 1.36, 1.08, 0.8
# optimized range:
# 2.199342995, 1.926114743, 1.679668063, 1.440609668, 1.204248899, 0.971241679

### new method-3- sim data
steps = [1, 2, 3, 4, 5, 6]
robot1_truth_y = np.array([0, 0, 0, 0, 0, 0])
noise1_noise_y = np.array([0, -0.953466, 0.619596, -0.106377, -0.188151, 0.0330163])
robot1_new_y = np.array([0.000246687826849, -0.731194705099,  0.646688189657, -0.117680031879, -0.183819395105, 0.0529876633085])
robot1_old_y = np.array([0.000250201812455, -0.728031051161, 0.662499514575, -0.0951275642589, -0.17696714668, 0.044135336085])
robot1_rmse_new = np.sqrt(np.mean((robot1_truth_y - robot1_new_y) ** 2))
robot1_rmse_old = np.sqrt(np.mean((robot1_truth_y - robot1_old_y) ** 2))
robot1_rmse_noise = np.sqrt(np.mean((robot1_truth_y - noise1_noise_y) ** 2))
print(f'robot1_rmse_new: {robot1_rmse_new}, robot1_rmse_old: {robot1_rmse_old}, robot1_rmse_noise: {robot1_rmse_noise}')

robot2_truth_y = np.array([0.8, 0.9, 0.8, 0.6, 0.8, 0.9])
noise2_noise_y = np.array([0.8, 0.357665, 0.78145, 0.799731, 0.253279, 0.455483])
robot2_new_y = np.array([0.800029813342, 0.384459355849, 0.764747540379, 0.739469492755, 0.149406349022, 0.209989806471])
robot2_old_y = np.array([0.800021493005, 0.376954512962, 0.727225413535, 0.671932283901, 0.0518645929771, 0.180432660406])
robot2_rmse_new = np.sqrt(np.mean((robot2_truth_y - robot2_new_y) ** 2))
robot2_rmse_old = np.sqrt(np.mean((robot2_truth_y - robot2_old_y) ** 2))
robot2_rmse_noise = np.sqrt(np.mean((robot2_truth_y - noise2_noise_y) ** 2))
print(f'robot2_rmse_new: {robot2_rmse_new}, robot2_rmse_old: {robot2_rmse_old}, robot2_rmse_noise: {robot2_rmse_noise}')

robot3_truth_y = np.array([1.75, 1.85, 1.7, 1.55, 1.35, 1.05])
noise3_noise_y = np.array([1.75, 1.81936, 1.50029, 0.756733, 1.14784, -1.15685])
robot3_new_y = np.array([1.74972349874, 1.57029844104, 1.48990310666, 0.828297966718, 1.24737661071, -0.931333134665])
robot3_old_y = np.array([1.74972830518, 1.57463973403, 1.51161412851, 0.873283084625, 1.33806665969, -0.8929229093])
robot3_rmse_new = np.sqrt(np.mean((robot3_truth_y - robot3_new_y) ** 2))
robot3_rmse_old = np.sqrt(np.mean((robot3_truth_y - robot3_old_y) ** 2))
robot3_rmse_noise = np.sqrt(np.mean((robot3_truth_y - noise3_noise_y) ** 2))
print(f'robot3_rmse_new: {robot3_rmse_new}, robot3_rmse_old: {robot3_rmse_old}, robot3_rmse_noise: {robot3_rmse_noise}')

fig = plt.figure(2, constrained_layout=True)
ax = fig.add_subplot(111)

ax.plot(steps, noise1_noise_y, color="blue")
ax.plot(steps, noise2_noise_y, color="green")
ax.plot(steps, noise3_noise_y, color="red")

ax.plot(steps, robot1_truth_y, color="blue",  linestyle='dashed')
ax.plot(steps, robot2_truth_y, color="green",  linestyle='dashed')
ax.plot(steps, robot3_truth_y, color="red",  linestyle='dashed')

ax.plot(steps, robot1_new_y, color="blue",  linestyle='dotted')
ax.plot(steps, robot2_new_y, color="green",  linestyle='dotted')
ax.plot(steps, robot3_new_y, color="red",  linestyle='dotted')

ax.plot(steps, robot1_old_y, color="blue",  linestyle='dashdot')
ax.plot(steps, robot2_old_y, color="green",  linestyle='dashdot')
ax.plot(steps, robot3_old_y, color="red",  linestyle='dashdot')

robot_legend_handles = [
    Line2D([0], [0], color="blue", label="robot1"),
    Line2D([0], [0], color="green", label="robot2"),
    Line2D([0], [0], color="red", label="robot3"),
]
type_legend_handles = [
    Line2D([0], [0], color="black", linestyle='solid', label="noise"),
    Line2D([0], [0], color="black", linestyle='dashed', label="truth"),
    Line2D([0], [0], color="black", linestyle='dotted', label="new"),
    Line2D([0], [0], color="black", linestyle='dashdot', label="old"),
]
# First legend (robot colors)
legend1 = ax.legend(handles=robot_legend_handles, loc="upper left", title="Robots", fontsize=15)
# Second legend (line styles)
legend2 = ax.legend(handles=type_legend_handles, loc="upper right", title="Data Type", fontsize=15)
# Add both legends to the plot
ax.add_artist(legend1)
ax.add_artist(legend2)
ax.set_xlabel('step', fontsize=15)
ax.set_ylabel('odometry y-axis', fontsize=15)
ax.tick_params(axis='both', which='major', labelsize=15)
ax.set_yticks(np.arange(0.0, 3.3, 1.0))
ax.set_yticks(np.arange(0.0, 3.3, 0.1), minor=True)
ax.grid(which='minor', alpha=0.2)
ax.grid(which='major', alpha=0.5)
plt.title("direct comm + one-hop", fontweight='bold', fontsize=24)
# plt.tight_layout()


plt.show()