# !/usr/bin/env python

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

### Usages:
# python3 /home/lin/develop/ros/soslab_ws/src/slam/RCS-SLAM/ajh_swarm_slam/test/scripts/plot_all.py

class evaluateAllTrajectory:

    def __init__(self):
        self.truth_path = '/home/lin/develop/data/swarm/20bots/truth/'
        self.odom_path = '/home/lin/develop/data/swarm/20bots/odom/'
        self.new_est_path = '/home/lin/develop/data/swarm/20bots/new_est/'
        self.old_est_path = '/home/lin/develop/data/swarm/20bots/old_est/'
        self.range_est_path = '/home/lin/develop/data/swarm/20bots/range_est/'
        self.saved_path = '/home/lin/develop/data/swarm/20bots/results/'
        self.robot_num = 21 

        # self.truth_path = '/home/lin/develop/data/swarm/roomba/truth/'
        # self.odom_path = '/home/lin/develop/data/swarm/roomba/odom/'
        # self.new_est_path = '/home/lin/develop/data/swarm/roomba/new_est/'
        # self.old_est_path = '/home/lin/develop/data/swarm/roomba/old_est/'
        # self.range_est_path = '/home/lin/develop/data/swarm/roomba/range_est/'
        # self.saved_path = '/home/lin/develop/data/swarm/roomba/results/'
        # self.robot_num = 2 

        # self.truth_path     = '/home/lin/develop/data/swarm/sim/truth/'
        # self.odom_path      = '/home/lin/develop/data/swarm/sim/odom/'
        # self.new_est_path   = '/home/lin/develop/data/swarm/sim/new_est/'
        # self.old_est_path   = '/home/lin/develop/data/swarm/sim/old_est/'
        # self.range_est_path = '/home/lin/develop/data/swarm/sim/range_est/'
        # self.saved_path     = '/home/lin/develop/data/swarm/sim/results/'        
        # self.robot_num = 3 

        self.robots_truth = []
        self.robots_odom = []
        self.robots_new_est = []
        self.robots_old_est = []
        self.robots_range_est = []
        self.fig_count = 0

    def main(self):

        self.parse_arguments()    

        self.plot()

    def parse_arguments(self):
        """ 
            Load the arguments and parse the data
        """

        # =================================================================== #
        # Parse the truth data
        # =================================================================== #
        # for i in range(1, self.robot_num+1):
        #     ### Get the file
        #     filepath = self.truth_path + 'Robot'+ str(i) + '_filtered.csv'
        #     is_exist = os.path.isfile(filepath)
        #     if is_exist==False:
        #         print(f'file: {filepath}, exist: {"True" if is_exist else "False"}')
        #         continue

        #     ### Parse the data
        #     df = pd.read_csv(filepath, usecols=["Time", "pose.position.x", "pose.position.y"])
        #     timestamp =  list(df["Time"].astype(float))
        #     pos_x = list(df["pose.position.x"].astype(float))
        #     pos_y = list(df["pose.position.y"].astype(float))
        #     robot = [timestamp, pos_x, pos_y]
        #     self.robots_truth.append(robot)

        # =================================================================== #
        # Parse the odom data
        # =================================================================== #
        # for i in range(1, self.robot_num+1):
        #     ### Get the file
        #     filepath = self.odom_path + 'Robot'+ str(i) + '_filtered.csv'
        #     is_exist = os.path.isfile(filepath)
        #     if is_exist==False:
        #         print(f'file: {filepath}, exist: {"True" if is_exist else "False"}')
        #         continue

        #     ### Parse the data
        #     df = pd.read_csv(filepath, usecols=["Time", "pose.x", "pose.y"])
        #     timestamp =  list(df["Time"].astype(float))
        #     pos_x = list(df["pose.x"].astype(float))
        #     pos_y = list(df["pose.y"].astype(float))
        #     robot = [timestamp, pos_x, pos_y]
        #     self.robots_odom.append(robot)

        # =================================================================== #
        # Parse the new est data
        # =================================================================== #
        for i in range(1, self.robot_num+1):
            ### Get the file
            filepath = self.new_est_path + 'Robot'+ str(i) + '.csv'
            is_exist = os.path.isfile(filepath)
            if is_exist==False:
                print(f'file: {filepath}, exist: {"True" if is_exist else "False"}')
                continue

            ### Parse the data
            df = pd.read_csv(filepath, usecols=["x", "y"])
            pos_x = list(df["x"].astype(float))
            pos_y = list(df["y"].astype(float))
            robot = [pos_x, pos_y]
            self.robots_new_est.append(robot)

        # =================================================================== #
        # Parse the old est data
        # =================================================================== #
        for i in range(1, self.robot_num+1):
            ### Get the file
            filepath = self.old_est_path + 'Robot'+ str(i) + '.csv'
            is_exist = os.path.isfile(filepath)
            if is_exist==False:
                print(f'file: {filepath}, exist: {"True" if is_exist else "False"}')
                continue

            ### Parse the data
            df = pd.read_csv(filepath, usecols=["x", "y"])
            pos_x = list(df["x"].astype(float))
            pos_y = list(df["y"].astype(float))
            robot = [pos_x, pos_y]
            self.robots_old_est.append(robot)

        # =================================================================== #
        # Parse the range est data
        # =================================================================== #
        for i in range(1, self.robot_num+1):
            ### Get the file
            filepath = self.range_est_path + 'Robot'+ str(i) + '.csv'
            is_exist = os.path.isfile(filepath)
            if is_exist==False:
                print(f'file: {filepath}, exist: {"True" if is_exist else "False"}')
                continue

            ### Parse the data
            df = pd.read_csv(filepath, usecols=["x", "y"])
            pos_x = list(df["x"].astype(float))
            pos_y = list(df["y"].astype(float))
            robot = [pos_x, pos_y]
            self.robots_range_est.append(robot)

        if len(self.robots_truth) != len(self.robots_odom) != len(self.robots_range_est) != len(self.robots_new_est) != len(self.robots_old_est) != self.robot_num:
            print('robots number are not right')

    def plot_all(self):
        """
            Plot same trajecory for all the robots
        """

        # =================================================================== #
        # Plot all the truth data
        # =================================================================== #
        # self.fig_count += 1
        # fig = plt.figure(self.fig_count, constrained_layout=True)
        # ax = fig.add_subplot(111)
        # for i in range(len(self.robots_truth)):
        #     plot= ax.plot(self.robots_truth[i][1], self.robots_truth[i][2], linewidth=1, color="blue")
        #     # plot= ax.scatter(self.robots_truth[i][1], self.robots_truth[i][2], s=1, c="red")

        # ax.set_xlabel('X', fontsize=15)
        # ax.set_ylabel('Y', fontsize=15)
        # # ax.legend(fontsize=15)
        # plt.title("ALl robots truth trajectory", fontweight='bold', fontsize=24)
        # plt.grid()
        # plt.savefig(f'{self.saved_path}all_truth.png', dpi=300, bbox_inches='tight', transparent=False)

        # =================================================================== #
        # Plot all the odom data
        # =================================================================== #
        # self.fig_count += 1
        # fig = plt.figure(self.fig_count, constrained_layout=True)
        # ax = fig.add_subplot(111)
        # for i in range(len(self.robots_odom)):
        #     plot= ax.plot(self.robots_odom[i][1], self.robots_odom[i][2], linewidth=1)

        # ax.set_xlabel('X', fontsize=15)
        # ax.set_ylabel('Y', fontsize=15)
        # # ax.legend(fontsize=15)
        # plt.title("ALl robots odom trajectory", fontweight='bold', fontsize=24)
        # plt.grid()
        # plt.savefig(f'{self.saved_path}all_odom.png', dpi=300, bbox_inches='tight', transparent=False)

        # =================================================================== #
        # Plot all the odom range est data
        # =================================================================== #
        self.fig_count += 1
        fig = plt.figure(self.fig_count, constrained_layout=True)
        ax = fig.add_subplot(111)
        for i in range(len(self.robots_range_est)):
            plot= ax.plot(self.robots_range_est[i][0], self.robots_range_est[i][1], linewidth=1)

        ax.set_xlabel('X', fontsize=15)
        ax.set_ylabel('Y', fontsize=15)
        # ax.legend(fontsize=15)
        plt.title("All robots odom range est trajectory", fontweight='bold', fontsize=24)
        plt.grid()
        plt.savefig(f'{self.saved_path}all_range.png', dpi=300, bbox_inches='tight', transparent=False)

        # =================================================================== #
        # Plot all the odom new est data
        # =================================================================== #
        self.fig_count += 1
        fig = plt.figure(self.fig_count, constrained_layout=True)
        ax = fig.add_subplot(111)
        for i in range(len(self.robots_new_est)):
            plot= ax.plot(self.robots_new_est[i][0], self.robots_new_est[i][1], linewidth=1)

        ax.set_xlabel('X', fontsize=15)
        ax.set_ylabel('Y', fontsize=15)
        # ax.legend(fontsize=15)
        plt.title("All robots odom new est trajectory", fontweight='bold', fontsize=24)
        plt.grid()
        plt.savefig(f'{self.saved_path}all_new.png', dpi=300, bbox_inches='tight', transparent=False)

        # =================================================================== #
        # Plot all the odom old est data
        # =================================================================== #
        self.fig_count += 1
        fig = plt.figure(self.fig_count, constrained_layout=True)
        ax = fig.add_subplot(111)
        for i in range(len(self.robots_old_est)):
            plot= ax.plot(self.robots_old_est[i][0], self.robots_old_est[i][1], linewidth=1)

        ax.set_xlabel('X', fontsize=15)
        ax.set_ylabel('Y', fontsize=15)
        # ax.legend(fontsize=15)
        plt.title("All robots odom old est trajectory", fontweight='bold', fontsize=24)
        plt.grid()
        plt.savefig(f'{self.saved_path}all_old.png', dpi=300, bbox_inches='tight', transparent=False)

    def plot_ate(self):
        """
            Plot individual robot for GT, range-constraint, direct-comm, one-hop
        """

        for i in range(self.robot_num):

            # --------------------------------------------------------------- #
            # Plot 2D error
            # --------------------------------------------------------------- #

            ### For odom
            sync_truth, sync_odom = self.sync_data(self.robots_truth[i], self.robots_odom[i])
            ate_odom, rmse_odom = self.compute_ate(sync_truth, sync_odom)

            ### For new est
            sync_truth, sync_new_est = self.sync_data(self.robots_truth[i], self.robots_new_est[i])
            ate_new_est, rmse_new_est = self.compute_ate(sync_truth, sync_new_est)

            ### For old est
            sync_truth, sync_old_est = self.sync_data(self.robots_truth[i], self.robots_old_est[i])
            ate_old_est, rmse_old_est = self.compute_ate(sync_truth, sync_old_est)

            ### For range-aided
            sync_truth, sync_range_est = self.sync_data(self.robots_truth[i], self.robots_range_est[i])
            ate_range_est, rmse_range_est = self.compute_ate(sync_truth, sync_range_est)

            ### Plot
            self.fig_count += 1
            fig = plt.figure(self.fig_count, constrained_layout=True)            
            ax1 = fig.add_subplot(211)

            # ax1.scatter(list(range(len(ate_odom))), ate_odom, s=5, label='Odom')
            # ax1.scatter(list(range(len(ate_new_est))), ate_new_est, s=5, label='New Est')
            # ax1.scatter(list(range(len(ate_old_est))), ate_old_est, s=5, label='Old Est')
            # ax1.scatter(list(range(len(ate_range_est))), ate_range_est, s=5, label='Range Est')
            ax1.plot(list(range(len(ate_odom))), ate_odom, linewidth=5, label='Odom')
            ax1.plot(list(range(len(ate_new_est))), ate_new_est, linewidth=5, label='New Est')
            ax1.plot(list(range(len(ate_old_est))), ate_old_est, linewidth=5, label='Old Est')
            ax1.plot(list(range(len(ate_range_est))), ate_range_est, linewidth=5, label='Range Est')

            ax1.set_xlabel('step', fontsize=15)
            ax1.set_ylabel('ATE', fontsize=15)
            fig.suptitle(f'ATE: robot-{i+1}', fontweight='bold', fontsize=20)
            plt.legend()
            plt.grid()
            plt.savefig(f'{self.saved_path}robot_{i+1}_ate.png', dpi=300, bbox_inches='tight', transparent=False)

    def plot_xy(self):
        """
            Plot individual robot for GT, range-constraint, direct-comm, one-hop for x-axis and y-axis
        """

        for i in range(self.robot_num):
            
            # --------------------------------------------------------------- #
            # Plot X-Y tracjectory
            # --------------------------------------------------------------- #            
            ### Prepare the fig
            self.fig_count += 1
            fig = plt.figure(self.fig_count, constrained_layout=True)

            ax = fig.add_subplot(111)
            ### TODO: check the 'self.robots_truth[i][1]' size
            # plot_truth = ax.plot(
            #     self.robots_truth[i][1], self.robots_truth[i][2], linewidth=1, label='Truth')            
            # plot_odom = ax.plot(
            #     self.robots_odom[i][1], self.robots_odom[i][2], linewidth=1, label='Odom')
            plot_range = ax.plot(
                self.robots_range_est[i][0], self.robots_range_est[i][1], linewidth=1, label='Range')            
            plot_new = ax.plot(
                self.robots_new_est[i][0], self.robots_new_est[i][1], linewidth=1, label='New') 
            plot_old = ax.plot(
                self.robots_old_est[i][0], self.robots_old_est[i][1], linewidth=1, label='Old') 
                        
            ax.set_xlabel('x', fontsize=15)
            ax.set_ylabel('y', fontsize=15)
            ax.legend(fontsize=15)
            ### 
            fig.suptitle(f'X-Y trajectory: robot-{i+1}', fontweight='bold', fontsize=20)
            plt.grid()

    def plot_x_y(self):
        """
            Plot individual robot for GT, range-constraint, direct-comm, one-hop for x-axis and y-axis
        """
        for i in range(self.robot_num):
            
            # --------------------------------------------------------------- #
            # Plot X, Y axis
            # --------------------------------------------------------------- #
            ### Prepare the fig
            self.fig_count += 1
            fig = plt.figure(self.fig_count, constrained_layout=True)
            
            ### X-axis
            ax1 = fig.add_subplot(211)
            truth_x = ax1.plot(
                list(range(1, len(self.robots_truth[i][1]) + 1)), 
                self.robots_truth[i][1], linewidth=1, label='Truth')
            odom_x = ax1.plot(
                list(range(1, len(self.robots_odom[i][1]) + 1)), 
                self.robots_odom[i][1], linewidth=1, label='Odom')
            range_est_x = ax1.plot(
                list(range(1, len(self.robots_range_est[i][0]) + 1)), 
                self.robots_range_est[i][0], linewidth=1, label='Range Est')
            new_est_x = ax1.plot(
                list(range(1, len(self.robots_new_est[i][0]) + 1)), 
                self.robots_new_est[i][0], linewidth=1, label='New Est')
            old_est_x = ax1.plot(
                list(range(1, len(self.robots_old_est[i][0]) + 1)), 
                self.robots_old_est[i][0], linewidth=1, label='Old Est')
            ax1.set_xlabel('step', fontsize=15)
            ax1.set_ylabel('X', fontsize=15)
            ax1.legend(fontsize=15)

            ### Y-axis
            ax2 = fig.add_subplot(212)
            truth_y = ax2.plot(
                list(range(1, len(self.robots_truth[i][2]) + 1)), 
                self.robots_truth[i][2], linewidth=1, label='Truth')
            odom_y = ax2.plot(
                list(range(1, len(self.robots_odom[i][2]) + 1)), 
                self.robots_odom[i][2], linewidth=1, label='Odom')
            range_est_y = ax2.plot(
                list(range(1, len(self.robots_range_est[i][1]) + 1)), 
                self.robots_range_est[i][1], linewidth=1, label='Range Est')            
            new_est_y = ax2.plot(
                list(range(1, len(self.robots_new_est[i][1]) + 1)), 
                self.robots_new_est[i][1], linewidth=1, label='New Est')
            old_est_y = ax2.plot(
                list(range(1, len(self.robots_old_est[i][1]) + 1)), 
                self.robots_old_est[i][1], linewidth=1, label='Old Est')
            ax2.set_xlabel('step', fontsize=15)
            ax2.set_ylabel('Y', fontsize=15)
            ax2.legend(fontsize=15)
            ### 
            fig.suptitle(f'Truth-Odom trajectory: robot-{i+1}', fontweight='bold', fontsize=20)
            plt.grid()
            plt.savefig(f'{self.saved_path}robot_{i+1}_x_y.png', dpi=300, bbox_inches='tight', transparent=False)

    def plot(self):
        """
            Plot the data
        """

        # =================================================================== #
        # Plot all robots 
        # =================================================================== #

        # self.plot_all()

        # =================================================================== #
        # Plot individual robot
        # =================================================================== #

        ### Plot X-Y plane trajectory
        self.plot_xy()

        ### Plot error in X-axis and Y-axis separately
        # self.plot_x_y()

        ### Plot ATE
        # self.plot_ate()

        ### Plot
        plt.show()

    def compute_ate(self, S_truth, S_est):
        """
        Compute Absolute Trajectory Error (ATE) for each pose.

        Parameters:
        - S_truth: Ground truth trajectory, list or array of shape (N, 2) or (N, 3)
        - S_est: Estimated trajectory, list or array of shape (N, 2) or (N, 3)

        Returns:
        - ATEs: Array of ATE per pose of shape (N,)
        - rmse: Scalar RMSE value
        """
        ### Convert to np array
        if type(S_truth) == list:
            S_truth = np.array(S_truth)
        if type(S_est) == list:
            S_est = np.array(S_est)

        assert S_truth.shape == S_est.shape, "Trajectories must be of same shape"

        ate_per_pose = np.linalg.norm(S_truth - S_est, axis=1)
        rmse = np.sqrt(np.mean(ate_per_pose**2))

        return ate_per_pose, rmse
    
    def sync_data(self, truth, est):
        """
        sync the data: check data size
        """

        ### Check the size
        size_truth = len(truth[0])
        size_odom = len(est[0])
        sync_size = min(size_truth, size_odom)

        ### convert the data to pose   
        x_truth = [] 
        y_truth = [] 
        if len(truth) == 3:
            _, x_truth, y_truth = truth
        else:
            x_truth, y_truth = truth
        poses_truth = np.array(list(zip(x_truth, y_truth)))            

        x_est = []
        y_est = []
        if len(est) == 3:
            _, x_est, y_est = est
        else:
            x_est, y_est = est
        poses_est = np.array(list(zip(x_est, y_est)))

        ### Select the same size, assuming time are sync and tf are right
        return poses_truth[0:sync_size, :], poses_est[0:sync_size, :]

if __name__ == '__main__':
   object = evaluateAllTrajectory()
   object.main()          