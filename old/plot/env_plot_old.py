import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import imageio
from math import cos, sin
from pathlib import Path

class env_plot:
    def __init__(self, width, height, resolu = 1, keep = False):
    
        self.fig, self.ax = plt.subplots()
        self.width = width
        self.height = height
        self.keep = keep
        self.resolu = resolu
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1, self.width)
        self.ax.set_ylim(-1, self.height)

    def draw_robot_diff(self, robot, robot_color = 'g'):

        x = robot.state[0][0]
        y = robot.state[1][0]
        theta = robot.state[2][0]

        robot_circle = mpl.patches.Circle(xy=(x, y), radius = robot.radius, color = robot_color)

        self.ax.add_patch(robot_circle)
        self.ax.arrow(x, y, 0.5*cos(theta), 0.5*sin(theta), head_width = 0.2)

        self.ax.set_xlim(-1, self.width)
        self.ax.set_ylim(-1, self.height)
    
    def plot_trajectory(self, robot, num_estimator, label_name = [''], color_line=['b-']):

        self.ax.plot(robot.state_storage_x, robot.state_storage_y, 'g-', label='trajectory')

        for i in range(num_estimator):
            self.ax.plot(robot.estimator_storage_x[i], robot.estimator_storage_y[i], color_line[i], label = label_name[i])

        self.ax.legend()

    def plot_pre_tra(self, pre_traj):
        list_x = []
        list_y = []

        if pre_traj != None:
            for pre in pre_traj:
                list_x.append(pre[0, 0])
                list_y.append(pre[1, 0])
            
            self.ax.plot(list_x, list_y, '-b')
    
    def draw_obs_point(self, obs_point_list):

        for obs in obs_point_list:
            obs_circle = mpl.patches.Circle(xy=(obs[0, 0], obs[1, 0]), radius = obs[2, 0], color = 'k')
            self.ax.add_patch(obs_circle)

    def draw_goal(self, goal):
        self.ax.plot(goal[0, 0], goal[1, 0], 'g*')
    
    def draw_start(self, start):
        self.ax.plot(start[0, 0], start[1, 0], 'rx')

    def draw_path(self, path_x, path_y):
        self.ax.plot(path_x, path_y, 'g-')

    def draw_goal_path(self, path_x, path_y):
        self.ax.plot(path_x, path_y, 'r-')

    def draw_obs_matrix(self, obs_matrix):
        width = obs_matrix.shape[0]
        height = obs_matrix.shape[1]

        for i in range(width):
            for j in range(height):
                if obs_matrix[i, j] == 1:
                    self.ax.scatter(i * self.resolu, j * self.resolu, c='k')

    def draw_graph_search(self, cal_goal, visited, start, goal, save_figure = False):
        
        path_x = []
        path_y = []
        current_node = cal_goal

        self.ax.plot(start.x * self.resolu, start.y * self.resolu, 'gx')
        self.ax.plot(goal.x * self.resolu, goal.y * self.resolu, 'gx')
        i = 0
        for coordinate in visited:
            self.ax.scatter(coordinate[0]* self.resolu, coordinate[1] * self.resolu, c='r', marker='*')
            i = i+1
            if i%3 == 0:
                plt.pause(0.001)

                if save_figure == True:
                    self.save_gif_figure('./fig/', i)
            # i = i +1

        while 1:
            if current_node != None:

                path_x.append(current_node.x * self.resolu)
                path_y.append(current_node.y * self.resolu)
                current_node = current_node.parent
            else:
                break
        
        path_x.reverse()
        path_y.reverse()

        self.ax.plot(path_x, path_y, 'g-')

        # save figure
        if save_figure == True: 
            for j in range(30):
                self.save_gif_figure('./fig/', i+j)

        plt.show()

    def show(self):
        plt.show()

    def env_cla(self):
        plt.cla()

    def pause(self, time):
        plt.pause(time)

    def save_gif_figure(self, path, i):
        order = str(i).zfill(3)
        plt.savefig(path+order)

    def create_animate(self, path, keep_len=30):
        image_path = Path(path)
        images = list(image_path.glob('*.png'))
        images.sort()
        image_list = []
        for i, file_name in enumerate(images):

            if i == 0:
                continue

            image_list.append(imageio.imread(file_name))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(file_name))

        imageio.mimsave(path+'animated.gif', image_list)

        print('Create animation successfully')


