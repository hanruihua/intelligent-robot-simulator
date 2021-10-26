import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos
import matplotlib as mpl
import imageio
from pathlib import Path
import platform

class env_multi_plot:

    def __init__(self, width = 10, height = 10, width_min = -1, height_min = -1, full= True):

        self.fig, self.ax = plt.subplots()
        self.width = width
        self.height = height
        self.width_min = width_min
        self.height_min = height_min

        self.ax.set_aspect('equal')
        self.ax.set_xlim(width_min, self.width)
        self.ax.set_ylim(height_min, self.height)
        
        if full:
            mode = platform.system()
            if mode == 'Linux':
                print(1)
                mng = plt.get_current_fig_manager()
                mng.resize(*mng.window.maxsize())

            elif mode == 'Windows':
                print(2)
                figManager = plt.get_current_fig_manager()
                figManager.window.showMaximized()


    def draw_robot_diff(self, robot):

        x = robot.state[0][0]
        y = robot.state[1][0]
        theta = robot.state[2][0]

        robot_circle = mpl.patches.Circle(xy=(x, y), radius = robot.radius, color = 'g')

        self.ax.add_patch(robot_circle)
        self.ax.arrow(x, y, 0.5*cos(theta), 0.5*sin(theta), head_width = 0.2)

        self.ax.set_xlim(self.width_min, self.width)
        self.ax.set_ylim(self.height_min, self.height)

    def draw_robot_diff_list(self, robot_list):

        for robot in robot_list:

            x = robot.state[0][0]
            y = robot.state[1][0]
            theta = robot.state[2][0]

            goal_x = robot.goal[0, 0]
            goal_y = robot.goal[1, 0]

            robot_circle = mpl.patches.Circle(xy=(x, y), radius = robot.radius, color = 'g')

            self.ax.add_patch(robot_circle)
            self.ax.arrow(x, y, 0.5*cos(theta), 0.5*sin(theta), head_width = 0.2)
            self.ax.plot(goal_x, goal_y, 'rx')
            self.ax.text(goal_x + 0.3, goal_y, 'g'+ str(robot.id), fontsize = 12, color = 'k')
            self.ax.text(x - 0.5, y, 'r'+ str(robot.id), fontsize = 10, color = 'k')

        self.ax.set_xlim(self.width_min, self.width)
        self.ax.set_ylim(self.height_min, self.height)

    def draw_obstacles(self, obstacle_state_list):

        for obs in obstacle_state_list:
            
            if obs[3] == 'sta_circular':

                x = obs[0]
                y = obs[1]
                radius = obs[2]

                obs_circle = mpl.patches.Circle(xy=(x, y), radius = radius, color = 'k')
                self.ax.add_patch(obs_circle)

    def draw_particles(self, particle_pose, robot_pose):
        # particle_pose: np.zeros((robot_num, particle_num, 2))
        
        for i in range(particle_pose.shape[0]):
            mean_pos = np.mean(particle_pose[i], axis = 0)

            x = mean_pos[0] + robot_pose[0]
            y = mean_pos[1] + robot_pose[1]
            
            # for j in range(particle_pose.shape[1]):
            #     x = particle_pose[i, j, 0]
            #     y = particle_pose[i, j, 1]

            self.ax.plot(x, y, 'ro')
            self.ax.text(x - 0.5, y, 'parti'+ str(i), fontsize = 10, color = 'k')

       
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

    def show(self):
        plt.show()

    def pause(self, time, mode='ubuntu'):
        plt.pause(time)

    def env_cla(self):
        plt.cla()