from ir_sim.world import obs_circle
from math import pi, cos, sin
import numpy as np


class env_obs_cir:
    def __init__(self, obs_cir_class=obs_circle, num=1, init_mode = 0, step_time=0.1, **kwargs):

        self.obs_cir_class = obs_cir_class
        self.num = num
        self.init_mode = init_mode
        self.obs_cir_list = []
    
    def init_state_distribute(self, init_mode=1, interval=1, radius=0.2, square=[0, 0, 10, 10], circular=[5, 5, 4],  **kwargs):
        # init_mode: 1 single row
        #            2 random
        #            3 circular      
        # square area: x_min, y_min, x_max, y_max
        # circular area: x, y, radius
    
        
        random_radius = kwargs.get('random_radius', False)

        num = self.robot_num
        state_list, goal_list = [], []

        if init_mode == 1:
             # single row
            state_list = [np.array([ [i * interval], [square[1]], [pi/2] ]) for i in range(int(square[0]), int(square[0])+num)]
            goal_list = [np.array([ [i * interval], [square[3]] ]) for i in range(int(square[0]), int(square[0])+num)]
            goal_list.reverse()

        elif init_mode == 2:
            # random
            state_list, goal_list = self.random_start_goal(interval, square)

        elif init_mode == 3:
            # circular
            circle_point = np.array(circular)
            theta_step = 2*pi / num
            theta = 0

            while theta < 2*pi:
                state = circle_point + np.array([ cos(theta) * circular[2], sin(theta) * circular[2], theta + pi- circular[2] ])
                goal = circle_point[0:2] + np.array([cos(theta+pi), sin(theta+pi)]) * circular[2]
                theta = theta + theta_step
                state_list.append(state[:, np.newaxis])
                goal_list.append(goal[:, np.newaxis])

        elif init_mode == 4:
            # random 2
            circle_point = np.array(circular)
            theta_step = 2*pi / num
            theta = 0

            while theta < 2*pi:
                state = circle_point + np.array([ cos(theta) * circular[2], sin(theta) * circular[2], theta + pi- circular[2] ])
                goal = circle_point[0:2] + np.array([cos(theta+pi), sin(theta+pi)]) * circular[2]
                theta = theta + theta_step
                state_list.append(state[:, np.newaxis])
                goal_list.append(goal[:, np.newaxis])

        elif init_mode == 5:
            
            half_num = int(num /2)

            state_list1 = [np.array([ [i * interval], [square[1]], [pi/2] ]) for i in range(int(square[0]), int(square[0])+half_num)]

            state_list2 = [np.array([ [i * interval], [square[3]], [pi/2] ]) for i in range(int(square[0]), int(square[0])+half_num)]
            state_list2.reverse()
            
            goal_list1 = [np.array([ [i * interval], [square[3]], [pi/2] ]) for i in range(int(square[0]), int(square[0])+half_num)]
            goal_list1.reverse()

            goal_list2 = [np.array([ [i * interval], [square[1]], [pi/2] ]) for i in range(int(square[0]), int(square[0])+half_num)]
            
            state_list, goal_list = state_list1+state_list2, goal_list1+goal_list2
                    
        if self.random_bear:
            for state in state_list:
                state[2, 0] = np.random.uniform(low = -pi, high = pi)

        if random_radius:
            radius_list = np.random.uniform(low = 0.2, high = 1, size = (num,))
        else:
            radius_list = [radius for i in range(num)]

        return state_list, goal_list, radius_list
    
    def random_start_goal(self, interval = 1, square=[0, 0, 10, 10], **kwargs):

        num = self.robot_num
        random_list = []
        goal_list = []
        while len(random_list) < 2*num:

            new_point = np.random.uniform(low = square[0:2]+[-pi], high = square[2:4]+[pi], size = (1, 3)).T

            if not self.check_collision(new_point, random_list, self.obs_line_list, interval):
                random_list.append(new_point)

        start_list = random_list[0 : num]
        goal_temp_list = random_list[num : 2 * num]

        for goal in goal_temp_list:
            goal_list.append(np.delete(goal, 2, 0))

        return start_list, goal_list

    