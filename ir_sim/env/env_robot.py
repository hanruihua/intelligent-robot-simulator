from ir_sim.world import mobile_robot
from math import pi, cos, sin
import numpy as np
from collections import namedtuple
from ir_sim.util import collision_cir_cir, collision_cir_matrix, collision_cir_seg

class env_robot:
    def __init__(self, robot_class=mobile_robot, robot_number=0, robot_mode='omni', robot_init_mode = 0, step_time=0.1, components=[], **kwargs):

        self.robot_class = robot_class
        self.robot_number = robot_number
        self.init_mode = robot_init_mode
        self.robot_list = []
        self.cur_mode = robot_init_mode
        self.com = components

        self.interval = kwargs.get('interval', 1)
        self.square = kwargs.get('square', [0, 0, 10, 10] )
        self.circular = kwargs.get('circular', [5, 5, 4] )
        self.random_bear = kwargs.get('random_bear', False)
        self.random_radius = kwargs.get('random_radius', False)


        # init_mode: 0 manually initialize
        #            1 single row
        #            2 random
        #            3 circular 
        # kwargs: random_bear random radius
        if self.robot_number > 0:
            if self.init_mode == 0:
                assert 'radius_list' and 'init_state_list' and 'goal_list' in kwargs.keys()
                radius_list = kwargs['radius_list']
                init_state_list = kwargs['init_state_list']
                goal_list = kwargs['goal_list']
            else:
                radius_list = kwargs.get('radius_list', [0.2])
                init_state_list, goal_list, radius_list = self.init_state_distribute(self.init_mode, radius=radius_list[0])

        # robot
        for i in range(self.robot_number):
            robot = self.robot_class(id=i, mode=robot_mode, radius=radius_list[i], init_state=init_state_list[i], goal=goal_list[i], step_time=step_time, **kwargs)
            self.robot_list.append(robot)
            self.robot = robot if i == 0 else None 
        
    def init_state_distribute(self, init_mode=1, radius=0.2):
        # init_mode: 1 single row
        #            2 random
        #            3 circular      
        # square area: x_min, y_min, x_max, y_max
        # circular area: x, y, radius
        
        num = self.robot_number
        state_list, goal_list = [], []

        if init_mode == 1:
             # single row
            state_list = [np.array([ [i * self.interval], [self.square[1]], [pi/2] ]) for i in range(int(self.square[0]), int(self.square[0])+num)]
            goal_list = [np.array([ [i * self.interval], [self.square[3]] ]) for i in range(int(self.square[0]), int(self.square[0])+num)]
            goal_list.reverse()

        elif init_mode == 2:
            # random
            state_list, goal_list = self.random_start_goal()

        elif init_mode == 3:
            # circular
            circle_point = np.array(self.circular)
            theta_step = 2*pi / num
            theta = 0

            while theta < 2*pi:
                state = circle_point + np.array([ cos(theta) * self.circular[2], sin(theta) * self.circular[2], theta + pi- self.circular[2] ])
                goal = circle_point[0:2] + np.array([cos(theta+pi), sin(theta+pi)]) * self.circular[2]
                theta = theta + theta_step
                state_list.append(state[:, np.newaxis])
                goal_list.append(goal[:, np.newaxis])

        elif init_mode == 4:
            # random 2
            circle_point = np.array(self.circular)
            theta_step = 2*pi / num
            theta = 0

            while theta < 2*pi:
                state = circle_point + np.array([ cos(theta) * self.circular[2], sin(theta) * self.circular[2], theta + pi- self.circular[2] ])
                goal = circle_point[0:2] + np.array([cos(theta+pi), sin(theta+pi)]) * self.circular[2]
                theta = theta + theta_step
                state_list.append(state[:, np.newaxis])
                goal_list.append(goal[:, np.newaxis])

        elif init_mode == 5:
            
            half_num = int(num /2)

            state_list1 = [np.array([ [i * self.interval], [self.square[1]], [pi/2] ]) for i in range(int(self.square[0]), int(self.square[0])+half_num)]

            state_list2 = [np.array([ [i * self.interval], [self.square[3]], [pi/2] ]) for i in range(int(self.square[0]), int(self.square[0])+half_num)]
            state_list2.reverse()
            
            goal_list1 = [np.array([ [i * self.interval], [self.square[3]], [pi/2] ]) for i in range(int(self.square[0]), int(self.square[0])+half_num)]
            goal_list1.reverse()

            goal_list2 = [np.array([ [i * self.interval], [self.square[1]], [pi/2] ]) for i in range(int(self.square[0]), int(self.square[0])+half_num)]
            
            state_list, goal_list = state_list1+state_list2, goal_list1+goal_list2
                    
        if self.random_bear:
            for state in state_list:
                state[2, 0] = np.random.uniform(low = -pi, high = pi)

        if self.random_radius:
            radius_list = np.random.uniform(low = 0.2, high = 1, size = (num,))
        else:
            radius_list = [radius for i in range(num)]

        return state_list, goal_list, radius_list
    
    def random_start_goal(self):

        num = self.robot_number
        random_list = []
        goal_list = []
        while len(random_list) < 2*num:

            new_point = np.random.uniform(low = self.square[0:2]+[-pi], high = self.square[2:4]+[pi], size = (1, 3)).T

            if not self.check_collision(new_point, random_list, self.com, self.interval):
                random_list.append(new_point)

        start_list = random_list[0 : num]
        goal_temp_list = random_list[num : 2 * num]

        for goal in goal_temp_list:
            goal_list.append(np.delete(goal, 2, 0))

        return start_list, goal_list
    
    def random_goal(self):

        num = self.robot_number
        random_list = []
        goal_list = []
        while len(random_list) < num:

            new_point = np.random.uniform(low = self.square[0:2]+[-pi], high = self.square[2:4]+[pi], size = (1, 3)).T

            if not self.check_collision(new_point, random_list, self.com, self.interval):
                random_list.append(new_point)

        goal_temp_list = random_list[:]

        for goal in goal_temp_list:
            goal_list.append(np.delete(goal, 2, 0))

        return goal_list

    def distance(self, point1, point2):
        diff = point2[0:2] - point1[0:2]
        return np.linalg.norm(diff)

    def check_collision(self, check_point, point_list, components, range):

        circle = namedtuple('circle', 'x y r')
        point = namedtuple('point', 'x y')
        self_circle = circle(check_point[0, 0], check_point[1, 0], range/2)

        for obs_cir in components['obs_circles'].obs_cir_list:
            temp_circle = circle(obs_cir.state[0, 0], obs_cir.state[1, 0], obs_cir.radius)
            if collision_cir_cir(self_circle, temp_circle):
                return True
        
        # check collision with map
        if collision_cir_matrix(self_circle, components['map_matrix'], components['xy_reso'], components['offset']):
            return True

        # check collision with line obstacles
        for line in components['obs_lines'].obs_line_states:
            segment = [point(line[0], line[1]), point(line[2], line[3])]
            if collision_cir_seg(self_circle, segment):
                return True

        for point in point_list:
            if self.distance(check_point, point) < range:
                return True
                
        return False


    def step(self, vel_list=[], **vel_kwargs):

        # vel_kwargs: vel_type = 'diff', 'omni'
        #             stop=True, whether stop when arrive at the goal
        #             noise=False, 
        #             alpha = [0.01, 0, 0, 0.01, 0, 0], noise for diff
        #             control_std = [0.01, 0.01], noise for omni

        for robot, vel in zip(self.robot_list, vel_list):
            robot.move_forward(vel, **vel_kwargs)

    def cal_des_list(self):
        vel_list = list(map(lambda x: x.cal_des_vel() , self.robot_list))
        return vel_list
    
    def cal_des_omni_list(self):
        vel_list = list(map(lambda x: x.cal_des_vel_omni() , self.robot_list))
        return vel_list

    def arrive_all(self):

        for robot in self.robot_list:
            if not robot.arrive():
                return False

        return True

    def robots_reset(self, reset_mode=1, **kwargs):
        
        if reset_mode == 0:
            for robot in self.robot_list:
                robot.reset(self.random_bear)
        
        elif self.cur_mode != reset_mode:
            state_list, goal_list, _ = self.init_state_distribute(init_mode = reset_mode)

            for i in range(self.robot_number):
                self.robot_list[i].init_state = state_list[i]
                self.robot_list[i].goal = goal_list[i]
                self.robot_list[i].reset(self.random_bear) 
            
            self.cur_mode = reset_mode

        elif reset_mode == 2:
            state_list, goal_list = self.random_start_goal()
            for i in range(self.robot_number):
                self.robot_list[i].init_state = state_list[i]
                self.robot_list[i].goal = goal_list[i]
                self.robot_list[i].reset(self.random_bear) 
        
        elif reset_mode == 4:
            goal_list = self.random_goal()
            for i in range(self.robot_number):
                self.robot_list[i].goal = goal_list[i]
                self.robot_list[i].reset(self.random_bear)

        else:
            for robot in self.robot_list:
                robot.reset(self.random_bear)

    def robot_reset(self, id=0):
        self.robot_list[id].reset(self.random_bear)

    def total_states(self):
        robot_state_list = list(map(lambda r: np.squeeze( r.omni_state()), self.robot_list))
        nei_state_list = list(map(lambda r: np.squeeze( r.omni_obs_state()), self.robot_list))
        obs_circular_list = list(map(lambda o: np.squeeze( o.omni_obs_state() ), self.com['obs_circles'].obs_cir_list))
        obs_line_list = self.com['obs_lines'].obs_line_states

        return [robot_state_list, nei_state_list, obs_circular_list, obs_line_list]
    # # states
    # def total_states(self, env_train=True):
        
    #     robot_state_list = list(map(lambda r: np.squeeze( r.omni_state(env_train)), self.robot_list))
    #     nei_state_list = list(map(lambda r: np.squeeze( r.omni_obs_state(env_train)), self.robot_list))
    #     obs_circular_list = list(map(lambda o: np.squeeze( o.omni_obs_state(env_train) ), self.obs_cir_list))
    #     obs_line_list = self.obs_line_list
        
    #     return [robot_state_list, nei_state_list, obs_circular_list, obs_line_list]
        
    # def render(self, time=0.1, save=False, path=None, i = 0, **kwargs):
        
    #     self.world_plot.draw_robot_diff_list(**kwargs)
    #     self.world_plot.draw_obs_cir_list()
    #     self.world_plot.pause(time)

    #     if save == True:
    #         self.world_plot.save_gif_figure(path, i)

    #     self.world_plot.com_cla()

    
    # def seg_dis(self, segment, point):
        
    #     point = np.squeeze(point[0:2])
    #     sp = np.array(segment[0:2])
    #     ep = np.array(segment[2:4])

    #     l2 = (ep - sp) @ (ep - sp)

    #     if (l2 == 0.0):
    #         return np.linalg.norm(point - sp)

    #     t = max(0, min(1, ((point-sp) @ (ep-sp)) / l2 ))

    #     projection = sp + t * (ep-sp)

    #     distance = np.linalg.norm(point - projection) 

    #     return distance