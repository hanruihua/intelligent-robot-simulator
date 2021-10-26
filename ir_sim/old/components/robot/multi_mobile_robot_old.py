import numpy as np
from ir_env import mobile_robot
from math import atan2, pi, sin, cos
import heapq

class multi_mobile_robot:

    def __init__(self, num_robot):

        self.num_robot = num_robot
        self.robot_list = []
        self.cur_position = []

    def __getitem__(self, robot_idx):
        return self.robot_list[robot_idx]

    def __setitem__(self, robot_idx, robot):
         self.robot_list[robot_idx] = robot

    # initialize
    def initialize(self, sg_kwargs=dict(), con_kwargs=dict()):
        
        # start_mode，interval，upper，lower
        # con_kwargs, robot_mode, step_time, radius, collision_r, goal_threshold

        start_goal = self.start_goal_list(**sg_kwargs)
        self.config_robots(*start_goal, **con_kwargs)
        
    def start_goal_list(self, start_mode = 0, interval = 1, upper = 8, lower = 0, right = 14, circle_point=[5, 5], radius=5, random_bear=False):
        # generate the list of start and goal
        num = self.num_robot
        start_bear_list = []

        if random_bear:
            start_bear_list = np.random.uniform(low = -pi, high = pi, size = (num, 1))
        else:
            start_bear_list = np.zeros((num, 1))

        if start_mode == 0:
            # double row
            half1 = num // 2
            half2 = num - half1

            start_list1 = [[i * interval, lower] for i in range(half1)]
            start_list2 = [[i * interval, upper] for i in range(half2)]
            start_list = start_list1 + start_list2

            goal_list1 = [[i * interval, upper] for i in range(half1)]
            goal_list2 = [[i * interval, lower] for i in range(half2)]

            goal_list1.reverse()
            goal_list2.reverse()

            goal_list = goal_list1 + goal_list2

        elif start_mode == 1:
            # single row
            start_list = [[i * interval, lower] for i in range(num)]
            goal_list = [[i * interval, upper] for i in range(num)]
            goal_list.reverse()

        elif start_mode == 2:
            # random
            start_list, goal_list, start_bear_list = self.random_start_goal(interval, upper, lower, right)

        elif start_mode == 3:
            
            theta_step = 2*pi / num
            theta = 0
            start_list = []
            goal_list = []

            if not random_bear:
                start_bear_list = []

            while theta < 2*pi:
                start_x = circle_point[0] + cos(theta) * radius
                start_y = circle_point[1] + sin(theta) * radius
                
                goal_x = circle_point[0] + cos(theta+pi) * radius
                goal_y = circle_point[1] + sin(theta+pi) * radius

                start_list.append([start_x, start_y])
                goal_list.append([goal_x, goal_y])

                if not random_bear:
                    start_bear_list.append([theta+pi])

                theta = theta + theta_step
        else:
            print('Please input the right mode number')


        return start_list, goal_list, start_bear_list

    def config_robots(self, start_list, goal_list, start_bear_list, robot_mode = 'diff', step_time=0.1, radius=0.2, collision_r = 0.1,goal_threshold=0.2, vel_max = 2*np.ones((2, 1)), random_bear=False ):

        for i in range(self.num_robot):

            robot_init_state = np.array([ [start_list[i][0]], [start_list[i][1]], [start_bear_list[i][0]] ])
            robot_goal = np.array([ [goal_list[i][0]], [goal_list[i][1]] ])

            robot = mobile_robot(i, robot_mode, init_state=robot_init_state, goal = robot_goal, step_time = step_time, radius = radius,collision_r=collision_r, goal_threshold=goal_threshold, vel_max = vel_max, random_bear=random_bear)

            self.robot_list.append(robot)
            self.cur_position.append(robot.state[0:2])

    def random_start_goal(self, interval = 1, upper = 8, lower = 0, right=14):

        num = self.num_robot
        random_list = []

        for i in range(2 * num):

            new_point = np.random.uniform(low = [lower, lower], high = [right, upper], size = (2,))

            while not multi_mobile_robot.check_collision(random_list, new_point, interval):
                new_point = np.random.uniform(low = [lower, lower], high = [right, upper], size = (2,))

            random_list.append(new_point)

        start_list = random_list[0 : num]
        goal_list = random_list[num : 2 * num]
        start_bear_list = np.random.uniform(low = -pi, high = pi, size = (num, 1))

        return start_list, goal_list, start_bear_list

    def random_init(self, interval = 1.5, upper = 8, lower = 0, right=14):

        start_list, goal_list, start_bear_list = self.random_start_goal(interval, upper, lower, right)
        
        for i in range(self.num_robot):

            robot_init_state = np.array([ [start_list[i][0]], [start_list[i][1]], [start_bear_list[i][0]] ])
            robot_init_goal = np.array([ [goal_list[i][0]], [goal_list[i][1]] ])

            self.robot_list[i].init_state = robot_init_state
            self.robot_list[i].init_goal = robot_init_goal

    # desired velocity
    def cal_des_vels(self):

        vel_list = list(map(lambda r: r.cal_des_vel(), self.robot_list))

        return vel_list

    def move_forward(self, vel_list, vel_type = 'diff',stop=False, noise_kwargs=dict()):

        cur_position = []
        
        for i in range(self.num_robot):
            self.robot_list[i].move_forward(vel_list[i], vel_type, stop, **noise_kwargs)
            cur_position.append(self.robot_list[i].state[0:2].copy())

        self.cur_position = cur_position

        return cur_position

    def total_states(self, neighbors_region=5, neighbors_num=5):
        
        total_multi_moving_list = list(map(lambda r: np.squeeze(r.omni_obs_state()), self.robot_list)) # all robot observed state
        robot_total_state_list = []
        for i in range(self.num_robot):

            robot_state = np.squeeze(self.robot_list[i].omni_state())  

            # calculate moving obstacles list 
            multi_moving_list = total_multi_moving_list[:]  
            del multi_moving_list[i]

            def rs_function(moving_state):
                return multi_mobile_robot.dis_point(moving_state, robot_state)

            filter_moving_list = list(filter(lambda m: rs_function(m) <= neighbors_region, multi_moving_list))

            if len(filter_moving_list) > neighbors_num:
                moving_list = heapq.nsmallest(neighbors_num, filter_moving_list, key=lambda m: rs_function(m))
            else:
                # moving_list = filter_moving_list[:]
                moving_list = sorted(filter_moving_list, key=lambda m: rs_function(m))
            
            # calculate statics obstacles list
            filter_obs_list = list(filter(lambda o: rs_function(o) <= neighbors_region, self.obstacles_list))

            robot_total_state_list.append([robot_state, moving_list, filter_obs_list])

        return robot_total_state_list

    def total_train_states(self, neighbors_region=5, neighbors_num=5):

        total_multi_moving_list = list(map(lambda r: np.squeeze(r.omni_obs_state()), self.robot_list))
        total_train_state_list = []

        for i in range(self.num_robot):

            robot_train_state = np.squeeze(self.robot_list[i].train_state()) 
            robot_omni_state = np.squeeze(self.robot_list[i].omni_state())  
            multi_moving_list = total_multi_moving_list[:]  
            del multi_moving_list[i]

            def rs_function(moving_state):
                return multi_mobile_robot.dis_point(moving_state, robot_omni_state)

            filter_moving_list = list(filter(lambda m: rs_function(m) <= neighbors_region, multi_moving_list))

            if len(filter_moving_list) > neighbors_num:
                moving_list = heapq.nsmallest(neighbors_num, filter_moving_list, key=lambda m: rs_function(m))
            else:
                moving_list = filter_moving_list[:]

            def rel_state(moving_state):  
                rel_moving_state = [0] * 5
                rel_moving_state[0:4] = moving_state[0:4] - robot_omni_state[0:4]
                rel_moving_state[4] = moving_state[4] + robot_omni_state[4]

                return rel_moving_state

            rel_moving_list = list(map(lambda mo: rel_state(mo), moving_list))
    
            total_train_state_list.append([robot_train_state, rel_moving_list]) 

        return total_train_state_list

    # check whether done
    def check_done(self):

        if self.multi_check_collision():
            return True

        if self.arrive():
            return True
        
        return False

    def check_done_list(self, mode = 'test'):

        # mode: 'test' or 'train'

        done_list = np.zeros((self.num_robot,))
        info_list = np.zeros((self.num_robot,))

        for i in range(self.num_robot):
            other_position_list = self.cur_position[:]
            del other_position_list[i]

            if mode == 'test':
                if self.robot_list[i].check_collision(other_position_list) == True:
                    done_list[i] = 1

            elif mode == 'train':
                if self.robot_list[i].check_collision_range(other_position_list) == True:
                    done_list[i] = 1
            
            if self.robot_list[i].arrive():
                info_list[i] = 1

            if self.robot_list[i].check_collision_obs(self.obstacles_list) == True:
                done_list[i] = 1
        
        return done_list, info_list

    def arrive(self):

        for robot in self.robot_list:
            if not robot.arrive():
                return False

        return True
    
    def multi_check_collision(self):

        for i in range(self.num_robot):
            other_position_list = self.cur_position[:]
            del other_position_list[i]

            if self.robot_list[i].check_collision(other_position_list) == True:
                return True
            if self.robot_list[i].check_collision_obs(self.obstacles_list) == True:
                return True
        
        return False

    # reset
    def multi_reset(self):

        for robot in self.robot_list:
            robot.reset()
    
    @staticmethod
    def relative(point1, point2):

        dis = np.sqrt( (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 )
        angle = atan2(point2[1] - point1[1], point2[0] - point1[0])

        return dis, angle

    def dis_point(point1, point2):
        return np.sqrt( (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2 )

    @staticmethod
    def check_collision(point_list, new_point, range):

        for point in point_list:
            dis, _ = multi_mobile_robot.relative(point, new_point)
            if dis < range:
                return False
            
        return True


