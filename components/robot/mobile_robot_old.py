import numpy as np
from math import sin, cos, atan2, pi, sqrt
from ir_env import motion_diff, motion_omni

class mobile_robot():

    def __init__(self, id, mode = 'diff', init_state=np.zeros((3, 1)), vel_diff = np.zeros((2, 1)), vel_omni = np.zeros((2, 1)), vel_max = 2 * np.ones((2, 1)), vel_ang_max=2, goal = np.zeros((2, 1)), goal_threshold = 0.2, radius = 0.2, collision_r = 0.1, step_time = 0.1, random_bear=False):

        # mode：'diff', differential robot
        #       'omni', omnidirectional robot

        assert init_state.shape==(3,1) and vel_max.shape==(2,1)

        self.id = id
        self.mode = mode  
        # init state
        self.init_state = init_state
        self.init_vel_diff = vel_diff
        self.init_vel_omni = vel_omni
        self.init_goal = goal

        if mode == 'diff':
            self.state = init_state
        elif mode == 'omni':
            self.state = init_state[0:2]

        self.vel_diff = vel_diff
        self.vel_omni = vel_omni
        self.vel_max = vel_max
        self.vel_ang_max = vel_ang_max
        self.goal = goal
        self.goal_threshold = goal_threshold
        self.radius = radius
        self.radius_collision = radius + collision_r
        self.step_time = step_time
        
        self.state_storage_x = []
        self.state_storage_y = []

        self.vel_storage = []
        self.random_bear = random_bear   # when reset or init the robot

    def update_info(self, state, vel):

        # update the information of the robot manually
        self.state = state
        self.vel = vel

    def diff2omni(self):

        vel_linear = self.vel_diff[0, 0]
        theta = self.state[2, 0]

        vx = vel_linear * cos(theta)
        vy = vel_linear * sin(theta)
        self.vel_omni = np.array([[vx], [vy]])

    def omni2diff(self, vel_omni, guarantee_time = 0.2, tolerance = 0.1):

        speed = np.sqrt(vel_omni[0, 0] ** 2 + vel_omni[1, 0] ** 2)
        vel_radians = atan2(vel_omni[1, 0], vel_omni[0, 0])
        
        w_max = self.vel_ang_max
        robot_radians = self.state[2, 0]
        diff_radians = robot_radians - vel_radians

        if diff_radians > pi:
            diff_radians = diff_radians - 2*pi
        elif diff_radians < -pi:
            diff_radians = diff_radians + 2*pi

        if diff_radians < tolerance and diff_radians > -tolerance:
            w = 0
        else:
            w = -diff_radians / guarantee_time
            if w > w_max:
                w = w_max
            
        v = speed * cos(diff_radians)

        if v<0:
            v = 0
        
        if speed == 0:
            v = 0
            w = 0

        vel_diff = np.array([[v], [w]])

        return vel_diff

    def move_forward(self, vel, vel_type = 'diff', stop=True, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0], control_std = [0.01, 0.01]):

        # default: robot mode: diff, no noise, vel_type: diff
        # vel_type: diff: np.array([[linear], [angular]])
        #           omni: np.array([[x], [y]])
        old_pose = self.state[0:2]
        
        if isinstance(vel, list):
            # if the velocity is a list, convert it to array 
            vel = np.array(vel, ndmin=2).T

        if vel.shape == (2,):
            vel = vel[:,np.newaxis]

        vel = np.clip(vel, -self.vel_max, self.vel_max)
        
        if stop and self.arrive():
            vel = np.zeros((2, 1))

        if self.mode == 'diff':
            if vel_type == 'diff':
                self.cur_acce = (np.linalg.norm(vel - self.vel_diff)) / self.step_time
                self.move_with_diff(vel, noise, alpha)

            elif vel_type == 'omni':
                self.cur_acce = (np.linalg.norm(vel - self.vel_omni)) / self.step_time
                self.move_from_omni(vel, noise, alpha) 

        elif self.mode == 'omni':
            self.cur_acce = (np.linalg.norm(vel - self.vel_omni)) / self.step_time
            self.move_with_omni(vel, noise, control_std)
        else:
            assert self.mode == 'omni' or self.mode == 'diff'

        cur_pos = self.state[0:2]
        
        self.dis_goal_inc = np.linalg.norm(cur_pos - self.goal)  - np.linalg.norm(old_pose - self.goal)

    def move_with_diff(self, vel_diff, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0]):
        
        next_state = motion_diff(self.state, vel_diff, self.step_time, noise, alpha)
        self.state = next_state
        self.vel_diff = vel_diff
        self.diff2omni()

    def move_from_omni(self, vel_omni, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0]):
        # vel_omni: np.array([[vx], [vy]])
        vel_diff = self.omni2diff(vel_omni)
        next_state = motion_diff(self.state, vel_diff, self.step_time, noise, alpha)
        self.state = next_state
        self.vel_diff = vel_diff
        self.vel_omni = vel_omni

    def move_with_omni(self, vel_omni, noise = False, control_std = [0.01, 0.01]):
         # vel_omni: np.array([[vx], [vy]])

        next_state = motion_omni(self.state, vel_omni, self.step_time, noise, control_std)
        
        self.state = next_state
        self.vel_omni = vel_omni

    def move_to_goal(self):
        vel = self.cal_des_vel()
        self.move_forward(vel)

    def if_goal(self, goal):
        
        position = self.state[0:2]

        dist = np.linalg.norm(position - goal) 

        if dist < self.radius:
            return True
        else:
            return False 

    def arrive(self):

        position = self.state[0:2]

        dist = np.linalg.norm(position - self.goal) 

        if dist < self.goal_threshold:
            return True
        else:
            return False 

    def cal_des_vel(self, tolerance = 0.12):

        if self.mode == 'diff':
            des_vel = self.cal_des_vel_diff(tolerance=tolerance)
        elif self.mode == 'omni':
            des_vel = self.cal_des_vel_omni()

        return des_vel

    def cal_des_vel_diff(self, tolerance = 0.12):

        dis, radian = mobile_robot.relative(self.state[0:2], self.goal)
        robot_radian = self.state[2, 0]

        w_opti = 0

        v_max = self.vel_max[0, 0]
        w_max = self.vel_max[1, 0]

        diff_radian = mobile_robot.to_pi( radian - robot_radian )

        if diff_radian > tolerance:
            w_opti = w_max
        
        elif diff_radian < - tolerance:
            w_opti = - w_max    

        if dis < self.goal_threshold:
            v_opti = 0
            w_opti = 0
        else:
            v_opti = v_max * cos(diff_radian)
            
            if v_opti < 0:
                v_opti = 0

        return np.array([[v_opti], [w_opti]])

    def cal_des_vel_omni(self):

        dis, radian = mobile_robot.relative(self.state[0:2], self.goal)

        if dis > self.goal_threshold:
            vx = self.vel_max[0, 0] * cos(radian)
            vy = self.vel_max[1, 0] * sin(radian)
        # elif dis > self.goal_threshold:
        #     vx = 0.7 * self.vel_max[0, 0] * cos(radian)
        #     vy = 0.7 * self.vel_max[1, 0] * sin(radian)
        else:
            vx = 0
            vy = 0

        return np.array([[vx], [vy]])

    def distance_des_vel(self):
        
        des_vel = self.cal_des_vel_omni()
        dis = np.linalg.norm(des_vel - self.vel_omni)
        
        return dis

    def distance_des_vel2(self, vel_omni):
        
        assert vel_omni.shape == (2, 1)

        des_vel = self.cal_des_vel_omni()
        diff = des_vel - vel_omni
        dis = np.linalg.norm(diff)

        # if des_vel[0, 0] == 0 and des_vel[1, 0] == 0 and dis < 0.2:
        #     dis = 0

        return dis

    def rel_angle_des_vel(self):
        # angle: 0 - pi
        theta = self.state[2]
        cur_vector = np.array([[cos(theta)], [sin(theta)]])
        des_vel = self.cal_des_vel_omni()

        if np.linalg.norm(des_vel) == 0:
            return 0

        cos_angle = (des_vel.T @ cur_vector) / ( np.linalg.norm(des_vel) * np.linalg.norm(cur_vector)  )
        angle = np.squeeze(np.arccos(cos_angle)) 

        return angle

    def omni_state(self):
        # summary of the current robot state 
        v_des = self.cal_des_vel_omni()
        rc_array = self.radius_collision * np.ones((1,1))

        return np.concatenate((self.state[0:2], self.vel_omni, rc_array, v_des), axis = 0)

    def train_state(self):

        dis_goal = np.linalg.norm(self.goal - self.state[0:2]) * np.ones((1,1))
        rc_array = self.radius_collision * np.ones((1,1))
        v_des = self.cal_des_vel_omni()

        return np.concatenate((dis_goal, v_des, rc_array), axis = 0)

    def omni_obs_state(self):
        # the states that can be observed by others
        rc_array = self.radius_collision * np.ones((1,1))
        return np.concatenate((self.state[0:2], self.vel_omni, rc_array), axis = 0)

    def omni_state_list(self):

        x = self.state[0, 0]
        y = self.state[1, 0]

        vx = self.vel_omni[0, 0]
        vy = self.vel_omni[1, 0]

        radius = self.radius_collision

        vx_des, vy_des = self.cal_des_vel_omni()

        return [x, y, vx, vy, radius, vx_des, vy_des]

    def omni_obs_state_list(self):

        x = self.state[0, 0]
        y = self.state[1, 0]

        vx = self.vel_omni[0, 0]
        vy = self.vel_omni[1, 0]

        radius = self.radius_collision

        return [x, y, vx, vy, radius]

    def heading_des(self):

        theta = self.state[2, 0]
        current_head = np.array([[cos(theta)], [sin(theta)]])
        des_head = self.goal-self.state[0:2]
        uni_des_head = des_head / np.linalg.norm(des_head)

        angle = np.arccos(current_head.T @ uni_des_head)   

        return angle


    def check_collision(self, position_list):
        # check collision with other robots 
        state = self.state[0:2]
        for position in position_list:
            if np.linalg.norm(position[0:2] - state) < self.radius * 2:
                return True

        return False

    def check_collision_range(self, position_list):
        # check collision with other robots 
        # if len(position_list)==0:
        #     return False
        
        state = self.state[0:2]
        for position in position_list:
            if np.linalg.norm(position[0:2] - state) < self.radius_collision * 2:
                return True

        return False
    
    def check_min_collision(self, position_list):

        if len(position_list) == 0:
            return np.inf

        state = self.state[0:2]
        range_list = list(map(lambda p: np.linalg.norm(p[0:2] - state), position_list))
        min_range = np.min(range_list)

        return min_range

    def check_collision_obs(self, obs_list):
        # check collision with various obstacles
        
        robot_position = self.state[0:2]
        for obs in obs_list:
            position = obs[0:2]
            obs_radius = obs[2]

            dis = np.linalg.norm(robot_position - position)

            if dis <  self.radius + obs_radius:
                return True
        
        return False

    def reset(self):
        self.state = self.init_state
        self.vel_diff = self.init_vel_diff
        self.vel_omni = self.init_vel_omni
        self.goal = self.init_goal

        if self.random_bear:
            start_bear = np.random.uniform(low = -pi, high = pi)
            self.state[2, 0] = start_bear

    def storage(self):
        self.state_storage_x.append(self.state[0, 0])
        self.state_storage_y.append(self.state[1, 0])
        self.vel_storage.append(self.vel_diff[0, 0])

    @staticmethod
    def relative(state1, state2):
        
        dif = state2 - state1

        dis = np.linalg.norm(dif)
        radian = atan2(dif[1, 0], dif[0, 0])
        
        return dis, radian

    @staticmethod
    def to_pi(radian):

        if radian > pi:
            radian = radian - 2 * pi
        elif radian < -pi:
            radian = radian + 2 * pi
        
        return radian