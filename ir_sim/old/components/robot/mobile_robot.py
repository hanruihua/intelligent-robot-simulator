import numpy as np
from math import sin, cos, atan2, pi
from ir_env import motion_diff, motion_omni
from queue import Queue

class mobile_robot():

    def __init__(self, id, mode = 'diff', init_state=np.zeros((3, 1)), vel_diff = np.zeros((2, 1)), vel_omni = np.zeros((2, 1)), vel_max = 2*np.ones((2, 1)), vel_ang_max=2, goal = np.zeros((2, 1)), goal_threshold = 0.2, radius = 0.2, step_time = 0.1, **kwargs):

        # mode：'diff', differential robot
        #       'omni', omnidirectional robot
        self.id = int(id)
        self.mode = mode  
        self.step_time = step_time

        if isinstance(init_state, list): 
            init_state = np.array(init_state, ndmin=2).T

        if isinstance(vel_diff, list): 
            vel_diff = np.array(vel_diff, ndmin=2).T

        if isinstance(vel_omni, list): 
            vel_omni = np.array(vel_omni, ndmin=2).T

        if isinstance(goal, list): 
            goal = np.array(goal, ndmin=2).T

        if isinstance(vel_max, list): 
            vel_max = np.array(vel_max, ndmin=2).T 
 
        if mode == 'diff':
            self.state = init_state
        elif mode == 'omni':
            self.state = init_state[0:2]

        self.init_state = init_state

        self.vel_diff = vel_diff
        self.vel_omni = vel_omni
        self.vel_max = vel_max
        self.vel_ang_max = vel_ang_max

        self.goal = goal
        self.goal_threshold = goal_threshold

        self.radius = radius

        self.radius_collision = round(radius + kwargs.get('radius_exp', 0.1), 2)
        self.arrive_flag = False

        self.state_queue = Queue(kwargs.get('state_queue_size', 5))

    def update_info(self, state, vel):

        # update the information of the robot manually
        self.state = state
        self.vel = vel

    def move_forward(self, vel, vel_type = 'diff', stop=True, noise=False, alpha = [0.01, 0, 0, 0.01, 0, 0], control_std = [0.01, 0.01]):

        # default: robot mode: diff, no noise, vel_type: diff
        # vel_type: diff: np.array([[linear], [angular]])
        #           omni: np.array([[x], [y]])
    
        if isinstance(vel, list): 
            vel = np.array(vel, ndmin=2).T

        if vel.shape == (2,):
            vel = vel[:,np.newaxis]

        vel = np.clip(vel, -self.vel_max, self.vel_max)
        
        if stop and self.arrive():
            vel = np.zeros((2, 1))

        if self.mode == 'diff':
            if vel_type == 'diff':
                self.move_with_diff(vel, noise, alpha)
            elif vel_type == 'omni':
                self.move_from_omni(vel, noise, alpha) 

        elif self.mode == 'omni':
            self.move_with_omni(vel, noise, control_std)
        else:
            assert self.mode == 'omni' or self.mode == 'diff'

        if not self.state_queue.full():
            self.state_queue.put(self.state)
        else:
            self.state_queue.get()
            self.state_queue.put(self.state)

    def move_with_diff(self, vel_diff, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0]):
        
        next_state = motion_diff(self.state, vel_diff, self.step_time, noise, alpha)
        self.state = next_state
        self.vel_diff = vel_diff
        self.diff2omni()

    def diff2omni(self):

        vel_linear = self.vel_diff[0, 0]
        theta = self.state[2, 0]

        vx = vel_linear * cos(theta)
        vy = vel_linear * sin(theta)
        self.vel_omni = np.array([[vx], [vy]])

    def move_from_omni(self, vel_omni, noise = False, alpha = [0.01, 0, 0, 0.01, 0, 0]):
        # vel_omni: np.array([[vx], [vy]])
        vel_diff = np.round(self.omni2diff(vel_omni), 2)
        next_state = motion_diff(self.state, vel_diff, self.step_time, noise, alpha)
        self.state = next_state
        self.vel_diff = vel_diff
        self.diff2omni()
        
    def omni2diff(self, vel_omni, guarantee_time = 0.1, tolerance = 0.1):

        speed = np.sqrt(vel_omni[0, 0] ** 2 + vel_omni[1, 0] ** 2)

        if speed > self.vel_max[0, 0]:
            speed = self.vel_max[0, 0]

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
        
        if speed <= 0.02:
            v = 0
            w = 0

        vel_diff = np.array([[v], [w]])

        return vel_diff

    def move_with_omni(self, vel_omni, noise = False, control_std = [0.01, 0.01]):
        # vel_omni: np.array([[vx], [vy]])
        next_state = motion_omni(self.state, vel_omni, self.step_time, noise, control_std)
        
        self.state = next_state
        self.vel_omni = vel_omni

    def move_to_goal(self):
        vel = self.cal_des_vel()
        self.move_forward(vel)

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
        else:
            vx = 0
            vy = 0

        return np.array([[vx], [vy]])

    def if_goal(self, goal):
        
        position = self.state[0:2]

        dist = np.linalg.norm(position - goal) 

        if dist < self.radius:
            return True
        else:
            return False 

    def arrive(self):

        position = self.state[0:2]

        dist = np.linalg.norm(position - self.goal[0:2]) 

        if dist < self.goal_threshold:
            return True
        else:
            return False 

    def reset(self, random_bear=False):
        self.state[:] = self.init_state[:]
        self.vel_omni = np.zeros((2, 1))
        self.vel_diff = np.zeros((2, 1))
        self.arrive_flag = False

        if random_bear:
            self.state[2, 0] = np.random.uniform(low = -pi, high = pi)

    # state
    # def omni_state(self, exp=True):
    #     v_des = self.cal_des_vel_omni()
    #     if exp:
    #         rc_array = self.radius_collision * np.ones((1,1))
    #     else:
    #         rc_array = self.radius* np.ones((1,1))

    #     return np.concatenate((self.state[0:2], self.vel_omni, rc_array, v_des, self.id*np.ones((1,1))), axis = 0)

    def omni_state(self, exp=True):
        v_des = self.cal_des_vel_omni()
        rc_array = self.radius_collision * np.ones((1,1))
            
        return np.concatenate((self.state[0:2], self.vel_omni, rc_array, v_des), axis = 0)

    def omni_obs_state(self, exp=True):
        # if exp:
        #     rc_array = self.radius_collision * np.ones((1,1))
        # else:
        rc_array = self.radius * np.ones((1,1))
        # rc_array = self.radius_collision * np.ones((1,1))
            
        return np.concatenate((self.state[0:2], self.vel_omni, rc_array), axis = 0)

    @staticmethod
    def relative(state1, state2):
        
        dif = state2[0:2] - state1[0:2]

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