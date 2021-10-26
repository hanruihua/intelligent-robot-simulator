import numpy as np
from math import pi, sin, cos, tan, atan2
from ir_sim.world import motion_ackermann

class car_robot:
    def __init__(self, id=0, shape = [1.5, 1, 1, 1], init_state=np.zeros((4, 1)), goal = np.zeros((3, 1)), goal_threshold = 0.2, limit=[2, 2], psi_limit = pi/4, step_time=0.1, **kwargs):

        # state: 0, x
        #        1, y
        #        2, phi, heading direction
        #        3, psi, steering angle
        # shape: length, width, wheelbase, wheelbase_w
        # limit: vel_limit, vel_ang_limit

        if isinstance(init_state, list): 
            init_state = np.array(init_state, ndmin=2).T

        if isinstance(goal, list): 
            goal = np.array(goal, ndmin=2).T

        self.id=id

        self.shape = shape
        self.length = shape[0]
        self.width = shape[1]
        self.wheelbase=shape[2]
        self.wheelbase_w= shape[3]
        
        self.v_l = limit[0]
        self.w_l = limit[1]
        self.psi_limit = psi_limit

        self.min_radius= self.wheelbase / tan(psi_limit)
    
        self.state=init_state
        self.angular_pos()

        self.goal=goal
        self.goal_th = goal_threshold
        self.pre_state = init_state
        self.vel = np.zeros((2, 1))

        self.arrive_flag = False
        self.collision_flag = False

        self.step_time = step_time

    def move_forward(self, vel=np.zeros((2, 1)), stop=True):

        if isinstance(vel, list): 
            vel = np.array(vel, ndmin=2).T

        if stop:
            if self.arrive_flag or self.collision_flag:
                vel = np.zeros((2, 1))

        self.vel = np.clip(vel, np.array([ [-self.v_l], [-self.w_l] ]), np.array([ [self.v_l], [self.w_l] ]))
        self.state = motion_ackermann(self.state, self.wheelbase, self.vel, self.psi_limit, self.step_time)
        self.angular_pos()
    
    def update_state(self, state):

        self.state = state
        self.angular_pos()

    # def state_pre(self, pre_time = 1): 
    #     psi = self.state[3, 0]
    #     vel = self.vel[0, 0]
    #     self.pre_state = motion_acker_pre(self.state, self.wheelbase, vel, psi, self.psi_limit, pre_time, self.step_time)

    def angular_pos(self): 
        # coordinates transform

        x = self.state[0, 0] 
        y = self.state[1, 0] 
        phi = self.state[2, 0] 

        car_x0 = - self.width / 2 
        car_y0 = - (self.length-self.wheelbase)/2

        car_x1 = car_x0 
        car_y1 = car_y0 + self.length

        car_x2 = car_x0 + self.width
        car_y2 = car_y0 + self.length

        car_x3 = car_x0 + self.width
        car_y3 = car_y0

        wheel_x0 = - self.wheelbase_w/2
        wheel_y0 = 0

        wheel_x1 = - self.wheelbase_w/2
        wheel_y1 = self.wheelbase

        wheel_x2 = self.wheelbase_w/2
        wheel_y2 = self.wheelbase

        wheel_x3 = self.wheelbase_w/2
        wheel_y3 = 0

        car_point = np.array([ [car_x0, car_x1, car_x2, car_x3], [car_y0, car_y1, car_y2, car_y3] ])
        wheel_point = np.array([ [wheel_x0, wheel_x1, wheel_x2, wheel_x3], [wheel_y0, wheel_y1, wheel_y2, wheel_y3] ])

        r_phi = phi - pi/2
        rotation_matrix = np.array([[cos(r_phi), -sin(r_phi)], [sin(r_phi), cos(r_phi)]])
        transition_matrix = np.array([[x], [y]])

        self.angular_position = rotation_matrix @ car_point + transition_matrix
        self.wheel_position = rotation_matrix @ wheel_point + transition_matrix

    def arrive(self):
        dis, radian = car_robot.relative(self.state[0:2], self.goal[0:2])

        if dis < self.goal_th:
            self.arrive_flag = True
            return True
        else:
            self.arrive_flag = False
            return False
    
    def cal_des_vel(self, tolerance=0.12):

        dis, radian = car_robot.relative(self.state[0:2], self.goal[0:2])
        car_radian = self.state[2, 0] + self.state[3, 0]

        v_max = self.v_l
        w_max = self.w_l

        diff_radian = car_robot.wraptopi( radian - car_radian )

        if diff_radian > tolerance:
            w_opti = w_max
        
        elif diff_radian < - tolerance:
            w_opti = - w_max   
        else:
            w_opti = 0

        if dis < self.goal_th:
            v_opti = 0
            w_opti = 0
        else:
            v_opti = v_max * cos(diff_radian)
            
            if v_opti < 0:
                v_opti = 0

        return np.array([[v_opti], [w_opti]])

    @staticmethod
    def relative(state1, state2):
        
        dif = state2[0:2] - state1[0:2]

        dis = np.linalg.norm(dif)
        radian = atan2(dif[1, 0], dif[0, 0])
        
        return dis, radian

    @staticmethod
    def wraptopi(radian):

        if radian > pi:
            radian = radian - 2 * pi
        elif radian < -pi:
            radian = radian + 2 * pi
        
        return radian

    
    