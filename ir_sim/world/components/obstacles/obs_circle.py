import numpy as np
from ir_sim.world import motion_omni
from math import sin, cos, atan2

class obs_circle:
    def __init__(self, id=0, state=np.zeros((2, 1)), radius=0.2, velocity=np.zeros((2, 1)), vel_max=2 * np.ones((2, 1)), 
     step_time=0.1, obs_model='static', goal=np.zeros((2, 1)), goal_threshold=0.1, **kwargs):
        
        if isinstance(state, list):
            state=np.array(state, ndmin=2).T   
        
        if isinstance(velocity, list):
            velocity=np.array(velocity, ndmin=2).T 

        if isinstance(goal, list): 
            goal = np.array(goal, ndmin=2).T

        if isinstance(vel_max, list): 
            vel_max = np.array(vel_max, ndmin=2).T 

        self.id = id
        self.state = state
        self.radius = radius
        self.vel_omni = velocity
        self.vel_max = vel_max
        self.step_time = step_time
        self.obs_model = obs_model # static, dynamic
        self.goal = goal
        self.goal_threshold = goal_threshold
        self.radius_collision = round(radius + kwargs.get('radius_exp', 0.1), 2)
        self.arrive_flag = False

    def move_forward(self, vel, stop=True, **vel_kwargs):
        
        if isinstance(vel, list): 
            vel = np.array(vel, ndmin=2).T

        if vel.shape == (2,):
            vel = vel[:,np.newaxis]

        if stop:
            if self.arrive():
                vel = np.zeros((2, 1))

        self.state = motion_omni(self.state, vel, self.step_time, **vel_kwargs)
        self.vel_omni = vel

    # def omni_obs_state(self):
        
    #     x = self.state[0, 0]
    #     y = self.state[1, 0]

    #     vx = self.vel_omni[0, 0]
    #     vy = self.vel_omni[1, 0]

    #     radius = self.radius_collision

    #     return [x, y, vx, vy, radius]

    def omni_obs_state(self):
        rc_array = self.radius * np.ones((1,1))    
        return np.concatenate((self.state, self.vel_omni, rc_array), axis = 0)
    
    def omni_state(self):

        x = self.state[0, 0]
        y = self.state[1, 0]

        vx = self.vel_omni[0, 0]
        vy = self.vel_omni[1, 0]

        radius = self.radius_collision

        vx_des, vy_des = self.cal_des_vel_omni(self.vel_max)

        return [x, y, vx, vy, radius, vx_des, vy_des] 

    def cal_des_vel_omni(self, v_max):

        dis, radian = self.relative(self.state, self.goal)

        if dis > self.goal_threshold:

            vx = v_max[0, 0] * cos(radian)
            vy = v_max[1, 0] * sin(radian)
        
        else:
            vx = 0
            vy = 0

        return vx, vy
    
    def relative(self, state1, state2):

        dis = np.sqrt( (state1[0, 0] - state2[0, 0])**2 + (state1[1, 0] - state2[1, 0])**2 )
        radian = atan2(state2[1, 0] - state1[1, 0], state2[0, 0] - state1[0, 0])

        return dis, radian 

    def arrive(self):

        dist = np.linalg.norm(self.state[0:2] - self.goal[0:2]) 

        if dist < self.goal_threshold:
            self.arrive_flag = True
            return True
        else:
            self.arrive_flag = False
            return False 