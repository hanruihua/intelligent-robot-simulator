import numpy as np
from ir_env import motion_omni


class obs_circle:
    def __init__(self, position=np.zeros((2, 1)), radius=0.2, velocity=np.zeros((2, 1)), velocity_max=2 * np.ones((2, 1)), 
     step_time=0.1, **kwargs):
        
        if isinstance(position, list):
            position=np.array(position, ndmin=2).T   
        
        if isinstance(velocity, list):
            velocity=np.array(velocity, ndmin=2).T 

        self.pos = position
        self.radius = radius
        self.vel = velocity
        self.vel_max = velocity_max
        self.step_time = step_time

    def move_forward(self, vel, **vel_kwargs):
        motion_omni(self.pos, vel, self.step_time, **vel_kwargs)

    def omni_obs_state(self, exp=True):
        rc_array = self.radius * np.ones((1,1))    
        return np.concatenate((self.pos, self.vel, rc_array), axis = 0)

        