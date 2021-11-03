from ir_sim.world import obs_circle
from math import pi, cos, sin
import numpy as np


class env_obs_cir:
    def __init__(self, obs_cir_class=obs_circle, obs_model='static', obs_cir_num=1, obs_cir_mode = 0, step_time=0.1, **kwargs):

        self.obs_cir_class = obs_cir_class
        self.num = obs_cir_num
        self.mode = obs_cir_mode
        self.obs_cir_list = []

        if self.mode == 0 and self.num > 0:
            assert 'obs_radius_list' and 'obs_state_list' in kwargs.keys()
            obs_radius_list = kwargs['obs_radius_list']
            obs_state_list = kwargs['obs_state_list']
        else:
            obs_state_list, obs_radius_list = self.obs_state_dis(self.mode, **kwargs)

        for i in range(self.num):
            obs_cir = self.obs_cir_class(id=i, position=obs_state_list[i], radius=obs_radius_list[i], step_time=step_time, obs_model=obs_model)
            self.obs_cir_list.append(obs_cir)

    def obs_state_dis(self, obs_cir_mode=1, obs_interval=1, obs_radius=0.2, obs_square=[0, 0, 10, 10],  **kwargs):
        # init_mode: 1 single row
        #            2 random
        #            3 circular      
        # square area: x_min, y_min, x_max, y_max
        # circular area: x, y, radius
    
        random_radius = kwargs.get('random_radius', False)

        num = self.num
        state_list = []

        if obs_cir_mode == 1:
             # single row
            state_list = [np.array([ [i * obs_interval], [obs_square[1]], [pi/2] ]) for i in range(int(obs_square[0]), int(obs_square[0])+num)]

        elif obs_cir_mode == 2:
            # random
            state_list = self.random_start(obs_interval, obs_square)

        if random_radius:
            radius_list = np.random.uniform(low = 0.2, high = 1, size = (num,))
        else:
            radius_list = [obs_radius for i in range(num)]

        return state_list, radius_list
    
    def random_start(self, interval = 1, square=[0, 0, 10, 10]):

        random_list = []

        while len(random_list) < self.robot_num:

            new_point = np.random.uniform(low = square[0:2]+[-pi], high = square[2:4]+[pi], size = (1, 3)).T

            if not self.check_collision(new_point, random_list, self.obs_line_list, interval):
                random_list.append(new_point)

        start_list = random_list[:]
        
        return start_list

    