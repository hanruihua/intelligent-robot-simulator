from math import pi
import numpy as np

class lidar2d:

    def __init__(self, range_min=0, range_max=10, angle_min=0, angle_max=pi, angle_inc= 5 * pi / 180, scan_time=0.1, noise=True, std=0.2):

        self.range_min = range_min
        self.range_max = range_max 
        self.angle_min = angle_min  
        self.angle_max = angle_max
        self.angle_inc = angle_inc

        self.scan_time = scan_time
        self.noise = noise
        self.std = std

        data_num = (angle_max - angle_min) / angle_inc 
        self.range_data = range_max * np.ones(data_num,)


    def cal_range(self, state, components):

        x = state[0, 0]
        y = state[1, 0]
        theta = state[2, 0]

        
        





