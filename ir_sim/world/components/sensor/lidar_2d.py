from math import pi
import numpy as np
from collections import namedtuple
from math import cos, sin
from ir_sim.util import range_seg_matrix, range_cir_seg, range_seg_seg
import random

class lidar2d:

    def __init__(self, range_min=0, range_max=10, angle_min=0, angle_max=pi, number=36, scan_time=0.1, noise=True, std=0.2, install_pos=np.zeros(3,)):
        
        self.range_min = range_min
        self.range_max = range_max 
        self.angle_min = angle_min  
        self.angle_max = angle_max
        self.angle_inc = (angle_max - angle_min) / number

        self.scan_time = scan_time
        self.noise = noise
        self.std = std

        self.data_num = number
        self.range_data = (range_max - range_min) * np.ones(self.data_num,)
        self.inter_points = np.zeros((self.data_num, 2))

        self.install_pos = install_pos

    def cal_range(self, state, components):

        theta = state[2, 0]

        a_min = theta - (self.angle_min + self.angle_max) / 2
        a_max = theta + (self.angle_min + self.angle_max) / 2

        angle_list = np.linspace(a_min, a_max, num=self.data_num)

        length = self.range_max - self.range_min

        start_point = state[0:2, 0]

        for i, angle in enumerate(angle_list):
            end_point = start_point + length * np.array([cos(angle), sin(angle)])
            segment = [start_point, end_point]

            flag, int_point, lrange = self.seg_components(segment, components)

            if flag:
                
                if self.noise:
                    self.range_data[i] = round(random.gauss(lrange, self.std), 2)
                else:
                    self.range_data[i] = round(lrange, 2)

                self.inter_points[i, :] = int_point[:]
            else:
                self.inter_points[i, :] = end_point[:]
        
    def seg_components(self, segment, components):
        
        min_lrange = self.range_max - self.range_min
        min_int_point = segment[1]
        collision_flag = False
        for robot in components['robots'].robot_list:
            
            flag, int_point, lrange = range_cir_seg(robot.state[0:2, 0], robot.radius, segment)

            if flag and lrange < min_lrange:
                min_lrange = lrange
                min_int_point = int_point
                collision_flag = True

        for obs_cir in components['obs_cirs'].obs_cir_list:
            flag, int_point, lrange = range_cir_seg(obs_cir.pos[0:2, 0], obs_cir.radius, segment)
            
            if flag and lrange < min_lrange:
                min_lrange = lrange
                min_int_point = int_point
                collision_flag = True

        flag, int_point, lrange = range_seg_matrix(segment, components['map_matrix'], components['xy_reso'])

        if flag and lrange < min_lrange:
            min_lrange = lrange
            min_int_point = int_point
            collision_flag = True

        for line in components['obs_lines'].line_states:
            segment2 = [ np.array([line[0], line[1]]), np.array([line[2], line[3]]) ]
            flag, int_point, lrange = range_seg_seg(segment, segment2)

            if flag and lrange < min_lrange:
                min_lrange = lrange
                min_int_point = int_point
                collision_flag = True

        return collision_flag, min_int_point, min_lrange
            
            
        
        




        

    


        
        





