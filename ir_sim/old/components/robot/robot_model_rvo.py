import numpy as np
import math
from math import sin
from math import cos
from scipy import stats
'''
robot model: 
    mode: 'diff'  differential drive robot
          'omni'  omnidirectional wheel
'''

def distance(point1, point2):

    dis = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    return dis

def robot_dis(robot1, robot2):
    dis = distance(robot1.position, robot2.position)
    return dis

class robot_model:

    def __init__(self, name, radius, mode = 'omni', init_position = [0.0, 0.0, 0.0], time_step = 0.05):
        
        self.name = name
        self.radius = radius
        self.mode = mode
        self.position = np.array(init_position[0:2]) if mode == 'omni' else np.array(init_position)     # x, y, theta
        self.position_est = np.array(init_position[0:2]) if mode == 'omni' else np.array(init_position)     # x, y, theta
        self.velocity = np.array([0.0, 0.0]) # 'omni' : x, y velocity 'diff': linear, angle
        self.time_step = time_step
       
    def update_position(self):

        time_step = self.time_step

        if self.mode == 'omni': 
            self.position = self.position + self.velocity * time_step
        
        if self.mode == 'diff': 
            
            theta = self.position[2]
            v = self.velocity[0]
            w = self.velocity[1]

            if abs(w) < 0.001:

                x_inc = v * time_step * cos(theta)
                y_inc = v * time_step * sin(theta)
                theta_inc = 0
            else:
                x_inc = - (v/w) * sin(theta) + (v/w) * sin(theta + w * time_step)
                y_inc = (v/w) * cos(theta) - (v/w) * cos(theta + w * time_step)
                theta_inc = w * time_step
        
            self.position = self.position + [x_inc, y_inc, theta_inc]
        
        # print(self.position)
        return self.position

    def update_position_noise(self, velx_noise, vely_noise):

        time_step = self.time_step

        if self.mode == 'omni': 
            self.position = self.position + (self.velocity + np.array(np.random.normal([0,0], [velx_noise, vely_noise]))) * time_step
            self.position_est = self.position_est + self.velocity * time_step

        if self.mode == 'diff': 
            
            theta = self.position[2]
            v = self.velocity[0]
            w = self.velocity[1]

            if abs(w) < 0.001:

                x_inc = v * time_step * cos(theta)
                y_inc = v * time_step * sin(theta)
                theta_inc = 0
            else:
                x_inc = - (v/w) * sin(theta) + (v/w) * sin(theta + w * time_step)
                y_inc = (v/w) * cos(theta) - (v/w) * cos(theta + w * time_step)
                theta_inc = w * time_step
        
            self.position = self.position + [x_inc, y_inc, theta_inc]
        
        # print(self.position)
        return self.position

    def update_vel(self, vel):
        self.velocity = np.array(vel)
    

class robot_rvo(robot_model):

    # for rvo calculation

    def __init__(self,name, radius, mode = 'omni', init_position = [0.0, 0.0, 0.0], time_step = 0.05, goal = [0.0, 0.0], vel_max = 1, reach_threshold = 0.1):

        super(robot_rvo, self).__init__(name, radius, mode, init_position, time_step)
        self.desired_vel = np.array([0, 0])
        self.goal = goal
        self.vel_max = vel_max
        self.reach_threshold = reach_threshold
        self.particle_angle = np.array([])

    def cal_desired_vel(self):
        
        diff = self.goal - self.position_est
        norm = distance(diff, np.array([0, 0]))
        norm_diff = diff/norm
        if norm > 1:
            self.desired_vel = self.vel_max * norm_diff
        else:
            self.desired_vel = self.vel_max * norm_diff * 0.5

        if self.reach():
            self.desired_vel = np.array([0, 0])

        # print(self.desired_vel)
        return self.desired_vel
       
    def reach(self):
        dis = distance(self.position_est, self.goal)
        return dis < self.reach_threshold

    def cal_rvo_vel(self):
        pass

    def cal_range(self, robot_list):   
        range = list(map(lambda x: distance(x.position, self.position), robot_list))
        return range

    def cal_range_noise(self, robot_list, range_var):   
        range = list(map(lambda x: distance(x.position, self.position) +  np.random.normal(0, range_var), robot_list))
        return range


    def resample(self, particle, weight):
        
        parti_num = particle.shape[0]
        parti_num_inv = 1.0/parti_num
        new_particle = []
        c = weight[0]
        r = np.random.random_sample() * parti_num_inv
        i = 0

        for m in range(1, parti_num + 1):
            U = r + (m - 1) * parti_num_inv
            while U > c:
                i = i + 1
                c = c + weight[i]
            new_particle.append(particle[i, :])

        return np.array(new_particle)

    
    def density_extraction(self, parti_pos):
        pass

    def cal_rel_pos(self, robot_range, particle_num, var_threshold):
        
        num_robot = len(robot_range)
        var_diff = 0.05

        if self.particle_angle.size == 0:     
            self.particle_angle = np.zeros((num_robot, particle_num))
            self.parti_pos = np.zeros((num_robot, particle_num, 2))
            self.weight = np.ones(particle_num) / particle_num
            self.rel_pos_list = [[0.0, 0.0] for i in range(num_robot)]
            self.rel_pos_var = [0.0 for i in range(num_robot)]
            
            for i in range(num_robot):
                if robot_range[i] != 0:
                    for j in range(particle_num):
                        self.particle_angle[i, j] = j * ((math.pi * 2)/(particle_num - 1))
                        self.parti_pos[i, j, 0] = robot_range[i] * cos(self.particle_angle[i, j])
                        self.parti_pos[i, j, 1] = robot_range[i] * sin(self.particle_angle[i, j])

                    mean_angle = np.mean(self.particle_angle[i, :])
                    var_angle = np.var(self.particle_angle[i, :])

                    rel_pos = [robot_range[i] * cos(mean_angle), robot_range[i] * sin(mean_angle)]
                    self.rel_pos_list[i] = rel_pos
                    self.rel_pos_var[i] = var_angle
            
            return self.rel_pos_list, self.rel_pos_var
                    
        else:
            for i in range(num_robot):
                if robot_range[i] != 0:
                    range_i = robot_range[i]
                    parti_pos_i = self.parti_pos[i, :]
                    diff = list(map(lambda x: distance(x, [0,0]) - range_i, parti_pos_i))
                    close_point = list(filter(lambda x: abs(x) < 0.05, diff))
                    
                    weight = stats.norm(0, var_diff).pdf(diff)
                    diff_weight = list(filter(lambda x: x > 1, weight))
                    weight_sum = np.sum(weight)
                    weight = weight / weight_sum
                    

                    new_pos_i = self.resample(parti_pos_i, weight)
                
                    mean_pos = np.mean(new_pos_i, axis = 0)
                    var_pos_i = np.var(parti_pos_i, axis = 0)
                    var_pos = np.var(new_pos_i, axis = 0)

                    rel_range = distance(mean_pos, [0,0])

                    # rel_pos = [robot_range[i] * cos(mean_angle), robot_range[i] * sin(mean_angle)]

                    self.rel_pos_list[i] = mean_pos
                    self.rel_pos_var[i] = var_pos
                    self.parti_pos[i, :] = new_pos_i

            return self.rel_pos_list, self.rel_pos_var

    def update_particle(self, v_list):

        if self.mode == 'omni': 
            for i in range(self.parti_pos.shape[0]):
                rel_vel = v_list[i] - self.velocity              
                for j in range(self.parti_pos.shape[1]):
                    self.parti_pos[i, j, :] = self.parti_pos[i, j, :] + rel_vel * self.time_step

    # def update_particle_noise(self, v_list):

    #     if self.mode == 'omni': 
    #         for i in range(self.parti_pos.shape[0]):
    #             rel_vel = v_list[i] - self.velocity              
    #             for j in range(self.parti_pos.shape[1]):
    #                 self.parti_pos[i, j, :] = self.parti_pos[i, j, :] + (rel_vel + np.array(np.random.normal([0,0], [0.02, 0.02]))) * self.time_step
        
        


                        






        

    


    




        
