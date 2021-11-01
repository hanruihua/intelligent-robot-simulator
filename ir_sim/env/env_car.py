from ir_sim.world import car_robot
from math import pi, cos, sin
import numpy as np

class env_car:
    def __init__(self, car_class=car_robot, car_num=1, step_time=0.1, **kwargs):

        self.car_class = car_class
        self.car_num = car_num
        self.car_list = []

        self.step_time = step_time

        car_shape_list = kwargs.get('car_shape_list', [[1.5, 1, 1, 1] for j in range(self.car_num)])
        car_state_list = kwargs.get('car_state_list', [[j+1, 1, 0, 0] for j in range(self.car_num)])
        car_goal_list = kwargs.get('car_goal_list', [[j+1, 9, 0] for j in range(self.car_num)])
        
        for j in range(self.car_num):
            car = self.car_class(id=j, shape=car_shape_list[j], init_state=car_state_list[j], goal=car_goal_list[j], step_time=self.step_time, **kwargs)

            self.car_list.append(car)
        


        


        

        