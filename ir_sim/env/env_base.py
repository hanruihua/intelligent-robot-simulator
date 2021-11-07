import yaml
import numpy as np
from ir_sim.world import env_plot, mobile_robot, car_robot, obs_circle
from ir_sim.env.env_robot import env_robot
from ir_sim.env.env_car import env_car
from ir_sim.env.env_obs_cir import env_obs_cir
from ir_sim.env.env_obs_line import env_obs_line
from PIL import Image
import sys

class env_base:

    def __init__(self, world_name=None, plot=True, **kwargs):

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name
            
            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)

                world_args = com_list['world']
                self.__height = world_args.get('world_height', 10)
                self.__width = world_args.get('world_width', 10)
                self.__step_time = world_args.get('step_time', 0.1)
                self.world_map =  world_args.get('world_map', None)
                self.xy_reso = world_args.get('xy_resolution', 1)
                self.yaw_reso = world_args.get('yaw_resolution', 5)

                self.robots_args = com_list.get('robots', dict())
                self.robot_number = self.robots_args.get('number', 0)
                
                self.cars_args = com_list.get('cars', dict())
                self.car_number = self.cars_args.get('number', 0)
            
                # obs_cir
                self.obs_cirs_args = com_list.get('obs_cirs', dict())
                self.obs_cir_number = self.obs_cirs_args.get('number', 0)
                
                # obs line
                self.obs_lines_args = com_list.get('obs_lines', dict())


        else:
            self.__height = kwargs.get('world_height', 10)
            self.__width = kwargs.get('world_width', 10)
            self.__step_time = kwargs.get('step_time', 0.1)
            self.world_map = kwargs.get('world_map', None)
            self.xy_reso = kwargs.get('xy_resolution', 1)
            self.yaw_reso = kwargs.get('yaw_resolution', 5)
                    
        self.plot = plot
        self.components = dict()
        self.init_environment(**kwargs)

    def init_environment(self, robot_class=mobile_robot, car_class=car_robot, obs_cir_class=obs_circle,  **kwargs):

        # world
        px = int(self.__width / self.xy_reso)
        py = int(self.__height / self.xy_reso)

        if self.world_map != None:
            
            world_map_path = sys.path[0] + '/' + self.world_map
            img = Image.open(world_map_path).convert('L')
            img = img.resize( (px, py), Image.NEAREST)
            map_matrix = np.array(img)
            map_matrix = 255 - map_matrix
            map_matrix[map_matrix>255/2] = 255
            map_matrix[map_matrix<255/2] = 0
            self.map_matrix = np.fliplr(map_matrix.T)
        else:
            self.map_matrix = np.zeros([px, py])

        self.components['map_matrix'] = self.map_matrix
        self.components['xy_reso'] = self.xy_reso

        self.components['robots'] = env_robot(robot_class=robot_class, robot_num=self.robot_number, step_time=self.__step_time, **{**self.robots_args, **kwargs})

        self.components['cars'] = env_car(car_class=car_class, car_num=self.car_number, step_time=self.__step_time, **{**self.cars_args, **kwargs})

        self.components['obs_cirs'] = env_obs_cir(obs_cir_class=obs_cir_class, obs_cir_num=self.obs_cir_number, step_time=self.__step_time, **{**self.obs_cirs_args, **kwargs})

        self.components['obs_lines'] = env_obs_line(**{**self.obs_lines_args, **kwargs})

        if self.plot:
            self.world_plot = env_plot(self.__width, self.__height, self.components, **kwargs)
        
        self.time = 0

        if self.robot_number > 0:
            self.robot = self.components['robots'].robot_list[0]
        
        if self.car_number > 0:
            self.car = self.components['cars'].car_list[0]
    
    def collision_check(self):
        collision = False
        for robot in self.components['robots'].robot_list: 
            if robot.collision_check(self.components):
                collision = True

        for car in self.components['cars'].car_list: 
            if car.collision_check(self.components):
                collision =True

        return collision

    def step(self, vel_list, **kwargs):
        # if not isinstance(vel_list, list):
        #     self.robot.move_forward(**kwargs)
        # else:
        #     pass
        
        # for robot in self.components['robots'].robot_list:
        #     pass
        pass

    
    def render(self, time=0.01, **kwargs):

        if self.plot:
            self.world_plot.com_cla()
            self.world_plot.draw_dyna_components(**kwargs)
            self.world_plot.pause(time)
        
        self.time = self.time + 1
        
    def save_fig(self, path, i):
        self.world_plot.save_gif_figure(path, i)
    
    def save_ani(self, image_path, ani_path):
        self.world_plot.create_animate(image_path, ani_path)

    def show(self):
        self.world_plot.show()
    
    def show_ani(self):
        self.world_plot.show_ani()
    


    



        
        
            
    
        

        

