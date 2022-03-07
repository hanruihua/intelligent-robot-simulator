import yaml
import numpy as np
import sys

import matplotlib.pyplot as plt

from ir_sim.world import env_plot, mobile_robot, car_robot, obs_circle, obs_polygon
from ir_sim.env.env_robot import env_robot
from ir_sim.env.env_car import env_car
from ir_sim.env.env_obs_cir import env_obs_cir
from ir_sim.env.env_obs_line import env_obs_line
from ir_sim.env.env_obs_poly import env_obs_poly
from PIL import Image
from pynput import keyboard

class env_base:

    def __init__(self, world_name=None, plot=True,  **kwargs):

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name
            
            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)

                world_args = com_list['world']
                self.__height = world_args.get('world_height', 10)
                self.__width = world_args.get('world_width', 10)
                self.offset_x = world_args.get('offset_x', 0)
                self.offset_y = world_args.get('offset_y', 0)
                self.step_time = world_args.get('step_time', 0.1)
                self.world_map =  world_args.get('world_map', None)
                self.xy_reso = world_args.get('xy_resolution', 1)
                self.yaw_reso = world_args.get('yaw_resolution', 5)
                self.offset = np.array([self.offset_x, self.offset_y])

                self.robots_args = com_list.get('robots', dict())
                self.robot_number = kwargs.get('robot_number', self.robots_args.get('robot_number', 0) )
                
                self.cars_args = com_list.get('cars', dict())
                self.car_number = self.cars_args.get('number', 0)
            
                # obs_cir
                self.obs_cirs_args = com_list.get('obs_cirs', dict())
                self.obs_cir_number = self.obs_cirs_args.get('number', 0)
                self.obs_step_mode = self.obs_cirs_args.get('obs_step_mode', 0)

                # obs line
                self.obs_lines_args = com_list.get('obs_lines', dict())

                # obs polygons
                self.obs_polygons_args = com_list.get('obs_polygons', dict())
                self.vertexes_list = self.obs_polygons_args.get('vertexes_list', [])
                self.obs_poly_num = self.obs_polygons_args.get('number', 0)
        else:
            self.__height = kwargs.get('world_height', 10)
            self.__width = kwargs.get('world_width', 10)
            self.step_time = kwargs.get('step_time', 0.1)
            self.world_map = kwargs.get('world_map', None)
            self.xy_reso = kwargs.get('xy_resolution', 1)
            self.yaw_reso = kwargs.get('yaw_resolution', 5)
            self.offset_x = kwargs.get('offset_x', 0)
            self.offset_y = kwargs.get('offset_y', 0)
            self.robot_number = kwargs.get('robot_number', 0)
            self.obs_cir_number = kwargs.get('obs_cir_number', 0)
            self.car_number = kwargs.get('car_number', 0)
            self.robots_args = kwargs.get('robots', dict())
            self.obs_cirs_args = kwargs.get('obs_cirs', dict())
            self.cars_args = kwargs.get('cars', dict())
            self.obs_lines_args = kwargs.get('obs_lines', dict())
            self.obs_polygons_args = kwargs.get('obs_polygons', dict())
        
        self.plot = plot
        self.components = dict()
        self.init_environment(**kwargs)

        if kwargs.get('teleop_key', False):
            
            self.key_lv_max = 2
            self.key_ang_max = 2
            self.key_lv = 0
            self.key_ang = 0
            self.key_id = 1
            self.alt_flag = 0

            plt.rcParams['keymap.save'].remove('s')
            plt.rcParams['keymap.quit'].remove('q')
            
            self.key_vel = np.zeros(2,)

            print('start to keyboard control')
            print('w: forward', 's: backforward', 'a: turn left', 'd: turn right', 
                  'q: decrease linear velocity', 'e: increase linear velocity',
                  'z: decrease angular velocity', 'c: increase angular velocity',
                  'alt+num: change current control robot id')
                  
            self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
            self.listener.start()

        if kwargs.get('mouse', False):
            pass


    def init_environment(self, robot_class=mobile_robot, car_class=car_robot, obs_cir_class=obs_circle, obs_polygon_class=obs_polygon,  **kwargs):

        # world
        px = int(self.__width / self.xy_reso)
        py = int(self.__height / self.xy_reso)

        if self.world_map != None:
            
            world_map_path = sys.path[0] + '/' + self.world_map
            img = Image.open(world_map_path).convert('L')
            # img = Image.open(world_map_path)
            img = img.resize( (px, py), Image.NEAREST)
            # img = img.resize( (px, py), Image.ANTIALIAS)
            # img.thumbnail( (px, py))

            map_matrix = np.array(img)
            map_matrix = 255 - map_matrix
            map_matrix[map_matrix>255/2] = 255
            map_matrix[map_matrix<255/2] = 0
            # map_matrix[map_matrix>0] = 255
            # map_matrix[map_matrix==0] = 0

            self.map_matrix = np.fliplr(map_matrix.T)
        else:
            self.map_matrix = None

        self.components['map_matrix'] = self.map_matrix
        self.components['xy_reso'] = self.xy_reso
        self.components['offset'] = np.array([self.offset_x, self.offset_y])

        self.components['obs_lines'] = env_obs_line(**{**self.obs_lines_args, **kwargs})
        self.obs_line_states=self.components['obs_lines'].obs_line_states

        self.components['obs_circles'] = env_obs_cir(obs_cir_class=obs_cir_class, obs_cir_num=self.obs_cir_number, step_time=self.step_time, components=self.components, **{**self.obs_cirs_args, **kwargs})
        self.obs_cir_list = self.components['obs_circles'].obs_cir_list

        self.components['obs_polygons'] = env_obs_poly(obs_poly_class=obs_polygon_class, vertex_list=self.vertexes_list, obs_poly_num=self.obs_poly_num, **{**self.obs_polygons_args, **kwargs})
        self.obs_poly_list = self.components['obs_polygons'].obs_poly_list        

        self.components['robots'] = env_robot(robot_class=robot_class, step_time=self.step_time, components=self.components, **{**self.robots_args, **kwargs})
        self.robot_list = self.components['robots'].robot_list

        self.components['cars'] = env_car(car_class=car_class, car_num=self.car_number, step_time=self.step_time, **{**self.cars_args, **kwargs})
        self.car_list = self.components['cars'].car_list

        if self.plot:
            self.world_plot = env_plot(self.__width, self.__height, self.components, offset_x=self.offset_x, offset_y=self.offset_y, **kwargs)
        
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
                collision = True

        return collision
    
    def arrive_check(self):
        arrive=True

        for robot in self.components['robots'].robot_list: 
            if not robot.arrive_flag:
                arrive = False

        for car in self.components['cars'].car_list: 
            if not car.arrive_flag:
                arrive = False

        return arrive

    def robot_step(self, vel_list, robot_id = None, **kwargs):

        if robot_id == None:

            if not isinstance(vel_list, list):
                self.robot.move_forward(vel_list, **kwargs)
            else:
                for i, robot in enumerate(self.components['robots'].robot_list):
                    robot.move_forward(vel_list[i], **kwargs)
        else:
            self.components['robots'].robot_list[robot_id-1].move_forward(vel_list, **kwargs)

        for robot in self.components['robots'].robot_list:
            robot.cal_lidar_range(self.components)

    def car_step(self, vel_list, car_id=None, **kwargs):

        if car_id == None:
            if not isinstance(vel_list, list):
                self.car.move_forward(vel_list, **kwargs)
            else:
                for i, car in enumerate(self.components['cars'].car_list):
                    car.move_forward(vel_list[i], **kwargs)
        else:
            self.components['cars'].car_list[car_id-1].move_forward(vel_list, **kwargs)
        
        for car in self.components['cars'].car_list:
            car.cal_lidar_range(self.components)

    def obs_cirs_step(self, vel_list=[], obs_id=None, **kwargs):
        
        if self.obs_step_mode == 'default':
            if obs_id == None:
                for i, obs_cir in enumerate(self.components['obs_circles'].obs_cir_list):
                    obs_cir.move_forward(vel_list[i], **kwargs)
            else:
                self.components['obs_circles'].obs_cir_list[obs_id-1].move_forward(vel_list, **kwargs)

        elif self.obs_step_mode == 'wander':
            # rvo
            self.components['obs_circles'].step_wander(**kwargs)

    def render(self, time=0.05, **kwargs):

        if self.plot:
            self.world_plot.com_cla()
            self.world_plot.draw_dyna_components(**kwargs)
            self.world_plot.pause(time)
            
        self.time = self.time + time

    # def reset(self, ):


    def on_press(self, key):

        try:

            if key.char.isdigit() and self.alt_flag:

                if int(key.char) > self.robot_number:
                    print('out of number of robots')
                else:
                    self.key_id = int(key.char)

            if key.char == 'w':
                self.key_lv = self.key_lv_max
            if key.char == 's':
                self.key_lv = - self.key_lv_max
            if key.char == 'a':
                self.key_ang = self.key_ang_max
            if key.char == 'd':
                self.key_ang = -self.key_ang_max
            
            self.key_vel = np.array([self.key_lv, self.key_ang])

        except AttributeError:
            
            if key == keyboard.Key.alt:
                self.alt_flag = 1
    
    def on_release(self, key):
        
        try:
            if key.char == 'w':
                self.key_lv = 0
            if key.char == 's':
                self.key_lv = 0
            if key.char == 'a':
                self.key_ang = 0
            if key.char == 'd':
                self.key_ang = 0
            if key.char == 'q':
                self.key_lv_max = self.key_lv_max - 0.2
                print('current lv ', self.key_lv_max)
            if key.char == 'e':
                self.key_lv_max = self.key_lv_max + 0.2
                print('current lv ', self.key_lv_max)
            
            if key.char == 'z':
                self.key_ang_max = self.key_ang_max - 0.2
                print('current ang ', self.key_ang_max)
            if key.char == 'c':
                self.key_ang_max = self.key_ang_max + 0.2
                print('current ang ', self.key_ang_max)
            
            self.key_vel = np.array([self.key_lv, self.key_ang])

        except AttributeError:
            if key == keyboard.Key.alt:
                self.alt_flag = 0

        
    def save_fig(self, path, i):
        self.world_plot.save_gif_figure(path, i)
    
    def save_ani(self, image_path, ani_path, ani_name='animated', **kwargs):
        self.world_plot.create_animate(image_path, ani_path, ani_name=ani_name, **kwargs)

    def show(self, **kwargs):
        self.world_plot.draw_dyna_components(**kwargs)
        self.world_plot.show()
    
    def show_ani(self):
        self.world_plot.show_ani()
    


    



        
        
            
    
        

        

