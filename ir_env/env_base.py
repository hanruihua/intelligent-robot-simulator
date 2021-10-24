import yaml
import numpy as np
from ir_world import env_plot
from PIL import Image
import sys
from ir_world import mobile_robot, car_robot

class env_base:

    def __init__(self, world_name=None, plot=True, **kwargs):

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name
            
            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)

                world = com_list['world']
                self.height = world.get('world_height', 10)
                self.width = world.get('world_width', 10)
                self.step_time = world.get('step_time', 0.1)
                self.world_map = sys.path[0] + '/' + world.get('world_map', None)
                self.xy_reso = world.get('xy_resolution', 1)
                self.yaw_reso = world.get('yaw_resolution', 5)

                robots = com_list.get('robots', None)

                if robots != None:
                    self.robot_number = robots.get('number', 0)
                else:
                    self.robot_number = 0
                    
                cars = com_list.get('cars', None)

                if cars != None:
                    self.car_number = cars.get('number', 0)
                else:
                    self.car_number = 0      
        else:
            self.height = kwargs.get('world_height', 10)
            self.width = kwargs.get('world_width', 10)
            self.step_time = kwargs.get('step_time', 0.1)
            self.world_map = kwargs.get('world_map', None)
            self.xy_reso = kwargs.get('xy_resolution', 1)
            self.yaw_reso = kwargs.get('yaw_resolution', 5)
                    
        self.plot = plot
        self.components = dict()
        self.init_environment(**kwargs)

    def init_environment(self, robot_class=mobile_robot, car_class=car_robot, **kwargs):

        # world
        px = int(self.width / self.xy_reso)
        py = int(self.height / self.xy_reso)

        if self.world_map != None:
            img = Image.open(self.world_map).convert('L')
            img = img.resize( (px, py), Image.NEAREST)
            map_matrix = np.array(img)
            map_matrix = 255 - map_matrix
            map_matrix[map_matrix>255/2] = 255
            map_matrix[map_matrix<255/2] = 0
            self.map_matrix = np.fliplr(map_matrix.T)
        else:
            self.map_matrix = np.zeros([px, py])



        if self.car_number > 0:
            for i in range(self.car_number):
                pass
        else:
          self.cars = []  

        
        self.components['map_matrix'] = self.map_matrix
        # self.components['robots'] = self.map_matrix

        if self.plot:
            self.world_plot = env_plot(self.width, self.height, self.components, **kwargs)
    


    def step(self):
        pass



    def show(self):
        self.world_plot.show()
    
    def show_ani(self):
        self.world_plot.show_ani()
    
    def save_ani(self):
        self.world_plot.save_ani()

    



        
        
            
    
        

        

