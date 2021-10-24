import yaml
import numpy as np
from ir_world import env_plot
from PIL import Image
import sys

class env_base:

    def __init__(self, world_name=None, plot=True, **kwargs):

        if world_name != None:
            world_name = sys.path[0] + '/' + world_name
            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)
    
                self.height = com_list.get('world_height', 10)
                self.width = com_list.get('world_width', 10)
                self.step_time = com_list.get('step_time', 0.1)
                self.world_map = com_list.get('world_map', None)
                self.xy_reso = com_list.get('xy_resolution', 1)
                self.yaw_reso = com_list.get('yaw_resolution', 5)
        else:
            self.height = kwargs.get('world_height', 10)
            self.width = kwargs.get('world_width', 10)
            self.step_time = kwargs.get('step_time', 0.1)
            self.world_map = kwargs.get('world_map', None)
            self.xy_reso = kwargs.get('xy_resolution', 1)
            self.yaw_reso = kwargs.get('yaw_resolution', 5)
                    
        self.plot = plot
        self.component_list = []
        self.init_environment(**kwargs)

    def init_environment(self, **kwargs):

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

        self.component_list.append(self.map_matrix)

        if self.plot:
            self.world_plot = env_plot(self.width+1, self.height+1, self.component_list, **kwargs)

    def show(self):
        self.world_plot.show()
    
    def show_ani(self):
        self.world_plot.show_ani()
    
    def save_ani(self):
        self.world_plot.save_ani()

    



        
        
            
    
        

        

