import yaml
import numpy as np
from ir_env import env_plot, obs_circle
from PIL import Image

class env_base:

    def __init__(self, world_name = None, robot_num = 1, **kwargs):
        
        if world_name != None:
            with open(world_name) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)
                
                # world 
                self.height = com_list['world_height']
                self.width = com_list['world_width']
                self.step_time = com_list['step_time']
                self.world_map = com_list.get('world_map', None)
                self.xy_reso = com_list.get('xy_resolution', 1)
                self.yaw_reso = com_list.get('yaw_resolution', 5)

                # obstacle
                self.num_obs_cir = com_list.get('obstacle_circle_number', 0)
                if self.num_obs_cir > 0:
                    self.obstacle_circle_list = com_list['obstacle_circle_list']
                
                self.obs_line_list = com_list.get('obstacle_line_list', [])

                # robot
                self.robot_num = com_list.get('robot_number', robot_num)
                if self.robot_num > 0:
                    self.robot_mode = com_list['robot_mode']
                    self.init_state_list = com_list.get('robot_init_state_list', [])
                    self.goal_list = com_list['robot_goal_list']
                    self.robot_radius_list = com_list['robot_radius_list']
                    self.robot_vel_max = com_list.get('robot_vel_max', [1.5, 1.5])
                    self.radius_exp = com_list.get('radius_exp', 0.1)
                # ackermann
                self.acker_num = com_list.get('acker_number', 0)
                if self.acker_num > 0:
                    self.acker_init_state_list = com_list.get('acker_init_state_list', [])
                    self.acker_goal_list=com_list['acker_goal_list']
                    self.acker_shape_list=com_list['acker_shape_list']
                    self.acker_vl=com_list['acker_vel_limit']
                    self.acker_val=com_list['acker_vel_ang_limit']
 
        self.robot_list=[]
        self.obs_cir_list=[]
        self.car_list=[]

        self.init_environment(**kwargs)

    def init_environment(self, **kwargs):

        # obstacle circle 
        for k in range(self.num_obs_cir):
            obs_cir=obs_circle(position=self.obstacle_circle_list[k][0:2], radius=self.obstacle_circle_list[k][2], **kwargs) 
            self.obs_cir_list.append(obs_cir)

        # world
        if self.world_map != None:
            img = Image.open(self.world_map).convert('L')
            px = self.width / self.xy_reso
            py = self.height / self.xy_reso
            img = img.resize( (int(px), int(py) ), Image.NEAREST)
            map_matrix = np.array(img)
            map_matrix = 255 - map_matrix
            map_matrix[map_matrix>255/2] = 255
            map_matrix[map_matrix<255/2] = 0
            self.map_matrix = np.fliplr(map_matrix.T)
        else:
            self.map_matrix = None

        plot = kwargs.get('plot', True)
        if plot:
            self.world_plot = env_plot(self.width+1, self.height+1, robot_list=self.robot_list, obs_cir_list=self.obs_cir_list, car_list=self.car_list, obs_line_list = self.obs_line_list, map_matrix=self.map_matrix, **kwargs)

    def show(self):
        self.world_plot.show()
    
    def show_ani(self):
        self.world_plot.show_ani()
    
    def save_ani(self):
        self.world_plot.save_ani()

    



        
        
            
    
        

        

