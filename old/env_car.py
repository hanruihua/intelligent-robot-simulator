from ir_env import env_base
from ir_env import car_robot

class env_car(env_base):
    def __init__(self, world_name):
        super.__init__(world_name)

    def initialization(self, **kwargs):

        # car robot
        for j in range(self.acker_num):
            car = car_robot(id=j, shape = self.acker_shape_list[j], init_state=self.acker_init_state_list[j], goal=self.acker_goal_list[j], step_time=self.step_time, vel_limit=self.acker_vl, vel_ang_limit =self.acker_val, **kwargs)
            self.car_list.append(car)
        
    def cal_des_car_list(self):
        vel_list = list(map(lambda x: x.cal_des_vel() , self.car_list))
        return vel_list

    def step(self, vel_car_list=[], **vel_kwargs):
        for car, vel in zip(self.car_list, vel_car_list):
                car.move_forward(vel, **vel_kwargs)

    def arrive_all(self):

        for car in self.car_list:
            if not car.arrive():
                return False
        
        return True

    def render(self, time=0.1, **kwargs):
        
        self.world.com_cla()
        self.world.draw_obs_cir_list()
        self.world.draw_car_list(**kwargs)
        self.world.pause(time)