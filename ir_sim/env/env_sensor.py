from ir_sim.world import lidar2d

class env_sensor:
    def __init__(self, sensor_num = 0, sensor_type='lidar2d', **kwargs):

        self.sensor_list = []

        for i in range(0, sensor_num):
            sensor = lidar2d(id=i, **kwargs)
            self.sensor_list.append(sensor)


    def cal_range(self, components):

        for sensor in self.sensor_list:

            
        



        

