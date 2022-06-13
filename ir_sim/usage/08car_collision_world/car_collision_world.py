from ir_sim.env import env_base
from pathlib import Path
# from ir_sim.world import 
import time 

world_name = 'car_collision_world.yaml'
env = env_base(world_name = world_name, plot=True, init_mode=0, robot_mode='diff')


# image_path = Path(__file__).parent / 'image'
# gif_path = Path(__file__).parent / 'gif'

for i in range(300):

    des_vel = env.car.cal_des_vel()
    env.car.move_forward(des_vel)

    env.render()
    
    start_time = time.time()
    if env.collision_check():
        break
    print(time.time() - start_time)

env.show()


