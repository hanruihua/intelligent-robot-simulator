from ir_sim.env import env_base
# from ir_sim.world import 
import matplotlib.pyplot as plt
from pathlib import Path

image_path = Path(__file__).parent / 'image'
gif_path = Path(__file__).parent / 'gif'

world_name = 'teleop_key.yaml'
env = env_base(world_name = world_name, plot=True, teleop_key=False)

# for i in range(100):

#     # print(env.key_vel)
#     env.robot_step(env.key_vel, env.key_id)

#     env.save_fig(image_path, i)
#     env.render()
    
#     env.collision_check()


env.save_ani(image_path, gif_path)
env.show()







