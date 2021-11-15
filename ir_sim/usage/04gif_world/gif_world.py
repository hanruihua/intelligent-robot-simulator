from ir_sim.env import env_base
from pathlib import Path
import sys

world_name = 'robot_world.yaml'
env = env_base(world_name = world_name, plot=True, init_mode=0, robot_mode='diff')
# image_path = sys.path[0] + '/' + 'image'
# gif_path = sys.path[0] + '/' + 'gif'

image_path = Path(__file__).parent / 'image'
gif_path = Path(__file__).parent / 'gif'

for i in range(300):

    des_vel = env.robot.cal_des_vel()
    env.robot_step(des_vel)

    env.save_fig(image_path, i) 
    env.render()

env.save_ani(image_path, gif_path)

    










