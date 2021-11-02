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

    for robot in env.components['robots'].robot_list:

        if robot.arrive_flag:
            break

        des_vel = robot.cal_des_vel()
        robot.move_forward(des_vel)

    if all([r.arrive_flag for r in env.components['robots'].robot_list]):
        break

    env.save_fig(image_path, i) 
    env.render()

env.save_ani(image_path, gif_path)

    










