from ir_sim.env import env_base
# from ir_sim.world import 
from pathlib import Path

image_path = Path(__file__).parent / 'image'
gif_path = Path(__file__).parent / 'gif'

world_name = 'multi_robot_systems.yaml'
env = env_base(world_name = world_name, plot=True, robot_init_mode=3)

for i in range(300):

    des_vel_list = [robot.cal_des_vel() for robot in env.robot_list]

    env.robot_step(des_vel_list)
    env.save_fig(image_path, i)
    env.render()
    
    if env.collision_check():
        break

env.save_ani(image_path, gif_path)
env.show()










