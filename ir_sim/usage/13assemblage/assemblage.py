from ir_sim.env import env_base
from pathlib import Path

world_name = 'assemblage.yaml'
env = env_base(world_name = world_name, plot=True, init_mode=0, robot_mode='diff', teleop_key=True)

image_path = Path(__file__).parent / 'image'
gif_path = Path(__file__).parent / 'gif'

for i in range(300):

    # des_vel = env.car.cal_des_vel()

    env.car_step(env.key_vel, ack_mode='steer')
    # env.robot_step(env.key_vel)
    env.render()

    # env.save_fig(image_path, i) 

    if env.collision_check() or env.arrive_check():
        break

# env.save_ani(image_path, gif_path)
env.show()


