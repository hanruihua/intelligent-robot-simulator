from ir_sim.env import env_base
from pathlib import Path

world_name = 'render.yaml'
env = env_base(world_name = world_name, plot=True, init_mode=0, robot_mode='diff', teleop_key=True)

image_path = Path(__file__).parent / 'image'
gif_path = Path(__file__).parent / 'gif'

# render: show_traj, traj_type, show_lidar, goal_color,

for i in range(300):

    env.car_step(env.key_vel, ack_mode='steer')

    env.render(show_traj=True, show_lidar=False)

    # env.save_fig(image_path, i) 

    if env.collision_check() or env.arrive_check():
        break

# env.save_ani(image_path, gif_path)
env.show()


