from ir_sim.env import env_base
from pathlib import Path

world_name = 'dynamic_obs.yaml'
env = env_base(world_name = world_name, teleop_key=True)

image_path = Path(__file__).parent / 'image'
gif_path = Path(__file__).parent / 'gif'

# render: show_traj, traj_type, show_lidar, goal_color,

while True:

    env.car_step(env.key_vel, ack_mode='steer')

    env.render(show_traj=True, show_lidar=True, show_goal=False)
    # env.save_fig(image_path, i) 

    env.obs_cirs_step()
    if env.collision_check():
        break

# env.save_ani(image_path, gif_path)
env.show()


