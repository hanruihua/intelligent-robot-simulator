from ir_sim.env import env_base
# from ir_sim.world import 
import matplotlib.pyplot as plt

world_name = 'teleop_key.yaml'
env = env_base(world_name = world_name, plot=True, teleop_key=True)

for i in range(300):

    # print(env.key_vel)
    env.robot_step(env.key_vel, env.key_id)
    env.render()
    
    env.collision_check()

env.show()







