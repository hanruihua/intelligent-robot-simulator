from ir_sim.env import env_base

world_name = 'robot_world.yaml'
env = env_base(world_name = world_name, plot=True, robot_mode='diff')

for i in range(300):

    des_vel = env.robot.cal_des_vel()

    env.robot_step(des_vel)
    env.render()

    










