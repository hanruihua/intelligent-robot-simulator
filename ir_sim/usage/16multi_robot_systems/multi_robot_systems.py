from ir_sim.env import env_base
# from ir_sim.world import 

world_name = 'multi_robot_systems.yaml'
env = env_base(world_name = world_name, plot=True, robot_init_mode=3)

for i in range(300):

    des_vel_list = [robot.cal_des_vel() for robot in env.robot_list]

    env.robot_step(des_vel_list)
    env.render()
    
    env.collision_check()










