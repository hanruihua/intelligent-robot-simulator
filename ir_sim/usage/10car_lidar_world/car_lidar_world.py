from ir_sim.env import env_base
# from ir_sim.world import 

world_name = 'car_lidar_world.yaml'
env = env_base(world_name = world_name, plot=True, init_mode=0, robot_mode='diff')

for i in range(300):

    des_vel = env.car.cal_des_vel()
    env.car_step(des_vel)
    env.render()

    if env.collision_check():
        break

env.show()


