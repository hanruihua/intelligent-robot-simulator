from ir_sim.env import env_base

world_name = 'car_world.yaml'
env = env_base(world_name = world_name, plot=True, init_mode=0, robot_mode='diff')

for i in range(300):
    
    des_vel = env.car.cal_des_vel()
    env.car.move_forward(des_vel)
    # print(i)
    env.render(0.001)
env.show()

    










