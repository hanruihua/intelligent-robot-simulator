from ir_sim.env import env_base
# from ir_sim.world import 
import time

world_name = 'polygon_car.yaml'
env = env_base(world_name = world_name, plot=True, init_mode=0, robot_mode='diff', teleop_key=True)

for i in range(300):
    
    # env.robot_step(env.key_vel, env.key_id)
    env.car_step(env.key_vel, ack_mode='steer')
    env.render()

    start_time = time.time()
    if env.collision_check():
        break
    print(time.time() - start_time)

env.show()

    


    
    






