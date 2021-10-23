import threading
import time

from ir_env import env_base

# env0 = env_base('ir_env/world/world_test.yaml')
env0 = env_base('E:\intelligent-robot\ir_env\world\world_test.yaml')
env0.initialization()
ani = True

def main_loop(max_step=100, ani=False):
    step = 0
    while step < max_step: 
        step+=1
        start = time.time()

        vel_list = env0.cal_des_list()
        env0.step(vel_list)

        if env0.arrive_all():
            print('done')
            break
            # step += max_step/20

        if ani:
            env0.step_time_adjust(start)

thread_ani = threading.Thread(target=main_loop, args=(100, ani))
thread_ani.start()

# show
env0.show_ani()

# save
# env0.save_ani()









