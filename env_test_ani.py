import numpy as np

from pathlib import Path
from ir_env import env_base

cur_path = Path(__file__).parent
env0 = env_base(cur_path/'world/world_test.yaml')
env0.initialization()
figure_path = cur_path / 'figure'
ani_path = cur_path/'animation'

max_step = 300
step = 0

while step < max_step: 
    step+=1
    env0.render_cla(0.05)
    # env0.world.save_gif_figure(figure_path, step)
    vel_list = env0.cal_des_list()
    env0.step(vel_list)

    if env0.arrive_all():
        print('done')
        break

# env0.world.create_animate(figure_path, ani_path, ani_name='ani_test', rm_fig_path=True)
# env0.world.show()
        













