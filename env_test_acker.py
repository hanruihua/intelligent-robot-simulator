from pathlib import Path
from ir_env import env_base


# env0 = env_base('ir_env/world/world_test.yaml')
cur_path = Path(__file__).parent

# figure_path = cur_path / 'figure'
# ani_path = cur_path/'animation'

world_path = str(cur_path/'world'/'world_test_acker.yaml')

env0 = env_base(world_path)
env0.initialization()

max_step = 600
step = 0

# env0.car_list[0].state_pre(vel=np.array([[2], [0]]), psi = 1, pre_time = 2)

while step < max_step: 
    step+=1
    env0.render_cla(0.1, pre_state=True)
    # env0.world.save_gif_figure(figure_path, step)
    vel_list = env0.cal_des_car_list()
    env0.step(vel_car_list=vel_list)

    if env0.arrive_all():
        print('done')
        break

env0.show()
# env0.world.create_animate(figure_path, ani_path, ani_name='ani_test', rm_fig_path=True)
# env0.world.show()
        













