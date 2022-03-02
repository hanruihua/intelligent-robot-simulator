from ir_sim.env import env_base
# from ir_sim.world import 
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from ir_sim.util.min_dis_opt import segment_car, segment_car_dual, segment_car_dual2, segment_car_dual3

world_name = 'mouse_test.yaml'
env = env_base(world_name = world_name, plot=True, teleop_key=True)

def on_move(event):

    if event.inaxes:
        ax2 = event.inaxes  # the axes instance

        x = event.xdata
        y = event.ydata

        point = np.array( [ [x], [y] ] )

        # for polygon in env.obs_poly_list:

        #     flag, temp = polygon.inside(point)
        #     if flag:
        #         print(flag)
        
        # for circle in env.obs_cir_list:
        #     flag = circle.inside(point)
        #     # m1, m2, m3 = circle.min_distance(point)
        #     if flag:
        #         print(flag)
        # print(env.car.inside(point))

        # for line in env.obs_line_list:
        #     flag = line.inside(point)

        #     print(flag)
  
    # env.render(time=0.0000001)
        # print('data coords %f %f' % (event.xdata, event.ydata))

def press(event):
    ax2 = event.inaxes  # the axes instance

    x = event.xdata
    y = event.ydata



    print(x, y)


# binding_id = plt.connect('motion_notify_event', on_move)
# binding_id = plt.connect('button_press_event', press)
# binding_id = plt.connect('figure_enter_event', on_move)
# env.show()
    
draw_func = env.world_plot.draw_trajectory

for i in range(1000):
    env.car_step(env.key_vel, env.key_id, ack_mode='steer')
    # s_list = []
    # p_list = []
    # draw_kwargs_list=[]

    # point1_list = [line.point1 for line in env.obs_line_list ]
    # point2_list = [line.point2 for line in env.obs_line_list ]

    # point1_array = np.hstack(point1_list)
    # point2_array = np.hstack(point2_list)

    # min_dis = segments_car(point1_array, point2_array, env.car)
    
    dis_dual_list = []
    dis_list = []

    for line in env.obs_line_list:
        s, p = segment_car(line, env.car)
        # dis_dual = segment_car_dual(line, env.car)
        dis_dual = segment_car_dual3(line, env.car)
        dis = np.linalg.norm(s-p)
        # s_list.append(s)
        # p_list.append(p)
        # print(dis)
        dis_dual_list.append(dis_dual[0,0])
        dis_list.append(dis)
        
        # draw_kwargs_list.append( { 'traj': [s, p], 'refresh':True} )
    print('primal', min(dis_list))
    print('dual', min(dis_dual_list))
    print('dif', min(dis_list) - min(dis_dual_list) )
    # env.render(time=0.01, draw_func_list=[draw_func, draw_func], draw_kwargs_list=draw_kwargs_list)
    env.render()


   
    




