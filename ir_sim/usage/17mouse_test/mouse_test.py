from ir_sim.env import env_base
# from ir_sim.world import 
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


world_name = 'mouse_test.yaml'
env = env_base(world_name = world_name, plot=True)



def on_move(event):

    if event.inaxes:
        ax2 = event.inaxes  # the axes instance

        x = event.xdata
        y = event.ydata

        point = np.array( [ [x], [y] ] )

        for polygon in env.obs_poly_list:

            flag, temp = polygon.inside(point)
            # print(temp)
            # print(point)
            # flag, temp = polygon.inside_collision(point)
            print(flag)

        # print('data coords %f %f' % (event.xdata, event.ydata))

binding_id = plt.connect('motion_notify_event', on_move)
env.show()



