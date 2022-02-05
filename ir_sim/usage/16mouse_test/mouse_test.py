from ir_sim.env import env_base
# from ir_sim.world import 
import matplotlib.pyplot as plt
import matplotlib as mpl


world_name = 'mouse_test.yaml'
env = env_base(world_name = world_name, plot=True)

def on_move(event):
    # get the x and y pixel coords
    # x, y = event.x, event.y
    if event.inaxes:
        ax2 = event.inaxes  # the axes instance
        # obs_circle = mpl.patches.Circle(xy=(event.xdata, event.ydata), radius = 0.2)
        # plt.pause(0.01)
        # ax.add_patch(obs_circle)
        # ax.clear()
        print('data coords %f %f' % (event.xdata, event.ydata))

binding_id = plt.connect('motion_notify_event', on_move)

# plt.connect('button_press_event', on_click)
env.show()

# for i in range(300):

#     # print(env.key_vel)
#     # env.robot_step(env.key_vel)
#     env.render()
    
#     # env.collision_check()


