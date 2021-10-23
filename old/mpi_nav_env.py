from mpi4py import MPI
from robot_model import mobile_robot
from environment import obstacle
import numpy as np
from algos import reciprocal_vel_obs
import heapq

class mpi_nav_env():
    def __init__(self, obstacles=None):

        self.comm = MPI.COMM_WORLD
        self.num_robot = self.comm.Get_size()
        self.rank = self.comm.Get_rank()
        self.robot = None
        self.obstacles = obstacles

    def initialize(self, robot_mode='diff', step_time=0.1, sg_kwargs=dict(), con_kwargs=dict()):

        init_state, goal_state = self.start_goal(**sg_kwargs)
        self.robot = mobile_robot(self.rank, mode = robot_mode, init_state=init_state, goal = goal_state, **con_kwargs) 
    
    def start_goal(self, start_mode=0, interval=1, upper=8, lower=0):

        if start_mode == 0:

            if self.num_robot % 2 == 0:
                half = self.num_robot // 2
            else:
                half = self.num_robot // 2 + 1
            
            if self.rank < half:
                x, y, bear = self.rank * interval, lower, 0
                goal_x, goal_y = (half-self.rank-1) * interval, upper 
            else:
                x, y, bear = (self.rank -half) * interval, upper, 0
                goal_x, goal_y = (2 * half - self.rank-1) * interval, lower

            init_state = np.array([[x], [y], [bear]])
            goal_state = np.array([[goal_x], [goal_y]])

        return init_state, goal_state

    def move_forward(self, vel, **move_kwargs):
        self.robot.move_forward(vel, **move_kwargs)

    def move_direct(self):
        self.robot.move_to_goal()

    def total_state(self, neighbors_region=7, neighbors_num=7):
        moving_states = self.comm.allgather(self.robot.omni_obs_state())
        del moving_states[self.rank]
        robot_state = self.robot.omni_state()

        def distance(state):
            mp = state[0:2]
            rp = robot_state[0:2]
            return np.linalg.norm(mp-rp)

        filter_moving_states = list(filter(lambda m: distance(m) <= neighbors_region, moving_states))

        if len(filter_moving_states) > neighbors_num:
            moving_list = heapq.nsmallest(neighbors_num, filter_moving_states, key=lambda m: distance(m))
        else:
            moving_list = sorted(filter_moving_states, key=lambda m: distance(m))

        total_state = [robot_state, moving_list]
        
        return total_state

    def cur_position(self):
        positions = self.comm.allgather(self.robot.state[0:2])
        return positions

    def robot_list(self):
        # robot_list = self.comm.allgather(self.robot)
        robot_list = self.comm.gather(self.robot, root=0)
        return robot_list

    def arrive(self):
        arrive_list = self.comm.allgather(self.robot.arrive())
        return min(arrive_list)

    # def total_states(self, neighbors_region=5, neighbors_num=5):
        
    #     total_multi_moving_list = list(map(lambda r: np.squeeze(r.omni_obs_state()), self.robot_list)) # all robot observed state
    #     robot_total_state_list = []
    #     for i in range(self.num_robot):

    #         robot_state = np.squeeze(self.robot_list[i].omni_state())  

    #         # calculate moving obstacles list 
    #         multi_moving_list = total_multi_moving_list[:]  
    #         del multi_moving_list[i]

    #         def rs_function(moving_state):
    #             return multi_mobile_robot.dis_point(moving_state, robot_state)

    #         filter_moving_list = list(filter(lambda m: rs_function(m) <= neighbors_region, multi_moving_list))

    #         if len(filter_moving_list) > neighbors_num:
    #             moving_list = heapq.nsmallest(neighbors_num, filter_moving_list, key=lambda m: rs_function(m))
    #         else:
    #             # moving_list = filter_moving_list[:]
    #             moving_list = sorted(filter_moving_list, key=lambda m: rs_function(m))
            
    #         # calculate statics obstacles list
    #         filter_obs_list = list(filter(lambda o: rs_function(o) <= neighbors_region, self.obstacles_list))

    #         robot_total_state_list.append([robot_state, moving_list, filter_obs_list])

    #     return robot_total_state_list


    
