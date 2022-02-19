import numpy as np
from ir_sim.world import motion_omni
from math import sin, cos, atan2
# import cvxpy

class obs_circle:
    def __init__(self, id=0, state=np.zeros((2, 1)), radius=0.2, velocity=np.zeros((2, 1)), vel_max=2 * np.ones((2, 1)), 
     step_time=0.1, obs_model='static', goal=np.zeros((2, 1)), goal_threshold=0.1, **kwargs):
        
        if isinstance(state, list):
            state=np.array(state, ndmin=2).T   
        
        if isinstance(velocity, list):
            velocity=np.array(velocity, ndmin=2).T 

        if isinstance(goal, list): 
            goal = np.array(goal, ndmin=2).T

        if isinstance(vel_max, list): 
            vel_max = np.array(vel_max, ndmin=2).T 

        self.id = id
        self.state = state
        self.radius = radius
        self.vel_omni = velocity
        self.vel_max = vel_max
        self.step_time = step_time
        self.obs_model = obs_model # static, dynamic
        self.goal = goal
        self.goal_threshold = goal_threshold
        self.radius_collision = round(radius + kwargs.get('radius_exp', 0.1), 2)
        self.arrive_flag = False
        
        # obstacle model, generalized inequality, Ax >=_k b
        self.A = np.array([ [1, 0], [0, 1], [0, 0] ])
        self.b = np.row_stack((self.state, self.radius * np.ones((1,1))))
        self.b_collision = np.row_stack((self.state, self.radius_collision * np.ones((1,1))))

    def inside(self, point):

        # generalized inequality over the norm cone
        # x<=_k x_c

        assert point.shape == (2, 1)

        return self.norm_cone( self.b - self.A @ point)
    
    def inside_collision(self, point):

        assert point.shape == (2, 1)

        return self.norm_cone( self.b_collision - self.A @ point)

    def norm_cone(self, matrix):

        assert matrix.shape == (3, 1)

        return np.linalg.norm(matrix[0:2]) <= matrix[2, 0]

    # # test dual 
    # def min_distance(self, point):
    #     min_distance1 = np.linalg.norm(point - self.state) - self.radius

    #     ind_t = cvxpy.Variable((2, 1))

    #     cost = 0
    #     constraints = []
    #     constraints_p = []

    #     cost += cvxpy.norm(ind_t) 
    #     constraints +=  [  cvxpy.norm(point + ind_t - self.state) <= self.radius ]
    #     prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    #     prob.solve() 

    #     if prob.status == cvxpy.OPTIMAL:
    #         min_distance2 = np.linalg.norm(ind_t.value)
    #     else:
    #         print('can not solve')

    #     cost2 = 0
    #     constraints2 = []

    #     ind_lambda = cvxpy.Variable((3, 1))
    
        
    #     cost2 += ind_lambda.T @ ( self.A @ point - self.b)

    #     constraints2 += [cvxpy.norm(self.A.T @ ind_lambda) <= 1]
    #     # constraints2 += [cvxpy.norm(ind_lambda) <= self.radius]
    #     constraints2 += [ cvxpy.norm(ind_lambda[0:2]) <= ind_lambda[2, 0] ]
    #     # constraints2 += [ind_lambda >= 0]

    #     prob2 = cvxpy.Problem(cvxpy.Maximize(cost2), constraints2)
    #     prob2.solve() 
        
    #     if prob2.status == cvxpy.OPTIMAL:
    #         min_distance3 = (self.A @ point -  self.b).T @ ind_lambda.value
    #     else:
    #         print('can not solve')

    #     return min_distance1, min_distance2, min_distance3
        
    def move_forward(self, vel, stop=True, **vel_kwargs):
        
        if isinstance(vel, list): 
            vel = np.array(vel, ndmin=2).T

        if vel.shape == (2,):
            vel = vel[:,np.newaxis]

        if stop:
            if self.arrive():
                vel = np.zeros((2, 1))

        self.state = motion_omni(self.state, vel, self.step_time, **vel_kwargs)
        self.vel_omni = vel

        self.b[0:2, 0] = self.state[:, 0]
        self.b_collision[0:2, 0] = self.state[:, 0]
    
    def state_predict_b(self, receding=5):
        cur_state = self.state
        pre_array = np.zeros((2, receding+1))
        pre_array[:, 0] = cur_state[:, 0]
        b_array = np.zeros((3, receding+1))
        b_array[:, 0] = self.b[:, 0]

        for i in range(receding):
            cur_state = motion_omni(cur_state, self.vel_omni, self.step_time)
            pre_array[:, i+1] = cur_state[:, 0]
            b_array[:, i+1:i+2] = np.row_stack((cur_state, self.radius * np.ones((1,1))))

        return b_array
    # def omni_obs_state(self):
        
    #     x = self.state[0, 0]
    #     y = self.state[1, 0]

    #     vx = self.vel_omni[0, 0]
    #     vy = self.vel_omni[1, 0]

    #     radius = self.radius_collision

    #     return [x, y, vx, vy, radius]

    def omni_obs_state(self):
        rc_array = self.radius * np.ones((1,1))    
        return np.concatenate((self.state, self.vel_omni, rc_array), axis = 0)
    
    def omni_state(self):

        x = self.state[0, 0]
        y = self.state[1, 0]

        vx = self.vel_omni[0, 0]
        vy = self.vel_omni[1, 0]

        radius = self.radius_collision

        vx_des, vy_des = self.cal_des_vel_omni(self.vel_max)

        return [x, y, vx, vy, radius, vx_des, vy_des] 

    def cal_des_vel_omni(self, v_max):

        dis, radian = self.relative(self.state, self.goal)

        if dis > self.goal_threshold:

            vx = v_max[0, 0] * cos(radian)
            vy = v_max[1, 0] * sin(radian)
        
        else:
            vx = 0
            vy = 0

        return vx, vy
    
    def relative(self, state1, state2):

        dis = np.sqrt( (state1[0, 0] - state2[0, 0])**2 + (state1[1, 0] - state2[1, 0])**2 )
        radian = atan2(state2[1, 0] - state1[1, 0], state2[0, 0] - state1[0, 0])

        return dis, radian 

    def arrive(self):

        dist = np.linalg.norm(self.state[0:2] - self.goal[0:2]) 

        if dist < self.goal_threshold:
            self.arrive_flag = True
            return True
        else:
            self.arrive_flag = False
            return False 