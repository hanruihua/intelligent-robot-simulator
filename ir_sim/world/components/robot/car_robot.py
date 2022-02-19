import numpy as np
from math import pi, sin, cos, tan, atan2
from ir_sim.world import motion_ackermann, lidar2d
from collections import namedtuple
from ir_sim.util import collision_cir_seg, collision_seg_matrix, collision_seg_seg

class car_robot:
    def __init__(self, id=0, shape = [1.5, 1, 1, 1], init_state=np.zeros((4, 1)), goal = np.zeros((3, 1)), goal_threshold = 0.2, limit=[2, 2], psi_limit = pi/4, step_time=0.1, **kwargs):

        # state: 0, x
        #        1, y
        #        2, phi, heading direction
        #        3, psi, steering angle
        # shape: length, width, wheelbase, wheelbase_w
        # limit: vel_limit, vel_ang_limit

        if isinstance(init_state, list): 
            init_state = np.array(init_state, ndmin=2).T

        if isinstance(goal, list): 
            goal = np.array(goal, ndmin=2).T

        self.id=id

        self.shape = shape
        self.length = shape[0]
        self.width = shape[1]
        self.wheelbase=shape[2]
        self.wheelbase_w= shape[3]
        
        self.v_l = limit[0]
        self.w_l = limit[1]
        self.psi_limit = psi_limit

        self.min_radius= self.wheelbase / tan(psi_limit)

        self.init_state = init_state
        self.state=init_state    
        
        self.init_angular_pos()   # init car model 
        self.init_matrix_model()    # origin of the coordinates

        self.goal=goal
        self.goal_th = goal_threshold
        self.previous_state = init_state
        self.vel = np.zeros((2, 1))

        self.arrive_flag = False
        self.collision_flag = False

        self.step_time = step_time

        self.state_list = []

        lidar_args = kwargs.get('lidar2d', None)

        if lidar_args is not None:
            self.lidar = lidar2d(**lidar_args)
        else:
            self.lidar = None

    # for loop
    def move_forward(self, vel=np.zeros((2, 1)), stop=True, keep=False, **kwargs):

        if isinstance(vel, list): 
            vel = np.array(vel, ndmin=2).T
        if vel.shape == (2,):
            vel = vel[:, np.newaxis]
        
        assert vel.shape == (2, 1)

        if stop:
            if self.arrive_flag or self.collision_flag:
                vel = np.zeros((2, 1))

        if keep:
            self.state_list.append(self.state)

        self.previous_state = self.state
        self.vel = np.clip(vel, np.array([ [-self.v_l], [-self.w_l] ]), np.array([ [self.v_l], [self.w_l] ]))
        self.state = motion_ackermann(self.state, self.wheelbase, self.vel, self.psi_limit, self.step_time, **kwargs)
        self.angular_pos()
        
    def update_state(self, state):
        self.state = state
        self.angular_pos()

    def angular_pos(self): 
        # coordinates transform
        rotation_matrix = np.array([[cos(self.state[2, 0]), -sin(self.state[2, 0] )], [sin(self.state[2, 0]), cos(self.state[2, 0])]])
        transition_matrix = self.state[0:2, 0:1] 

        self.ang_pos = rotation_matrix @ self.init_ang_pos + transition_matrix

    # initialize 
    def init_angular_pos(self): 
        # for car in the origin point (length: x, width: y )
        # car point 1 2 3 4 with anticlockwise
        init_x = (self.length-self.wheelbase)/2
        init_y = self.width/2

        car_point1 = np.array( [ [ -init_x ], [-init_y] ] )
        car_point2 = np.array( [ [ -init_x + self.length ], [-init_y] ] )
        car_point3 = np.array( [ [ -init_x + self.length ], [-init_y + self.width ] ] )
        car_point4 = np.array( [ [ -init_x ], [-init_y + self.width ] ] )

        self.init_ang_pos = np.column_stack( (car_point1, car_point2, car_point3, car_point4) )
        self.ang_pos = self.init_ang_pos

    def init_matrix_model(self):

        self.G = np.zeros(( 4, 2))  # 4 * 2
        self.g = np.zeros(( 4, 1))  # 4 * 1
        
        for i in range(4):

            if i + 1 < 4:
                pre_point = self.init_ang_pos[:, i]
                next_point = self.init_ang_pos[:, i+1]
            else:
                pre_point = self.init_ang_pos[:, i]
                next_point = self.init_ang_pos[:, 0]
            
            diff = next_point - pre_point
            
            a = diff[1]
            b = -diff[0]
            c = a * pre_point[0] + b * pre_point[1]

            self.G[i, 0] = a
            self.G[i, 1] = b
            self.g[i, 0] = c 

        return self.G, self.g

    # 
    def get_trans_matrix(self):
        
        rot = np.array([[cos(self.state[2, 0]), -sin(self.state[2, 0])], [sin(self.state[2, 0]), cos(self.state[2, 0])]])
        trans = self.state[0:2, 0:1]  

        return rot, trans

    def inside(self, point):

        rot = np.array([[cos(self.state[2, 0]), -sin(self.state[2, 0])], [sin(self.state[2, 0]), cos(self.state[2, 0])]])
        trans = self.state[0:2, 0:1] 

        trans_point = np.linalg.inv(rot) @ ( point - trans)
        return (self.G @ trans_point <= self.g).all()

    def arrive(self):
        dis, radian = car_robot.relative(self.state[0:2], self.goal[0:2])

        if dis < self.goal_th:
            self.arrive_flag = True
            return True
        else:
            self.arrive_flag = False
            return False
    
    def cal_des_vel(self, tolerance=0.12):

        dis, radian = car_robot.relative(self.state[0:2], self.goal[0:2])
        car_radian = self.state[2, 0] + self.state[3, 0]

        v_max = self.v_l
        w_max = self.w_l

        diff_radian = car_robot.wraptopi( radian - car_radian )

        if diff_radian > tolerance:
            w_opti = w_max
        
        elif diff_radian < - tolerance:
            w_opti = - w_max   
        else:
            w_opti = 0

        if dis < self.goal_th:
            v_opti = 0
            w_opti = 0
        else:
            v_opti = v_max * cos(diff_radian)
            
            if v_opti < 0:
                v_opti = 0

        return np.array([[v_opti], [w_opti]])

    def collision_check(self, components):

        circle = namedtuple('circle', 'x y r')
        point = namedtuple('point', 'x y')

        segment1 = [point(self.ang_pos[0, 0], self.ang_pos[1, 0]), point(self.ang_pos[0, 1], self.ang_pos[1, 1])]
        segment2 = [point(self.ang_pos[0, 1], self.ang_pos[1, 1]), point(self.ang_pos[0, 2], self.ang_pos[1, 2])]
        segment3 = [point(self.ang_pos[0, 2], self.ang_pos[1, 2]), point(self.ang_pos[0, 3], self.ang_pos[1, 3])]
        segment4 = [point(self.ang_pos[0, 3], self.ang_pos[1, 3]), point(self.ang_pos[0, 0], self.ang_pos[1, 0])]

        segment_list = [segment1, segment2, segment3, segment4]

        # check collision with obstacles
        for obs_cir in components['obs_circles'].obs_cir_list:
            temp_circle = circle(obs_cir.state[0, 0], obs_cir.state[1, 0], obs_cir.radius)
            for segment in segment_list:
                if collision_cir_seg(temp_circle, segment):
                    self.collision_flag = True
                    print('collisions with obstacles')
                    return True
        
        # check collision with map
        for segment in segment_list:
            if collision_seg_matrix(segment, components['map_matrix'], components['xy_reso'], components['offset']):
                self.collision_flag = True
                print('collisions between obstacle map')
                return True

        # check collision with line obstacles:
        for line in components['obs_lines'].obs_line_states:
            seg1 = [point(line[0], line[1]), point(line[2], line[3])]
            for seg2 in segment_list:
                if collision_seg_seg(seg1, seg2):
                    print('collisions with line obstacle')
                    return True
        
        for polygon in components['obs_polygons'].obs_poly_list:
            for edge in polygon.edge_list:
                seg1 = [point(edge[0], edge[1]), point(edge[2], edge[3])]
                for seg2 in segment_list:
                    if collision_seg_seg(seg1, seg2):
                        print('collisions with polygon obstacle')
                        return True
    
    def cal_lidar_range(self, components):
        if self.lidar is not None:
            self.lidar.cal_range(self.state, components)

    @staticmethod
    def relative(state1, state2):
        
        dif = state2[0:2] - state1[0:2]

        dis = np.linalg.norm(dif)
        radian = atan2(dif[1, 0], dif[0, 0])
        
        return dis, radian

    @staticmethod
    def wraptopi(radian):

        if radian > pi:
            radian = radian - 2 * pi
        elif radian < -pi:
            radian = radian + 2 * pi
        
        return radian

    
    