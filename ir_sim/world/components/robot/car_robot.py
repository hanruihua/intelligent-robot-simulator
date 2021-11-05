import numpy as np
from math import pi, sin, cos, tan, atan2
from ir_sim.world import motion_ackermann
from collections import namedtuple

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
    
        self.state=init_state
        self.angular_pos()

        self.goal=goal
        self.goal_th = goal_threshold
        self.pre_state = init_state
        self.vel = np.zeros((2, 1))

        self.arrive_flag = False
        self.collision_flag = False

        self.step_time = step_time

    def move_forward(self, vel=np.zeros((2, 1)), stop=True):

        if isinstance(vel, list): 
            vel = np.array(vel, ndmin=2).T

        if stop:
            if self.arrive_flag or self.collision_flag:
                vel = np.zeros((2, 1))

        self.vel = np.clip(vel, np.array([ [-self.v_l], [-self.w_l] ]), np.array([ [self.v_l], [self.w_l] ]))
        self.state = motion_ackermann(self.state, self.wheelbase, self.vel, self.psi_limit, self.step_time)
        self.angular_pos()
    
    def update_state(self, state):

        self.state = state
        self.angular_pos()

    # def state_pre(self, pre_time = 1): 
    #     psi = self.state[3, 0]
    #     vel = self.vel[0, 0]
    #     self.pre_state = motion_acker_pre(self.state, self.wheelbase, vel, psi, self.psi_limit, pre_time, self.step_time)

    def angular_pos(self): 
        # coordinates transform

        x = self.state[0, 0] 
        y = self.state[1, 0] 
        phi = self.state[2, 0] 

        car_x0 = - self.width / 2 
        car_y0 = - (self.length-self.wheelbase)/2

        car_x1 = car_x0 
        car_y1 = car_y0 + self.length

        car_x2 = car_x0 + self.width
        car_y2 = car_y0 + self.length

        car_x3 = car_x0 + self.width
        car_y3 = car_y0

        wheel_x0 = - self.wheelbase_w/2
        wheel_y0 = 0

        wheel_x1 = - self.wheelbase_w/2
        wheel_y1 = self.wheelbase

        wheel_x2 = self.wheelbase_w/2
        wheel_y2 = self.wheelbase

        wheel_x3 = self.wheelbase_w/2
        wheel_y3 = 0

        car_point = np.array([ [car_x0, car_x1, car_x2, car_x3], [car_y0, car_y1, car_y2, car_y3] ])
        wheel_point = np.array([ [wheel_x0, wheel_x1, wheel_x2, wheel_x3], [wheel_y0, wheel_y1, wheel_y2, wheel_y3] ])

        r_phi = phi - pi/2
        rotation_matrix = np.array([[cos(r_phi), -sin(r_phi)], [sin(r_phi), cos(r_phi)]])
        transition_matrix = np.array([[x], [y]])

        self.ang_pos = rotation_matrix @ car_point + transition_matrix
        self.wheel_pos = rotation_matrix @ wheel_point + transition_matrix

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

        segment1 = [self.ang_pos[:, 0], self.ang_pos[:, 1]]
        segment2 = [self.ang_pos[:, 1], self.ang_pos[:, 2]]
        segment3 = [self.ang_pos[:, 2], self.ang_pos[:, 3]]
        segment4 = [self.ang_pos[:, 3], self.ang_pos[:, 0]]

        segment_list = [segment1, segment2, segment3, segment4]

        # check collision with obstacles
        for obs_cir in components['obs_cirs'].obs_cir_list:
            temp_circle = circle(obs_cir.pos[0, 0], obs_cir.pos[1, 0], obs_cir.radius)

            for segment in segment_list:
                if self.collision_circle(segment, temp_circle):
                    self.collision_flag = True
                    print('collisions with obstacles')
                    return True
        
        # check collision with map
        for segment in segment_list:
            if self.collision_matrix(segment, components['map_matrix'], components['xy_reso']):
                self.collision_flag = True
                print('collisions between obstacle map')
                return True

        # check collision with line obstacles:
        for line in components['obs_lines'].line_states:
            segment = [np.array([line[0], line[1]]), np.array([line[2], line[3]])]
            for seg in segment_list:
                if self.collision_segment(segment, seg):
                    print('collisions with line obstacle')
                    return True

    def collision_circle(self, segment, circle):
        
        point = np.array([circle.x, circle.y])
        sp = segment[0]
        ep = segment[1]

        l2 = (ep - sp) @ (ep - sp)

        if (l2 == 0.0):
            distance = np.linalg.norm(point - sp)
            if distance < circle.r:
                return True

        t = max(0, min(1, ((point-sp) @ (ep-sp)) / l2 ))

        projection = sp + t * (ep-sp)
        relative = projection - point

        distance = np.linalg.norm(relative) 
        # angle = atan2( relative[1], relative[0] )
        if distance < circle.r:
            return True 

    def collision_matrix(self, segment, matrix, reso):

        init_point = segment[0]
        len_seg = np.linalg.norm(segment[1] - segment[0])

        slope_cos = (segment[1] - segment[0])[0] / len_seg
        slope_sin = (segment[1] - segment[0])[1] / len_seg

        point_step = reso
        cur_len = 0

        while cur_len <= len_seg:

            cur_point_x = init_point[0] + cur_len * slope_cos
            cur_point_y = init_point[1] + cur_len * slope_sin

            cur_len = cur_len + point_step
            
            index_x = int(cur_point_x / reso)
            index_y = int(cur_point_y / reso)
            if matrix[index_x, index_y]:
                return True    
    
    def collision_segment(self, segment1, segment2):
        # reference https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/; https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        point = namedtuple('point', 'x y')

        p1 = point(segment1[0][0], segment1[0][1])
        p2 = point(segment1[1][0], segment1[1][1])
        q1 = point(segment2[0][0], segment2[0][1])
        q2 = point(segment2[1][0], segment2[1][1])

        o1 = car_robot.orientation(p1, q1, q2)
        o2 = car_robot.orientation(p2, q1, q2)
        o3 = car_robot.orientation(p1, p2, q1)
        o4 = car_robot.orientation(p1, p2, q2)

        # general case
        if o1 != o2 and o3 != o4:
            return True

        # special case
        if o1 == 0 and car_robot.onSegment(p1, q1, p2):
             return True
 
        # p1, q1 and q2 are collinear and q2 lies on segment p1q1
        if (o2 == 0 and car_robot.onSegment(p1, q2, p2)):
            return True
 
        # p2, q2 and p1 are collinear and p1 lies on segment p2q2
        if (o3 == 0 and car_robot.onSegment(q1, p1, q2)):
            return True

        # p2, q2 and q1 are collinear and q1 lies on segment p2q2
        if (o4 == 0 and car_robot.onSegment(q1, p2, q2)):
            return True
 
        return False


    @staticmethod
    def onSegment(p, q, r):

        if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)):
            return True
        
        return False

    @staticmethod
    def orientation(p, q, r):

        # # val = (q.y - p.y) * (r.x - p.x) - (q.x - p.x) * (r.y - q.y)
        # val = (r.y - p.y) * (q.x-p.x) - (q.y - p.y) * (r.x-p.x)
        val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))

        if val > 0:
            return 1
        elif val < 0:
            return 2
        else:
            return 0
        
        # 0 collinear 
        # 1 counterclockwise 
        # 2 clockwise


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

    
    