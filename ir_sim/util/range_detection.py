import numpy as np
from math import sqrt, pi, cos, sin

def range_seg_matrix(segment, matrix, reso):

    init_point = segment[0]
    dif_x = segment[1].x - segment[0].x
    dif_y = segment[1].y - segment[0].y

    len_seg = sqrt( dif_x**2 + dif_y**2 )

    slope_cos = dif_x / len_seg
    slope_sin = dif_y / len_seg

    point_step = 2*reso
    cur_len = 0

    while cur_len <= len_seg:

        cur_point_x = init_point.x + cur_len * slope_cos
        cur_point_y = init_point.y + cur_len * slope_sin

        cur_len = cur_len + point_step
        
        index_x = int(cur_point_x / reso)
        index_y = int(cur_point_y / reso)

        if matrix[index_x, index_y]:
            int_point = np.array([cur_point_x, cur_point_y])
            lrange = np.linalg.norm( int_point -  init_point)

            return True, int_point, lrange
    
    return False, None, None


def range_cir_seg(circle, segment):
    
    # reference: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    
    point = np.array([circle.x, circle.y])
    sp = np.array([segment[0].x, segment[0].y])
    ep = np.array([segment[1].x, segment[1].y])
    
    l2 = (ep - sp) @ (ep - sp)

    if (l2 == 0.0):
        lrange = np.linalg.norm(point - sp)
        if lrange <= circle.r:
            return True, sp, lrange

    t = max(0, min(1, ((point-sp) @ (ep-sp)) / l2 ))

    int_point = sp + t * (ep-sp)
    relative = int_point - point

    lrange = np.linalg.norm(relative) 
    # angle = atan2( relative[1], relative[0] )
    if lrange <= circle.r:
        return True, int_point, lrange


def range_seg_seg(segment1, segment2):
    # reference https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    # p, p+r, q, q+s

    p = np.array([segment1[0].x, segment1[0].y])
    r = np.array([segment1[1].x - segment1[0].x, segment1[1].y - segment1[0].y])
    q = np.array([segment2[0].x, segment2[0].y])
    s = np.array([segment2[1].x - segment2[0].x, segment2[1].y - segment2[0].y])

    temp1 = np.cross(r, s)
    temp2 = np.cross(q-p, r)

    if temp1 == 0 and temp2 == 0:
        # collinear
        t0 = (q-p) @ r / (r @ r)
        t1 = t0 + s @ r / (r @ r)

        if max(t0, t1) >= 0 and min(t0, t1) < 0:
            int_point = p
            lrange = 0
            return True, int_point, lrange
        
        elif min(t0, t1) >=0 and min(t0, t1) <= 1:
            int_point = p + min(t0, t1) * r
            lrange = np.linalg.norm(int_point - p)
            return True, int_point, lrange

        else:
            return False, None, None
    
    elif temp1 == 0 and temp2 != 0:
        # parallel and non-intersecting
        return False, None, None
    elif temp1 != 0:

        t = np.cross( q-p, s) / np.cross(r, s)
        u = np.cross( q-p, r) / np.cross(r, s)

        if t >=0 and t<=1 and u>=0 and u<= 1:

            int_point = p + t*r
            lrange = np.linalg.norm(int_point - p)
            return True, int_point, lrange
    else:
        # not parallel and not intersect
        False, None, None



