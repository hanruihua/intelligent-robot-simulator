import numpy as np
from math import sqrt, pi, cos, sin

# collision detection

## circle:  x, y, r
## segment: [point1, point2]
## point: x, y
## point_set: 2*n, matrix 
# rectangle: 

def collision_cir_cir(circle1, circle2):

    dis = sqrt( (circle2.x - circle1.x)**2 + (circle2.y - circle1.y)**2 )

    if dis >0 and dis <= circle1.r + circle2.r:
        return True
    
    return False

def collision_cir_matrix(circle, matrix, reso, offset=np.zeros(2,)):

    if matrix is None:
        return False

    rad_step = 0.1
    cur_rad = 0

    while cur_rad <= 2*pi:
        crx = circle.x + circle.r * cos(cur_rad)
        cry = circle.y + circle.r * sin(cur_rad)
        cur_rad = cur_rad + rad_step
        index_x = int( (crx - offset[0]) / reso)
        index_y = int( (cry - offset[1]) / reso)
        if matrix[index_x, index_y]:
            return True

def collision_cir_seg(circle, segment):
    
    # reference: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    
    point = np.array([circle.x, circle.y])
    sp = np.array([segment[0].x, segment[0].y])
    ep = np.array([segment[1].x, segment[1].y])
    
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

def collision_seg_matrix(segment, matrix, reso, offset=np.zeros(2,)):

    if matrix is None:
        return False

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
        
        index_x = int( (cur_point_x - offset[0]) / reso)
        index_y = int( (cur_point_y - offset[1]) / reso)

        if index_x < 0 or index_x > matrix.shape[0] or index_y < 0 or index_y > matrix.shape[1]:
            return True

        if matrix[index_x, index_y]:
            return True    

def collision_circle_point(circle, point_set):
    
    assert point_set.shape[0] == 2

    center = np.array([ [circle.x], [circle.y] ]) # 2*1
    temp = point_set - center

    dis_set = np.linalg.norm(temp, axis=0)

    min_dis = np.min(dis_set)

    return min_dis < circle.r

# def collision_rect_point(rectangle, point_set):
    
#     assert point_set.shape[0] == 2

#     center = np.array([ [circle.x], [circle.y] ]) # 2*1
#     temp = point_set - center

#     dis_set = np.linalg.norm(temp, axis=0)

#     min_dis = np.min(dis_set)

#     return min_dis < circle.r
        
def collision_seg_seg(segment1, segment2):
    # reference https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/; https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

    p1, p2, q1, q2 = segment1[0], segment1[1], segment2[0], segment2[1]

    o1 = orientation(p1, q1, q2)
    o2 = orientation(p2, q1, q2)
    o3 = orientation(p1, p2, q1)
    o4 = orientation(p1, p2, q2)

    # general case
    if o1 != o2 and o3 != o4:
        return True

    # special case
    if o1 == 0 and onSegment(p1, q1, p2):
            return True

    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 and onSegment(p1, q2, p2)):
        return True

    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 and onSegment(q1, p1, q2)):
        return True

    # p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 and onSegment(q1, p2, q2)):
        return True

    return False

def onSegment(p, q, r):

    if (q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y)):
        return True
    
    return False

def orientation(p, q, r):

    # 0 collinear 
    # 1 counterclockwise 
    # 2 clockwise

    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))

    if val > 0:
        return 1
    elif val < 0:
        return 2
    else:
        return 0
    
    
 
   

