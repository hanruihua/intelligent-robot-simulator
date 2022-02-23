import numpy as np
import cvxpy as cp

def segment_car(segment, car):

    # s = cp.Variable((2, 1))
    theta = cp.Variable()
    p = cp.Variable((2, 1))

    rot, trans = car.get_trans_matrix()

    cur_s = segment.point2 + theta * (segment.point1 - segment.point2)
    cur_p = rot @ p + trans

    cost = 0
    constraints = []
    cost += cp.norm(cur_s-cur_p)

    # constraints += [ segment.A @ s == segment.b   ]
    constraints += [ car.A @ p <= car.b   ]

    # constraints += [ s[0, 0] <= segment.max_x ]
    # constraints += [ s[0, 0] >= segment.min_x ]
    # constraints += [ s[1, 0] <= segment.max_y ]
    # constraints += [ s[1, 0] >= segment.min_y ]
    constraints += [ theta >= 0 ]
    constraints += [ theta <= 1 ]

    prob= cp.Problem(cp.Minimize(cost), constraints)  
    prob.solve() 
         
    if prob.status == cp.OPTIMAL:
        return cur_s.value, cur_p.value
    else:
        print('can not solve', prob.status)
        return None

def segment_car_dual(segment, car):
    pass

    
