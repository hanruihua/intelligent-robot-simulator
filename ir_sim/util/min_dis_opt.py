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
    lam = cp.Variable((4, 1), nonneg=True)
    mu1 = cp.Variable(nonneg=True)
    mu2 = cp.Variable(nonneg=True)

    rot, trans = car.get_trans_matrix()

    cost = 0
    constraints = []

    cost+= -lam.T @ car.b - mu1 + lam.T @ car.A @ rot.T @ (segment.point2 - trans)
    constraints += [ cp.norm( lam.T @ car.A @ rot.T  ) <= 1 ]
    constraints += [ cp.constraints.zero.Zero( mu1 - mu2 + lam.T @ car.A @ rot.T @ ( segment.point1 - segment.point2 ) ) ]

    prob= cp.Problem(cp.Maximize(cost), constraints)  
    prob.solve() 

    if prob.status == cp.OPTIMAL:
        dis = -lam.value.T @ car.b - mu1.value + lam.value.T @ car.A @ rot.T @ (segment.point2 - trans)
        return dis
    else:
        print('can not solve', prob.status)
        return None

def segment_car_dual2(segment, car):
    lam = cp.Variable((4, 1), nonneg=True)
    mu2 = cp.Variable(nonneg=True)
    v =  cp.Variable((2, 1))

    rot, trans = car.get_trans_matrix()

    cost= 0
    constraints=[]

    cost+= -lam.T @ car.b - mu2 + v.T @ trans - v.T @ segment.point2

    constraints += [cp.norm(v) <= 1]
    constraints += [lam.T @ car.A + v.T @ rot == 0]
    constraints += [mu2 - v.T@(segment.point1 - segment.point2) >= 0]
    
    prob=cp.Problem(cp.Maximize(cost), constraints)  
    prob.solve() 

    if prob.status == cp.OPTIMAL:
        dis = -lam.value.T @ car.b - mu2.value + v.value.T @ trans - v.value.T @ segment.point2
        return dis
    else:
        print('can not solve', prob.status)
        return None

def segment_car_dual3(segment, car):
    lam = cp.Variable((4, 1), nonneg=True)
    v =  cp.Variable((2, 1))
    mu2 = cp.Variable(nonneg=True)

    rot, trans = car.get_trans_matrix()

    cost= 0
    constraints=[]

    cost+= -lam.T @ car.b + v.T @ trans - v.T @ segment.point2 - mu2

    constraints += [cp.norm(v) <= 1]
    constraints += [lam.T @ car.A + v.T @ rot == 0]
    constraints += [mu2 - v.T @ segment.point1 + v.T @ segment.point2 >= 0]

    # constraints += [mu2 - v.T@(segment.point1 - segment.point2) >= 0]
    
    prob=cp.Problem(cp.Maximize(cost), constraints)  
    prob.solve() 

    if prob.status == cp.OPTIMAL:
        dis = -lam.value.T @ car.b + v.value.T @ trans - v.value.T @ segment.point2 - mu2.value
        return dis
    else:
        print('can not solve', prob.status)
        return None

# def segments_car(point1_array, point2_array, car):
    
#     pass

    


    
