import numpy as np

class obs_line:
    def __init__(self, id=0, line_state=None, **kwargs):
        self.line_state = line_state
        self.point1 = np.array([ [line_state[0]], [line_state[1]] ])
        self.point2 = np.array([ [line_state[2]], [line_state[3]] ])
        
        self.min_x = np.min([self.point1[0, 0], self.point2[0, 0] ])
        self.max_x = np.max([self.point1[0, 0], self.point2[0, 0] ])

        self.min_y = np.min([ self.point1[1, 0], self.point2[1, 0] ])
        self.max_y = np.max([self.point1[1, 0], self.point2[1, 0]] )

        self.cone = 'R_positive'

        self.init_matrix()

    def init_matrix(self):
        # Ax == b
        diff = self.point2 - self.point1

        a = diff[1, 0]
        b = -diff[0, 0]
        c = a * self.point1[0, 0] + b * self.point1[1, 0]

        self.A = np.array( [ [a, b] ] )
        self.b = c
        
    def inside(self, point):
        assert point.shape == (2, 1)
        return np.squeeze(self.A) @ point == self.b
    
    def inside2(self, point):
        assert point.shape == (2, 1)
        # theta = 

        return np.squeeze(self.A) @ point == self.b

        
        

    
    


        