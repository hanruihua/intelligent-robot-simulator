import numpy as np

class obs_line:
    def __init__(self, id=0, line_state=None, **kwargs):
        self.line_state = line_state
        self.point1 = np.array([ [line_state[0]], [line_state[1]] ])
        self.point2 = np.array([ [line_state[2]], [line_state[3]] ])
        
        self.init_matrix()

    def init_matrix(self):
        # Ax == b
        diff = self.point2 - self.point1

        a = diff[1, 0]
        b = -diff[0, 0]
        c = a * self.point1[0, 0] + b * self.point1[1, 0]

        self.A = np.array( [ [a, b] ] )
        self.b = c
        self.min_x = np.min(self.point1[0, 0], )

    def inside(self, point):
        assert point.shape == (2, 1)
        return np.squeeze(self.A) @ point == self.b

        
        

    
    


        