import numpy as np

class obs_line:
    def __init__(self, id=0, line_state=None):
        self.line_state = line_state
        self.point1 = np.array([ [line_state[0]], [line_state[1]] ])
        self.point2 = np.array([ [line_state[2]], [line_state[3]] ])
        
        self.init_matrix()

    def init_matrix(self):
        pass
        
        

    
    


        