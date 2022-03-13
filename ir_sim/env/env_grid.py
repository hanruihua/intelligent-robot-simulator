import matplotlib.pyplot as plt

class env_grid:
    def __init__(self, grid_map_matrix=None, reward_matrix=None, state_prob=1):
        self.matrix = grid_map_matrix

        self.state_space = grid_map_matrix.shape
        self.action_space = [ (1, 0), (-1, 0), (0, 1), (0, -1) ]
        self.state_prob = state_prob
        self.reward_matrix = reward_matrix
        self.reward_bound = -5

        
    def draw_map(self, time=0.01):
        plt.imshow(self.matrix, extent=[0, self.width, 0, self.height])
        plt.pause(time) 

    
    

