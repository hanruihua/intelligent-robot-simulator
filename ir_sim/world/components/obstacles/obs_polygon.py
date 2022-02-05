from re import I
import numpy as np

class obs_polygon:
    def __init__(self, vertex = None, **kwargs):
        self.vertexes = np.array(vertex).T  # 2*n
        self.ver_num = self.vertexes.shape[1]

        self.gen_edges()
        self.gen_matrix()

    def gen_edges(self):

        self.edge_list = []

        for i in range(self.ver_num-1):
            edge = [self.vertexes[0, i], self.vertexes[1, i], self.vertexes[0, i+1], self.vertexes[1, i+1]]
            self.edge_list.append(edge)
        
        edge_final = [ self.vertexes[0, self.ver_num-1], self.vertexes[1, self.ver_num-1], self.vertexes[0, 0], self.vertexes[1, 0] ]

        self.edge_list.append(edge_final)


        
    def gen_matrix(self):
        pass

    
    

        # p = Polygon(y, facecolor = 'k')
        # ax.add_patch(p)


