from re import I
import numpy as np

class obs_polygon:
    def __init__(self, vertex = None, collision_thick=1, **kwargs):
        self.vertexes = np.array(vertex).T  # 2*n
        self.ver_num = self.vertexes.shape[1]
        self.collision_thick = collision_thick

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

        self.A = np.zeros(( self.ver_num, 2))  # n * 2
        self.b = np.zeros(( self.ver_num, 1))  # n * 1
        self.b_collision = np.zeros(( self.ver_num, 1))  # n * 1

        for i in range(self.ver_num):

            if i + 1 < self.ver_num:
                pre_point = self.vertexes[:, i]
                next_point = self.vertexes[:, i+1]
            else:
                pre_point = self.vertexes[:, i]
                next_point = self.vertexes[:, 0]
            
            diff = next_point - pre_point

            a = diff[1]
            b = -diff[0]
            c = a * pre_point[0] + b * pre_point[1]

            self.A[i, 0] = a
            self.A[i, 1] = b
            self.b[i, 0] = c 

            if b != 0:
                self.b_collision[i, 0] = c + self.collision_thick * abs(b)
            else:
                self.b_collision[i, 0] = c + self.collision_thick * abs(a)

        return self.A, self.b
    
    def inside(self, point):

        assert point.shape == (2, 1)
        temp = self.A @ point - self.b
        return  (self.A @ point < self.b).all(), temp
            
    def inside_collision(self, point):

        assert point.shape == (2, 1)
        temp = self.A @ point - self.b_collision
        return  (self.A @ point < self.b_collision).all(), temp
        
        






        
        


    
    

        # p = Polygon(y, facecolor = 'k')
        # ax.add_patch(p)


