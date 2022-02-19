import numpy as np
from ir_sim.world import obs_polygon

class env_obs_poly:
    def __init__(self, obs_poly_class=obs_polygon, vertex_list=[], obs_poly_num=1, **kwargs):
        self.obs_poly_list = [ obs_poly_class(vertex=v, **kwargs) for v in vertex_list[0:obs_poly_num]] 

    