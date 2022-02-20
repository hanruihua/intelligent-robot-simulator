import numpy as np
from ir_sim.world import obs_line


class env_obs_line:
    def __init__(self, obs_line_states=[], obs_line_class=obs_line, **kwargs):
        self.obs_line_states = obs_line_states
        
        self.obs_line_list = [ obs_line_class(id=i, line_state=obs_line_states[i], **kwargs) for i in range(len(obs_line_states)) ]




    
    # def