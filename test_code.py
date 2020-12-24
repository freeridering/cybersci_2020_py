import numpy as np
import scipy.integrate
import math

from typing import List

import parameter
import scipy.stats
import random
import pickle
import  target
import  parameter
import  uav

def list_tar(target_swarm: List[target.TargetSingle]):
    print(target_swarm[1].p_map)

p = parameter.Parameter()
target_swarm =   [target.TargetSingle(i,p) for i in range(p.nt)]
list_tar(target_swarm)
