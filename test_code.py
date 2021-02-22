import numpy as np
import scipy.integrate
import math

from typing import List

import parameter
import scipy.stats
import random
import pickle
import target
import parameter
import uav

a = np.ones([3,3])
b = np.ones([3,3])*6
a[0,0] = 12
c = b/a
print(c)