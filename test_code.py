import numpy as np
import scipy.integrate
import math
import parameter
import scipy.stats



a = np.ones([3,3],dtype=float)
b  = ~np.ones([3,3],dtype=bool)
b[0,1] = True
print(b)
b[2,1] = True
print(b)
print(a)
c= a*b
print(c)