import numpy as np

a = np.zeros([2, ], dtype=int)
b = np.zeros([1, 3], dtype=float)
c = np.ones([10, 2], dtype=float)
c[0] = a.T

aa = True
bb = True
cc = False

tnp = np.ones([5,5,5])
tnp[0,0,1]  +=1