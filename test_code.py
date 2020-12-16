import numpy as np

a = np.zeros([2, ], dtype=int)
b = np.zeros([1, 3], dtype=float)
c = np.ones([10, 2], dtype=float)
c[0] = a.T

aa = True
bb = True
cc = False

ze = np.zeros([3,3],dtype=float)
ze[1,1]  = 1
print(np.count_nonzero(ze))