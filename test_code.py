import numpy as np


class SSSS:
    def __init__(self, i):
        self.i = i


diccc = set()
for i in range(10):
    diccc.add(SSSS(i))
diccc = list(diccc)

a = [1, 2, 3, 4, 5, 6, 7, 8]
b = np.array(a)
c = -1 * np.ones([10, ], dtype=float)

al = np.ones([3, 3])

si = SSSS(20)
def change_si(si:SSSS):
    si.j = 50
change_si(si)


c1 = 0
def func1(c1):
    c2 = func2(c1)
    return  c2
def func2(c1):
    c2 = c1 +2
    return c2
c2 = func1(c1)
print(c2)
