import numpy as np
class po:
    def __init__(self):
        self.p =  [1,2,3,4,5]
class cc:
    def __init__(self):
        self.i = []
        self.p = po()
        self.i = [self.p,[0,1]]
ccc = cc()
def cal_po(po:po):
    po.p.append(6)
def cal_cc(ccc:cc):
    cal_po(ccc.i[0])

cal_cc(ccc)
print(ccc)