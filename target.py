import numpy as np
import parameter


class TargetSwarm:
    def __init__(self, p: parameter.Parameter):
        self.targets = [None] * p.nt
        for i in range(p.nt):
            self.targets[i] = TargetSingle(i, p.nx, p.ny, p.time_limit)


class TargetSingle:
    def __init__(self, tid, nx, ny, time_limit):
        self.tid = tid  # 目标的id
        self.pos_now = np.zeros([2, ], dtype=int)  # 目标的当前位置/初始化位置
        self.type = 0  # 目标的类型
        self.pos_past = np.copy(self.pos_now)  # 目标的上一时刻位置/初始化位置
        self.foundflag = False  # 初始状态 未被找到
        self.p_map = self.init_p_map(nx, ny)
        self.path = np.zeros([time_limit, 2], dtype=int)  # 走过的路径，原名way

    def init_p_map(self, nx, ny):
        p_map = np.zeros([nx, ny], dtype=float)
        return p_map
