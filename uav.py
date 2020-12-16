import parameter
import numpy as np


class UavSwarm:
    def __init__(self, p: parameter.Parameter):
        self.uavs = [None] * p.nu
        for i in range(p.nu):
            self.uavs[i] = UavSingle(i, p)


class UavSingle:  # 单个无人机
    def __init__(self, uid, p: parameter.Parameter):
        self.uid = uid  # uav的id
        self.pos_now = np.zeros([2, ], dtype=int)  # uav的当前位置/初始化位置，=
        self.pos_past = np.copy(self.pos_now)  # uav的上一时刻位置/初始化位置
        self.all_way_local = -1 * np.ones([p.max_way_num, p.n_step * 2], dtype=int)  # 某一决策周期uav的所有可行路径，可以设定最大条数
        self.S_d = np.zeros([p.nx, p.ny], dtype=float)  # 调度信息素矩阵
        self.way_local = np.zeros([p.n_step, 2], dtype=int)  # 短暂局部最优
        self.way_global = np.zeros([p.n_step, 2], dtype=int)  # 全局最优
        self.path = np.zeros([p.time_limit, 2], dtype=int)  # 走过的路径，原名wayed
