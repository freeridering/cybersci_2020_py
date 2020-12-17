import numpy as np
import parameter


class TargetSwarm:
    def __init__(self, p: parameter.Parameter):
        self.targets = [TargetSingle] * p.nt
        for i in range(p.nt):
            self.targets[i] = TargetSingle(i, p)


class TargetSingle:
    def __init__(self, tid, p: parameter.Parameter):
        self.tid = tid  # 目标的id
        self.pos_now = np.zeros([2, ], dtype=int)  # 目标的当前位置/初始化位置
        self.type = 0
        self.pos_now = np.zeros([2, ], dtype=int)  # 目标的当前位置/初始化位置，=
        self.path = np.zeros([p.time_limit, 2], dtype=int)  # 走过的路径，原名wayed
        self.foundflag = False  # 初始状态，未被找到
        self.p_map = self.init_p_map(p)

    @staticmethod
    def init_p_map(p: parameter.Parameter):
        p_map = np.zeros([p.nx, p.ny], dtype=float)
        length_D = int(p.v_target * p.t_det)
        g_map_D_L = np.ones([p.nx + 2 * length_D, p.ny + 2 * length_D], dtype=float)
        p_map_D_L = np.zeros([p.nx + 2 * length_D, p.ny + 2 * length_D], dtype=float)
        g_map_D_L[length_D:(p.nx + length_D), length_D:(p.ny + length_D)] = p.g_map
        # 根据目标类型确定概率分布模型

        return p_map
