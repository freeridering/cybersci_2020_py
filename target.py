import numpy as np
import parameter
import math
import scipy.integrate
import random


class TargetSwarm:
    def __init__(self, p: parameter.Parameter):
        self.targets = [TargetSingle(0, p)] * p.nt
        for i in range(p.nt):
            self.targets[i] = TargetSingle(i, p)


class TargetSingle:
    def __init__(self, tid: int, p: parameter.Parameter):
        self.tid = tid  # 目标的id
        self.pos_now = self.init_pos_now(tid, p)
        self.type = int(tid % p.t_type_num)  # 目标的类别
        self.move_dir = random.randint(0, 1)  # 仅针对第二类目标，
        self.path = np.zeros([p.time_limit, 2], dtype=int)  # 走过的路径，原名wayed
        self.found_flag = False  # 初始状态，未被找到
        self.p_map = self.init_p_map(p)

    def init_pos_now(self, tid: int, p: parameter.Parameter):
        pos_now = np.zeros([2, ], dtype=int)
        temp_num = int(np.sqrt(p.nt))
        flex_axis = int(np.round(p.nx // (temp_num + 1) / (temp_num + 1)) * (temp_num + 1))
        pos_now[0] = flex_axis * (tid // temp_num + 1)
        pos_now[1] = flex_axis * (tid % temp_num + 1)

        return pos_now

    def init_p_map(self, p: parameter.Parameter):
        length_D = int(p.v_target * p.t_det)
        p_map_D_L = np.zeros([p.nx + 2 * length_D, p.ny + 2 * length_D], dtype=float)
        g_map_D_L = np.pad(p.g_map, ((length_D, length_D), (length_D, length_D)), 'constant', constant_values=True)
        tid = self.tid
        type = self.type
        # 根据目标类型确定概率分布模型
        if self.type == 0:  # 平均分布
            p_map_D_L = np.ones([p.nx + 2 * length_D, p.ny + 2 * length_D], dtype=float)
        elif self.type == 1:  # 十字平均分布
            init_pos = self.pos_now
            p_map_D_L[init_pos[0]:init_pos[0] + 2 * length_D, init_pos[1] + length_D] = np.full([1, 2 * length_D], 1)
            p_map_D_L[init_pos[0] + length_D, init_pos[1]:init_pos[1] + 2 * length_D] = np.full([1, 2 * length_D], 1)

        elif self.type == 2:
            pos_d_l = self.pos_now + [length_D, length_D]
            for i in range(length_D, p.nx + length_D):
                for j in range(length_D, p.ny + length_D):
                    p_map_D_L[i, j] = self.cal_cdf(i, j, p, pos_d_l)
        else:
            print('type error')
            exit(0)
        p_map_D_L = p_map_D_L * (~g_map_D_L)
        p_map_D_L = p_map_D_L / np.sum(p_map_D_L)
        p_map = p_map_D_L[length_D:p.nx + length_D, length_D:p.ny + length_D]
        return p_map

    def cal_cdf(self, i: int, j: int, p: parameter.Parameter, pos_d_l):
        func_1 = lambda x, y: (1 / (2 * math.pi * p.theta2)) * np.exp(
            -(1 / (2 * p.theta2)) * (x ** 2 + y ** 2))
        res, err = scipy.integrate.dblquad(func_1, i - pos_d_l[0] - p.lix / 2,
                                           i - pos_d_l[0] + p.lix / 2,
                                           lambda g: j - pos_d_l[1] - p.liy / 2,
                                           lambda h: j - pos_d_l[1] + p.liy / 2)

        return res
# def cal_cdf_term(y,x):
#     theta_mus = 1/(2*p.theta2)
#     res = np.exp(-theta_mus*((x-pos_d_l[0])**2+(y-pos_d_l[1])**2))*theta_mus/math.pi
#     return  res
