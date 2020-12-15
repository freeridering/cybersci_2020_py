import numpy as np
import tensorflow as tf


class Parameter:
    def __init__(self):
        nx = 20
        ny = 20
        self.nx = nx * 3 + 1  # 地图长宽
        self.ny = ny * 3 + 1
        self.ox = 3  # 障碍建筑物长宽
        self.oy = 3
        self.lix = 4  # 方格长宽
        self.liy = 4
        self.g_map = self.init_map()  # 地理地图
        self.p_map = np.zeros([self.nx, self.ny], dtype=float, )  # 概率分布地图
        self.t_map = np.zeros([self.nx, self.ny], dtype=float, )  # 目标位置地图
        self.g_map_l = np.ones([self.nx + 2, self.ny + 2], dtype=float, )
        self.g_map_l[1:self.nx + 1, 1:self.ny + 1] = self.g_map  # 地理地图扩展
        self.nu = 12  # uav数量
        self.nt = 9  # 目标数量
        self.time_limit = 300  # 最大仿真时间
        self.n_step = 5  # 预测周期
        self.d_d = 5  # dd
        self.G_a = 0.3  # G_a
        self.E_a = 0.4  # E_a
        self.d_a = 1  # d_a
        self.G_r = 0.3  # G_r
        self.E_r = 0.4  # E_r
        self.d_r = 10.  # d_r
        self.S_a = np.zeros([self.nx, self.ny], dtype=float)  # t时刻s_a
        self.S_a_p = np.zeros([self.nx, self.ny], dtype=float)  # t - 1时刻s_a
        self.S_r = np.zeros([self.nx, self.ny], dtype=float)  # t时刻s_r
        self.S_r_p = np.zeros([self.nx, self.ny], dtype=float)  # t - 1时刻s_r
        self.V = np.zeros([self.nx, self.ny], dtype=float)  # V
        self.alpha = 1.0  # alpha
        self.beta = 1.0  # beta
        self.gama = 1.  # gama
        self.theta2 = 10.  # theta ^ 2
        self.lampda1 = 1.  # lampda1
        self.lampda2 = 1.  # lampda2
        self.t_type_num = 3.  # 目标种类
        self.t_det = 10.  #
        self.v_target = 0.5  # 目标速度
        self.found_counter = 0.  # 已找到目标数量
        self.fd_ct_list = -1 * np.ones([self.time_limit, ], dtype=float)  # 不同时刻找到的目标数量
        self.detect_map = np.zeros([self.nx, self.ny], dtype=float)
        self.detect_list = -1 * np.ones([self.time_limit, ], dtype=float) # 记录对地图的探索度

    def init_map(self):
        """

        :return:
        """
        g_map = np.zeros([self.nx, self.ny], dtype=float)
        oxn = int(np.floor(self.nx / (self.ox + 1)))
        oyn = int(np.floor(self.ny / (self.oy + 1)))
        for i in range(oxn):
            for j in range(oyn):
                g_map[4 * i - 2:4 * i, 4 * j - 2:4 * j] = 1
        return g_map
