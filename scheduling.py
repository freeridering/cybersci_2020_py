import parameter
import numpy as np
import uav
import target


#
# # 暂时没什么用的类
# class EventScheduling:
#     def __init__(self):
#         self.event_list = []  # 事件列表
#         self.timing = 0  # 计时器
#
#     def add_evevt(self):
#         """
#         添加事件
#         :return:
#         """
#         pass
#
#     def fetch_event(self):
#         """
#         取符合要求的事件
#         :return:
#         """
#         pass
#
#     def scheduling(self, p: parameter.Parameter):
#         """
#         调度过程主体
#         :return:
#         """
#         self.uav_step(p)
#         self.target_step(p)


def uav_swarm_step(time_counter: int, p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    p.S_a_p = p.S_a
    p.S_r_p = p.S_r
    p.V = np.zeros([p.nx, p.ny], dtype=float)
    for i in range(p.nu):
        p.detect_map[uavs.uavs[i].pos_now[0], uavs.uavs[i].pos_now[1]] = 1
        p.V[uavs.uavs[i].pos_now[0], uavs.uavs[i].pos_now[1]] = 1
        # 更新单无人机信息
        uavs.uavs[i] = uav_single_step(time_counter, p, uavs.uavs[i], targets)
        # 更新概率图
        detection(p, uavs.uavs[i], uavs, targets)
    p.S_a_p = p.S_a
    p.S_a = cal_S_a(p)
    p.S_r_p = p.S_r
    p.S_r = cal_S_r(p)
    search_way_local(p, uavs, targets)
    for i in range(p.nu):
        cal_S_d_i(p, uavs.uavs[i], uavs)
    search_way_global(p, uavs, targets)


def uav_single_step(time_counter: int, p: parameter.Parameter, uav: uav.UavSingle, targets: target.TargetSwarm):
    uav.path[time_counter] = uav.pos_now.T  # 记录路径
    uav.pos_past = uav.pos_now  # 赋值上一时刻点
    uav.pos_now = uav.way_global[0]  # 更新当前位置
    return uav


def get_total_map(p: parameter.Parameter, targets: target.TargetSwarm):
    p_map = np.dot(np.ones([p.nx, p.ny]), ~p.g_map)
    time_num = 0
    for i in range(p.nt):
        if not targets.targets[i].foundflag:
            time_num += 1
            temp_p_map = np.ones([p.nx, p.ny], dtype=float) - targets.targets[i].p_map
            p_map = np.dot(p_map, temp_p_map)
    p_map = np.ones([p.nx, p.ny], dtype=float) - p_map / (10 ** time_num)
    p_map = np.dot(p_map, ~p.g_map)
    return p_map


def cal_J(p: parameter.Parameter, temp_uav: uav.UavSingle, temp_way_local: np.ndarray):
    [temp_x, temp_y] = temp_uav.pos_now
    J_c = -p.alpha * temp_uav.S_d[temp_x, temp_y]
    J_t = 0
    for i in range(p.n_step):
        [temp_x, temp_y] = temp_way_local[i]
        J_t = J_t + np.exp((1 - i) / p.n_step) * np.log(1 / (1 - p.p_map[temp_x, temp_y]))
        J_c = J_c + np.exp((1 - i) / p.n_step) * p.beta * p.S_a[temp_x, temp_y] - p.gama * p.S_r[temp_x, temp_y]
    # no TPM 则J_t = 0
    # J_t = 0
    # no DPM 则 J_c = 0
    J_total = p.lampda_1 * J_t + p.lampda_2 * J_c
    return J_total


def search_way_global(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    p.p_map = get_total_map(p, targets)
    for i in range(p.nu):
        J_total = np.zeros([p.max_way_num, ], dtype=float)
        for j in range(p.max_way_num):
            temp_way_local = np.reshape(uavs.uavs[i].all_way_local[j], [p.n_step, 2])  # 重塑成 n_step*2的矩阵
            if temp_way_local[0, 0] > -1:
                J_total[j] = cal_J(p, uavs.uavs[i], temp_way_local)
        uavs.uavs[i].way_global = np.reshape(uavs.uavs[i].all_way_local[np.argmax(J_total)], [p.n_step, 2])


def cal_S_d_i(p: parameter.Parameter, temp_uav: uav.UavSingle, uavs: uav.UavSwarm):
    d_k = np.zeros([p.nx, p.ny, p.n_step], dtype=float)
    u_l = np.zeros([p.nx, p.ny, p.n_step], dtype=float)
    # 计算
    for i in range(p.n_step):
        for j in range(p.nu):
            [temp_x, temp_y] = uavs.uavs[i].way_local[i]
            d_k[temp_x, temp_y, i] += 1
        [temp_x, temp_y] = temp_uav.way_local[i]
        u_l[temp_x, temp_y, i] = 1
    S_d = np.zeros([p.nx, p.ny], dtype=float)
    for i in range(p.n_step):
        for j in range(i):
            S_d += np.exp(-j / p.n_step) * p.d_d * np.dot(u_l[:, :, j], d_k[:, :, j])
    return S_d


def cal_total_p_map(p: parameter.Parameter, targets: target.TargetSwarm):
    p_map = np.dot(np.ones([p.nx, p.ny], dtype=float), ~p.g_map)
    times_num = 0
    for i in range(p.nt):
        if not targets.targets[i].foundflag:
            times_num += 1
            temppmap = (np.ones([p.nx, p.ny], dtype=float) - targets.targets[i].p_map) * 10
            p_map = np.dot(p_map, temppmap)
    p_map = np.ones([p.nx, p.ny], dtype=float) - p_map / (10 ** times_num)
    p_map = np.dot(p_map, ~p.g_map)
    return p_map


def get_all_way_local(p: parameter.Parameter, temp_uav: uav.UavSingle):  # 针对单uav生成其最大数量可能路径
    [temp_x, temp_y] = temp_uav.pos_now
    max_page = 4 ** (p.n_step - 1) - 1
    for i in range(p.max_way_num):
        way_map = np.zeros([p.nx, p.ny], dtype=float)
    return -1 * temp_uav.all_way_local


def cal_J_noSd(p: parameter.Parameter, temp_uav: uav.UavSingle, temp_way_local: np.ndarray):
    J_c = 0  # 没有Sd调度信息素
    J_t = 0
    for i in range(p.n_step):
        [temp_x, temp_y] = temp_way_local[i]
        J_t = J_t + np.exp((1 - i) / p.n_step) * np.log(1 / (1 - p.p_map[temp_x, temp_y]))
        J_c = J_c + np.exp((1 - i) / p.n_step) * p.beta * p.S_a[temp_x, temp_y] - p.gama * p.S_r[temp_x, temp_y]
    # no TPM 则 J_t=0
    # J_t = 0
    # no DPM则 J_c = 0
    J_total = p.lampda_1 * J_t + p.lampda_2 * J_c
    return J_total


def search_way_local(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    p.p_map = cal_total_p_map(p, targets)
    for i in range(p.nu):
        uavs.uavs[i].all_way_local = get_all_way_local(p, uavs.uavs[i])
        J_totnoSd = np.zeros(p.max_way_num, )
        for j in range(p.max_way_num):
            temp_way_local = np.reshape(uavs.uavs[i].all_way_local[j], [p.n_step, 2])
            if temp_way_local[0, 0] > -1:  # 合法路径
                J_totnoSd[j] = cal_J_noSd(p, uavs.uavs[i], temp_way_local)
        uavs.uavs[i].way_local = np.reshape(uavs.uavs[i].all_way_local[np.argmax(J_totnoSd)], [p.n_step, 2])


def cal_GP_r(p: parameter.Parameter):
    GP_r = np.zeros([p.nx, p.ny], dtype=float)
    temp_srp = np.zeros([p.nx + 2, p.ny + 2], dtype=float)
    temp_srp[1:p.nx + 1, 1:p.ny + 1] = p.S_r_p
    for i in range(p.nx):
        for j in range(p.ny):
            grid_ln = temp_srp[i:i + 3, j:j + 3]  # 临近栅格
            ln = np.count_nonzero(grid_ln)  # 非零元素个数
            if ln != 0:
                GP_r[i, j] = p.G_r * (p.d_r + (np.sum(grid_ln) / ln))
    return GP_r


def cal_S_r(p: parameter.Parameter):
    GP_r = cal_GP_r(p)
    S_r = (1 - p.E_r) * ((1 - p.G_r) * (p.S_r_p + p.d_r * p.V) + GP_r)
    S_r = np.dot(S_r, ~p.g_map)
    return S_r


def cal_GP_a(p: parameter.Parameter):
    GP_a = np.zeros([p.nx, p.ny], dtype=float)
    temp_sap = np.zeros([p.nx + 2, p.ny + 2], dtype=float)
    temp_sap[1:p.nx + 1, 1:p.ny + 1] = p.S_a_p
    for i in range(p.nx):
        for j in range(p.ny):
            grid_ln = temp_sap[i:i + 3, j:j + 3]  # 临近栅格
            grid_ln[1, 1] = 0
            ln = np.count_nonzero(grid_ln)  # 非零元素个数
            if ln != 0:
                GP_a[i, j] = p.G_a * (p.d_a + (np.sum(grid_ln) / ln))
    return GP_a


def cal_S_a(p: parameter.Parameter):
    E = np.ones([p.nx, p.ny], dtype=float)
    GP_a = cal_GP_a(p)
    S_a = (1 - p.E_a) * ((1 - p.G_a) * (p.S_a_p + p.d_a * (E - p.V)) + GP_a)
    S_a = np.dot(S_a, ~p.g_map)
    return S_a


def detection(p: parameter.Parameter, temp_uav: uav.UavSingle, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    [pos_nox_x, pos_nox_y] = temp_uav.pos_now
    t_map_num = p.t_map[pos_nox_x, pos_nox_y]
    if t_map_num != -1:  # 遇到目标
        targets.targets[t_map_num].foundflag = True
        p.found_counter = 0
        for i in range(p.nt):
            p.found_counter = p.found_counter + targets.targets[i].foundflag
    else:
        for j in range(p.nt):
            targets.targets[j].p_map[pos_nox_x, pos_nox_y] = 0
            # 除法还未加入对全0的考虑，或许也可以不修改概率地图
            targets.targets[j].p_map = targets.targets[j].p_map / np.sum(targets.targets[j].p_map)


def target_swarm_step(time_counter: int, p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    p.t_map = -1 * np.ones([p.nx, p.ny], dtype=int)  # 目标地图清零
    for i in range(p.nt):
        targets.targets[i] = target_single_step(time_counter, p, uavs, targets.targets[i])


def target_single_step(time_counter: int, p: parameter.Parameter, uavs: uav.UavSwarm, temp_target: target.TargetSingle):
    if not temp_target.foundflag:  # 没有被找到
        temp_target.path[time_counter] = temp_target.pos_now.T  # 记录路径
        # 更新位置
    return temp_target
