import parameter
import numpy as np
import uav
import target
import random


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
        # uavs.uavs[i] = uav_single_step(time_counter, p, uavs.uavs[i], targets)
        uavs.uavs[i].pos_past = uavs.uavs[i].pos_now
        uavs.uavs[i].path[time_counter] = uavs.uavs[i].pos_now
        # 改变位置
        uavs.uavs[i].pos_now = uavs.uavs[i].way_global[0]  # 更新当前位置
        # 更新概率图
        detection(p, uavs.uavs[i], uavs, targets)
    # 求下一步策略
    p.S_a_p = p.S_a
    p.S_a = cal_S_a(p)
    p.S_r_p = p.S_r
    p.S_r = cal_S_r(p)
    search_way_local(p, uavs, targets)
    for i in range(p.nu):
        cal_S_d_i(p, uavs.uavs[i], uavs)
    search_way_global(p, uavs, targets)


# def uav_single_step(time_counter: int, p: parameter.Parameter, temp_uav: uav.UavSingle, targets: target.TargetSwarm):
#     temp_uav.path[time_counter] = temp_uav.pos_now.T  # 记录路径
#     temp_uav.pos_past = temp_uav.pos_now  # 赋值上一时刻点
#     temp_uav.pos_now = temp_uav.way_global[0]  # 更新当前位置
#     return temp_uav


def get_total_map(p: parameter.Parameter, targets: target.TargetSwarm):
    p_map = np.ones([p.nx, p.ny]) * (~p.g_map)
    time_num = 0
    for i in range(p.nt):
        if not targets.targets[i].found_flag:
            time_num += 1
            temp_p_map = np.ones([p.nx, p.ny], dtype=float) - targets.targets[i].p_map
            p_map = p_map * temp_p_map
    p_map = np.ones([p.nx, p.ny], dtype=float) - p_map / (10 ** time_num)
    p_map = p_map * (~p.g_map)
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
            S_d += np.exp(-j / p.n_step) * p.d_d * (u_l[:, :, j] * d_k[:, :, j])
    return S_d


def cal_total_p_map(p: parameter.Parameter, targets: target.TargetSwarm):
    p_map = np.ones([p.nx, p.ny], dtype=float) * (~p.g_map)
    times_num = 0
    for i in range(p.nt):
        if not targets.targets[i].found_flag:
            times_num += 1
            temp_map = (np.ones([p.nx, p.ny], dtype=float) - targets.targets[i].p_map) * 10
            p_map = p_map * temp_map
    p_map = np.ones([p.nx, p.ny], dtype=float) - p_map / (10 ** times_num)
    p_map = p_map * (~p.g_map)
    return p_map


def get_all_way_local(p: parameter.Parameter, temp_uav: uav.UavSingle):  # 针对单uav生成其最大数量可能路径
    init_pos = temp_uav.pos_now
    all_way_local = -1 * np.ones([p.max_way_num, p.n_step * 2], dtype=int)  # 某一决策周期uav的所有可行路径，可以设定最大条数
    init_way = np.zeros([p.n_step + 1, 2], dtype=int)
    init_way[0] = init_pos
    all_way_local, i_way = call_path(init_pos, init_way, all_way_local, i_step=1, i_way=0, p=p)
    return all_way_local


def call_path(pos_now, way, all_way, i_step, i_way, p: parameter.Parameter):
    pos_avi = np.tile(pos_now.T, (4, 1)) + np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    for i, next_pos in enumerate(pos_avi):
        if next_pos[0] in range(p.nx) and next_pos[1] in range(p.ny) and not p.g_map[next_pos[0], next_pos[1]]:
            if i_step > 1 and not (next_pos == way[i_step - 2]).all():
                new_way = np.copy(way)
                new_way[i_step] = next_pos
                if i_step < p.n_step:
                    all_way, i_way = call_path(next_pos, new_way, all_way, i_step + 1, i_way, p)
                else:
                    all_way[i_way] = np.reshape(new_way[1:, :], [1, p.n_step * 2])
                    i_way += 1
            elif i_step < 2:
                new_way = np.copy(way)
                new_way[i_step] = next_pos
                if i_step < p.n_step:
                    all_way, i_way = call_path(next_pos, new_way, all_way, i_step + 1, i_way, p)
                else:
                    all_way[i_way] = np.reshape(new_way, [1, p.n_step * 2])
                    i_way += 1
    return all_way, i_way


def cal_J_noSd(p: parameter.Parameter, temp_uav: uav.UavSingle, temp_way_local: np.ndarray):
    J_c = 0  # 没有Sd调度信息素
    J_t = 0
    for i in range(p.n_step):
        [temp_x, temp_y] = temp_way_local[i]
        p_map_xy = p.p_map[temp_x, temp_y]
        temp_J_t = np.exp((1 - i) / p.n_step) * np.log(1 / (1 - p.p_map[temp_x, temp_y]))
        J_t = J_t + np.exp((1 - i) / p.n_step) * np.log(1 / (1 - p.p_map[temp_x, temp_y]))
        p_S_a_xy = p.S_a[temp_x, temp_y]
        p_S_r_xy = p.S_r[temp_x, temp_y]
        J_c = J_c + np.exp((1 - i) / p.n_step) * p.beta * p.S_a[temp_x, temp_y] - p.gama * p.S_r[temp_x, temp_y]
    # no TPM 则 J_t=0
    # J_t = 0
    # no DPM则 J_c = 0
    ratio = p.lampda_1 * J_t/(p.lampda_2 * J_c)
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
    S_r = S_r * (~p.g_map)
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
    S_a = S_a * (~p.g_map)
    return S_a


def detection(p: parameter.Parameter, temp_uav: uav.UavSingle, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    [pos_nox_x, pos_nox_y] = temp_uav.pos_now
    t_map_num = p.t_map[pos_nox_x, pos_nox_y]
    if t_map_num != -1:  # 遇到目标
        targets.targets[t_map_num].found_flag = True
        p.found_counter = 0
        for i in range(p.nt):
            p.found_counter = p.found_counter + targets.targets[i].found_flag
    else:
        for j in range(p.nt):
            targets.targets[j].p_map[pos_nox_x, pos_nox_y] = 0
            # 除法还未加入对全0的考虑，或许也可以不修改概率地图
            targets.targets[j].p_map = targets.targets[j].p_map / np.sum(targets.targets[j].p_map)


def target_swarm_step(time_counter: int, p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    p.t_map = -1 * np.ones([p.nx, p.ny], dtype=int)  # 目标地图清零
    for i in range(p.nt):
        if not targets.targets[i].found_flag:  # 没有被找到
            targets.targets[i].path[time_counter] = targets.targets[i].pos_now.T  # 记录路径
            # 更新位置
            pos_now = np.zeros([2, ], dtype=int)
            if targets.targets[i].type == 0 or targets.targets[i].type == 2:
                while True:
                    step = random.choice(np.array([[1, 0], [-1, 0], [0, 1], [0, -1]], dtype=int))  # 下一步的位移
                    pos_now = targets.targets[i].pos_now + step
                    if not p.g_map_l[pos_now[0] + 2, pos_now[1] + 2]:
                        break
            elif targets.targets[i].type == 1:
                if targets.targets[i].move_dir == 0:  # 上下运动,变y不变x
                    while True:
                        step = random.choice(np.array([[0, 1], [0, -1]], dtype=int))  # 下一步的位移
                        pos_now = targets.targets[i].pos_now + step
                        if not p.g_map[p.g_map[pos_now[0], pos_now[1]]]:
                            break
                elif targets.targets[i].move_dir == 1:
                    while True:
                        step = random.choice(np.array([[1, 0], [-1, 0]], dtype=int))  # 下一步的位移
                        pos_now = targets.targets[i].pos_now + step
                        if not p.g_map[p.g_map[pos_now[0], pos_now[1]]]:
                            break
                else:
                    print('move_dir error')
                    exit(0)
            else:
                print('type error')
                exit(0)
            targets.targets[i].pos_now = pos_now
            p.t_map[pos_now[0], pos_now[1]] = targets.targets[i].tid

#
# def target_single_step(time_counter: int, p: parameter.Parameter, uavs: uav.UavSwarm, temp_target: target.TargetSingle):
#     temp_target.path[time_counter] = temp_target.pos_now.T  # 记录路径
#     # 更新位置
#     pos_now = np.zeros([2, ], dtype=int)
#     if temp_target.type == 0 or temp_target.type == 2:
#         while True:
#             step = random.choice(np.array([[1, 0], [-1, 0], [0, 1], [0, -1]], dtype=int))  # 下一步的位移
#             pos_now = temp_target.pos_now + step
#             if not p.g_map_l[pos_now[0] + 2, pos_now[1] + 2]:
#                 break
#     elif temp_target.type == 1:
#         if temp_target.move_dir == 0:  # 上下运动,变y不变x
#             while True:
#                 step = random.choice(np.array([[0, 1], [0, -1]], dtype=int))  # 下一步的位移
#                 pos_now = temp_target.pos_now + step
#                 if not p.g_map[p.g_map[pos_now[0], pos_now[1]]]:
#                     break
#         elif temp_target.move_dir == 1:
#             while True:
#                 step = random.choice(np.array([[1, 0], [-1, 0]], dtype=int))  # 下一步的位移
#                 pos_now = temp_target.pos_now + step
#                 if not p.g_map[p.g_map[pos_now[0], pos_now[1]]]:
#                     break
#         else:
#             print('move_dir error')
#             exit(0)
#     else:
#         print('type error')
#         exit(0)
#     temp_target.pos_now = pos_now
#     p.t_map[pos_now[0], pos_now[1]] = temp_target.tid
#
#     return temp_target
