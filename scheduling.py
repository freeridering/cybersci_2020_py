import parameter
import numpy as np
import uav
import target
import random
import optimal


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
# 更新路径和相关信息
def uav_update(time_counter: int, p: parameter.Parameter, uav_swarm: uav.Uav_Swarm,
               target_swarm: target.Target_Swarm):
    p.S_a_p = p.S_a
    p.S_r_p = p.S_r
    p.V = np.zeros([p.nx, p.ny], dtype=float)
    for i in range(p.nu):
        [pos_nox_x, pos_nox_y] = uav_swarm[i].pos_now
        p.detect_map[pos_nox_x, pos_nox_y] = 1
        p.V[pos_nox_x, pos_nox_y] = 1
        # 更新单无人机信息
        # uav_swarm[i] = uav_single_step(time_counter, p, uav_swarm[i], target_swarm)
        uav_swarm[i].pos_past = uav_swarm[i].pos_now
        uav_swarm[i].path[time_counter] = uav_swarm[i].pos_now
        # 改变位置
        uav_swarm[i].pos_now = uav_swarm[i].way_global[0]  # 更新当前位置
        # 更新概率图 arm, target_swarm)
        [pos_nox_x, pos_nox_y] = uav_swarm[i].pos_now
        t_map_num = p.t_map[pos_nox_x, pos_nox_y]
        if t_map_num != -1:  # 遇到目标
            target_swarm[t_map_num].found_flag = True
        for j in range(p.nt):
            target_swarm[j].p_map[pos_nox_x, pos_nox_y] = 0
    # 除法还未加入对全0的考虑，或许也可以不修改概率地图
    p.found_counter = 0
    for i in range(p.nt):
        p_map_sum = np.sum(target_swarm[i].p_map)
        if not target_swarm[i].found_flag and p_map_sum != 0:
            target_swarm[i].p_map = target_swarm[i].p_map / p_map_sum
        elif target_swarm[i].found_flag:
            p.found_counter += target_swarm[i].found_flag


def uav_swarm_step(time_counter: int, p: parameter.Parameter, uav_swarm: uav.Uav_Swarm,
                   target_swarm: target.Target_Swarm):
    # 求下一步策略
    p.S_a_p = p.S_a
    p.S_a = cal_S_a(p)
    p.S_r_p = p.S_r
    p.S_r = cal_S_r(p)
    search_way_local(p, uav_swarm, target_swarm)
    for i in range(p.nu):
        search_way_global(time_counter, p, uav_swarm[i], uav_swarm, target_swarm)
    optimal.interation(time_counter, uav_swarm, target_swarm, p)


def cal_J(j: int, p: parameter.Parameter, temp_uav: uav.UavSingle, temp_way_local: np.ndarray):
    [temp_x, temp_y] = temp_uav.pos_now
    J_c = -p.alpha * temp_uav.S_d[temp_x, temp_y]
    s_d = 0
    s_a = 0
    s_r = 0
    for i in range(p.n_step):
        [temp_x, temp_y] = temp_way_local[i]
        s_a += np.exp((1 - i) / p.n_step) * p.beta * p.S_a[temp_x, temp_y]
        s_r += p.gama * p.S_r[temp_x, temp_y]
        s_d += -p.alpha * temp_uav.S_d[temp_x, temp_y]
    J_cp = s_a + s_d + s_r
    J_t = 0
    for i in range(p.n_step):
        [temp_x, temp_y] = temp_way_local[i]
        J_t = J_t + np.exp((1 - i) / p.n_step) * np.log(1 / (1 - p.p_map[temp_x, temp_y]))
        J_c = J_c + np.exp((1 - i) / p.n_step) * (p.beta * p.S_a[temp_x, temp_y] - p.gama * p.S_r[temp_x, temp_y])
    # no TPM 则J_t = 0
    # J_t = 0
    # no DPM 则 J_c = 0
    # J_c = 0
    uid = temp_uav.uid
    p.draw_meterial.temp_Jt[uid, j] = J_t
    p.draw_meterial.temp_Jc[uid, j] = J_c
    J_total = p.lampda_1 * J_t + p.lampda_2 * J_cp
    return J_total


def interation_global(time_counter: int, p: parameter.Parameter, temp_uav: uav.UavSingle, uav_swarm: uav.Uav_Swarm,
                      target_swarm: target.Target_Swarm):
    p.p_map = cal_total_p_map(p, target_swarm)
    J_total = -np.inf * np.ones([p.max_way_num, ], dtype=float)

    for j in range(p.max_way_num):

        temp_way_local = np.reshape(temp_uav.all_way_local[j], [p.n_step, 2])  # 重塑成 n_step*2的矩阵
        interation_cal_S_d_i(p, temp_uav, uav_swarm, temp_way_local)
        if temp_way_local[0, 0] > -1:
            [temp_x, temp_y] = temp_uav.pos_now
            J_c = -p.alpha * temp_uav.S_d[temp_x, temp_y]
            s_d = 0
            s_a = 0
            s_r = 0
            for i in range(p.n_step):
                [temp_x, temp_y] = temp_way_local[i]
                s_a += np.exp((1 - i) / p.n_step) * p.beta * p.S_a[temp_x, temp_y]
                s_r -= np.exp((1 - i) / p.n_step) * p.gama * p.S_r[temp_x, temp_y]
                s_d += -p.alpha * temp_uav.S_d[temp_x, temp_y]
            J_cp = s_a + s_d + s_r
            J_t = 0
            for i in range(p.n_step):
                [temp_x, temp_y] = temp_way_local[i]
                if p.p_map[temp_x, temp_y] > 0.3:
                    J_t = J_t + np.exp((1 - i) / p.n_step) * np.log(1 / (1 - p.p_map[temp_x, temp_y]))
                else:
                    l_map = np.ones([p.nx, p.ny], dtype=float)
                    l_map = cal_manhattan(l_map, temp_x, temp_y, p)
                    temp_p_map = p.p_map / l_map
                    temp_p_map[temp_x, temp_y] = 0
                    temp_p_map = np.sum(temp_p_map)
                    J_t = J_t + np.exp((1 - i) / p.n_step) * np.log(1 / (1 - temp_p_map))

                J_c = J_c + np.exp((1 - i) / p.n_step) * (
                        p.beta * p.S_a[temp_x, temp_y] - p.gama * p.S_r[temp_x, temp_y])
            # no TPM 则J_t = 0
            # J_t = 0
            # no DPM 则 J_c = 0
            # J_c = 0
            J_total[j] = p.lampda_1 * J_t + p.lampda_2 * J_cp
    temp_uav.Jmax[time_counter] = np.max(J_total)
    temp_uav.way_global_inter = np.reshape(temp_uav.all_way_local[np.argmax(J_total)], [p.n_step, 2])
    p.draw_meterial.Jc_max[temp_uav.uid, time_counter] = p.draw_meterial.temp_Jc[temp_uav.uid][np.argmax(J_total)]
    p.draw_meterial.Jt_max[temp_uav.uid, time_counter] = p.draw_meterial.temp_Jt[temp_uav.uid][np.argmax(J_total)]


def cal_manhattan(l_map: np.array, x, y, p: parameter.Parameter):
    for i in range(p.nx):
        for j in range(p.ny):
            if i != x and j != y:
                l_map[i, j] = abs(i - x) + abs(j - y)
    return l_map


def search_way_global(time_counter: int, p: parameter.Parameter, temp_uav: uav.UavSingle, uav_swarm: uav.Uav_Swarm,
                      target_swarm: target.Target_Swarm):
    p.p_map = cal_total_p_map(p, target_swarm)
    J_total = -np.inf * np.ones([p.max_way_num, ], dtype=float)
    for j in range(p.max_way_num):
        temp_way_local = np.reshape(temp_uav.all_way_local[j], [p.n_step, 2])  # 重塑成 n_step*2的矩阵
        cal_S_d_i(p, temp_uav, uav_swarm, temp_way_local)
        if temp_way_local[0, 0] > -1:
            [temp_x, temp_y] = temp_uav.pos_now
            J_c = -p.alpha * temp_uav.S_d[temp_x, temp_y]
            s_d = 0
            s_a = 0
            s_r = 0
            for i in range(p.n_step):
                [temp_x, temp_y] = temp_way_local[i]
                s_a += np.exp((1 - i) / p.n_step) * p.beta * p.S_a[temp_x, temp_y]
                s_r -= np.exp((1 - i) / p.n_step) * p.gama * p.S_r[temp_x, temp_y]
                s_d += -p.alpha * temp_uav.S_d[temp_x, temp_y]
            J_cp = s_a + s_d + s_r
            J_t = 0
            for i in range(p.n_step):
                [temp_x, temp_y] = temp_way_local[i]
                J_t = J_t + np.exp((1 - i) / p.n_step) * np.log(1 / (1 - p.p_map[temp_x, temp_y]))
                J_c = J_c + np.exp((1 - i) / p.n_step) * (
                        p.beta * p.S_a[temp_x, temp_y] - p.gama * p.S_r[temp_x, temp_y])
            # no TPM 则J_t = 0
            # J_t = 0
            # no DPM 则 J_c = 0
            # J_c = 0
            uid = temp_uav.uid
            p.draw_meterial.temp_Jt[uid, j] = J_t
            p.draw_meterial.temp_Jc[uid, j] = J_cp
            J_total[j] = p.lampda_1 * J_t + p.lampda_2 * J_cp
    temp_uav.Jmax[time_counter] = np.max(J_total)
    temp_uav.way_global = np.reshape(temp_uav.all_way_local[np.argmax(J_total)], [p.n_step, 2])
    p.draw_meterial.Jc_max[temp_uav.uid, time_counter] = p.draw_meterial.temp_Jc[temp_uav.uid][np.argmax(J_total)]
    p.draw_meterial.Jt_max[temp_uav.uid, time_counter] = p.draw_meterial.temp_Jt[temp_uav.uid][np.argmax(J_total)]


def cal_S_d_i(p: parameter.Parameter, temp_uav: uav.UavSingle, uav_swarm: uav.Uav_Swarm, tempway):
    d_k = np.zeros([p.n_step, p.nx, p.ny], dtype=float)
    u_l = np.zeros([p.n_step, p.nx, p.ny], dtype=float)
    # 计算
    for i in range(p.n_step):
        for j in range(p.nu):
            if j > temp_uav.uid:
                [temp_x, temp_y] = uav_swarm[j].way_local[i]
                d_k[i, temp_x, temp_y] += 1
            elif j < temp_uav.uid:
                [temp_x, temp_y] = uav_swarm[j].way_global[i]
                d_k[i, temp_x, temp_y] += 1
        [temp_x, temp_y] = tempway[i]
        u_l[i, temp_x, temp_y] = 1
    S_d = np.zeros([p.nx, p.ny], dtype=float)
    for i in range(p.n_step):
        for j in range(i + 1):
            S_d += np.exp((1 - j + i) / p.n_step) * p.d_d * (u_l[j] * d_k[i])
    temp_uav.S_d = S_d


def interation_cal_S_d_i(p: parameter.Parameter, temp_uav: uav.UavSingle, uav_swarm: uav.Uav_Swarm, tempway):
    d_k = np.zeros([p.n_step, p.nx, p.ny], dtype=float)
    u_l = np.zeros([p.n_step, p.nx, p.ny], dtype=float)
    # 计算
    for i in range(p.n_step):
        for j in range(p.nu):
            if j != temp_uav.uid:
                [temp_x, temp_y] = uav_swarm[j].way_global_inter[i]
                d_k[i, temp_x, temp_y] += 1
        [temp_x, temp_y] = tempway[i]
        u_l[i, temp_x, temp_y] = 1
    S_d = np.zeros([p.nx, p.ny], dtype=float)
    for i in range(p.n_step):
        for j in range(i + 1):
            S_d += np.exp((1 - j + i) / p.n_step) * p.d_d * (u_l[j] * d_k[i])
    temp_uav.S_d = S_d


def cal_total_p_map(p: parameter.Parameter, target_swarm: target.Target_Swarm):
    p_map = np.ones([p.nx, p.ny], dtype=float) * (~p.g_map)
    for i in range(p.nt):
        if not target_swarm[i].found_flag:
            p_map = p_map * (np.ones([p.nx, p.ny], dtype=float) - target_swarm[i].p_map)
    p_map = np.ones([p.nx, p.ny], dtype=float) * (~p.g_map) - p_map
    p_map = p_map / p_map.sum()
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
                    if i_way < p.max_way_num:
                        all_way[i_way] = np.reshape(new_way[1:, :], [1, p.n_step * 2])
                        i_way += 1
            elif i_step < 2:
                new_way = np.copy(way)
                new_way[i_step] = next_pos
                if i_step < p.n_step:
                    all_way, i_way = call_path(next_pos, new_way, all_way, i_step + 1, i_way, p)
                else:
                    if i_way < p.max_way_num:
                        all_way[i_way] = np.reshape(new_way, [1, p.n_step * 2])
                        i_way += 1
    return all_way, i_way


def cal_J_noSd(p: parameter.Parameter, temp_way_local: np.ndarray):
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


def search_way_local(p: parameter.Parameter, uav_swarm: uav.Uav_Swarm, target_swarm: target.Target_Swarm):
    p.p_map = cal_total_p_map(p, target_swarm)
    for i in range(p.nu):
        uav_swarm[i].all_way_local = get_all_way_local(p, uav_swarm[i])
        J_totnoSd = -np.inf * np.ones(p.max_way_num, )
        for j in range(p.max_way_num):
            temp_way_local = np.reshape(uav_swarm[i].all_way_local[j], [p.n_step, 2])
            if temp_way_local[0, 0] > -1:  # 合法路径
                J_totnoSd[j] = cal_J_noSd(p, temp_way_local)
        uav_swarm[i].way_local = np.reshape(uav_swarm[i].all_way_local[np.argmax(J_totnoSd)], [p.n_step, 2])


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


def target_swarm_step(time_counter: int, p: parameter.Parameter,
                      target_swarm: target.Target_Swarm):
    p.t_map = -1 * np.ones([p.nx, p.ny], dtype=int)  # 目标地图清零
    for i in range(p.nt):
        if not target_swarm[i].found_flag:  # 没有被找到
            target_swarm[i].path[time_counter] = target_swarm[i].pos_now.T  # 记录路径
            # 更新位置
            pos_now = np.zeros([2, ], dtype=int)
            if target_swarm[i].type == 0 or target_swarm[i].type == 2:
                while True:
                    step = random.choice(np.array([[1, 0], [-1, 0], [0, 1], [0, -1]], dtype=int))  # 下一步的位移
                    pos_now = target_swarm[i].pos_now + step
                    if not p.g_map_l[pos_now[0] + 2, pos_now[1] + 2]:
                        break
            elif target_swarm[i].type == 1:
                if target_swarm[i].move_dir == 0:  # 上下运动,变y不变x
                    while True:
                        step = random.choice(np.array([[0, 1], [0, -1]], dtype=int))  # 下一步的位移
                        pos_now = target_swarm[i].pos_now + step
                        if pos_now[0] in range(p.nx) and pos_now[1] in range(p.ny) and not p.g_map[
                            pos_now[0], pos_now[1]]:
                            break
                elif target_swarm[i].move_dir == 1:
                    while True:
                        step = random.choice(np.array([[1, 0], [-1, 0]], dtype=int))  # 下一步的位移
                        pos_now = target_swarm[i].pos_now + step
                        if pos_now[0] in range(p.nx) and pos_now[1] in range(p.ny) and not p.g_map[
                            pos_now[0], pos_now[1]]:
                            break
                else:
                    print('move_dir error')
                    exit(0)
            else:
                print('type error')
                exit(0)
            target_swarm[i].pos_now = pos_now
            p.t_map[pos_now[0], pos_now[1]] = target_swarm[i].tid
