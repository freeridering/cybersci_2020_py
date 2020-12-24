import parameter
import uav
import target
import pickle
import matplotlib.pyplot as plt
import numpy as np


def draw_map(counter, p: parameter.Parameter, uav_swarm: uav.Uav_Swarm, target_swarm: target.Target_Swarm):
    draw_g_map(p)
    for i in range(p.nu):
        draw_uav_way(p, uav_swarm[i])
    for i in range(p.nt):
        draw_tar_way(p, target_swarm[i])


def draw_g_map(p: parameter.Parameter):
    plt.matshow(p.g_map)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.xticks(np.arange(0, p.nx, 4))
    plt.yticks(np.arange(0, p.ny, 4))
    plt.show()


def draw_uav_way(p: parameter.Parameter, uav: uav.UavSingle):
    pass


def draw_tar_way(p: parameter.Parameter, target: target.TargetSingle):
    pass


def draw_all(p: parameter.Parameter, uav_swarm: uav.Uav_Swarm, target_swarm: target.Target_Swarm):
    x_label_list = np.tile(np.reshape(np.array(range(p.ny)), (-1, 1)), (1, p.nx)).ravel()
    y_label_list = np.tile(range(p.nx), (p.ny, 1)).ravel()
    # x_label_list = range(p.nx)
    # y_label_list = range(p.ny)
    p_map = p.p_map.ravel()
    map_result = np.reshape(list(map(draw_cell, x_label_list, y_label_list, p_map)), (p.nx, p.ny))
    return map_result


def draw_cell(x, y, value):
    return value


if __name__ == '__main__':
    pickle_file = open('data.pkl', 'rb')
    data = pickle.load(pickle_file)
    p = data['p']  # 环境参数
    uav_swarm = data['uav_swarm']  # 初始化无人机群
    target_swarm = data['target_swarm']  # 初始化目标
    pickle_file.close()
    draw_all(p, uav_swarm, target_swarm)
