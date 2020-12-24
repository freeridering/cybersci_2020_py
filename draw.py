import parameter
import uav
import target
import pickle
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.pyplot import MultipleLocator


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
    global pixel_length_x, pixel_length_y, ax1

    map(draw_cell, x_label_list, y_label_list, p_map)
    # map_result = np.reshape(list(map(draw_cell, x_label_list, y_label_list, p_map)), (p.nx, p.ny))
    plt.show()


def draw_cell(x, y, value):
    global pixel_length_x, pixel_length_y, ax1
    ax1.add_patch(plt.Rectangle((pixel_length_x * x, pixel_length_y * y), pixel_length_x, pixel_length_y,
                                color=(value, value, value, value))
                  )


def set_color(value):
    if value == True:
        color = 'yellow'
    else:
        color = 'blue'
    return color


if __name__ == '__main__':

    pickle_file = open('result.pkl', 'rb')
    data = pickle.load(pickle_file)
    p = data['p']  # 环境参数
    uav_swarm = data['uav_swarm']  # 初始化无人机群
    target_swarm = data['target_swarm']  # 初始化目标
    pickle_file.close()

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, aspect='equal')
    for i in range(p.nx):
        for j in range(p.ny):
            value = p.g_map[i, j]
            ax1.add_patch(
                plt.Rectangle(
                    (i * p.pixel_length_x, j * p.pixel_length_y),  # (x,y)矩形左下角
                    p.pixel_length_x,  # width长
                    p.pixel_length_y,  # height宽
                    color=set_color(value)
                )
            )
    x_pix = 4
    y_pix = 4
    xticks = [p.pixel_length_x * x_pix * i for i in range(math.ceil((p.nx + 1) / x_pix))]
    xtick_labes = [str(x_pix * i) for i in range(math.ceil((p.nx + 1) / x_pix))]
    plt.xticks(xticks, xtick_labes)
    yticks = [p.pixel_length_y * y_pix * i for i in range(math.ceil((p.ny + 1) / y_pix))]
    ytick_labes = [str(y_pix * i) for i in range(math.ceil((p.ny + 1) / y_pix))]
    plt.yticks(yticks, ytick_labes)
    plt.xlim(-5, p.nx * p.pixel_length_x)
    plt.ylim(-5, p.ny * p.pixel_length_y)
    plt.xlabel("x", fontsize=14)
    plt.ylabel("y", fontsize=14)
    for i in range(p.nu):
        x_axis_data = uav_swarm[i].path[:, 0]*p.pixel_length_x + 1
        y_axis_data = uav_swarm[i].path[:, 1]*p.pixel_length_y + 1
        x_axis_data = [j for j in x_axis_data if j > -1]
        y_axis_data = [j for j in y_axis_data if j > -1]

        plt.plot(x_axis_data,y_axis_data)
    plt.show()
