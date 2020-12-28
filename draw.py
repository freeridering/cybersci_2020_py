import parameter
import uav
import target
import pickle
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.pyplot import MultipleLocator
import parameter
import uav
import target


def set_color(value):
    if value == True:
        color = 'yellow'
    else:
        color = 'blue'
    return color


def draw(time_counter: int, fig: plt.Figure, p: parameter.Parameter, uav_swarm: uav.Uav_Swarm,
         target_swarm: target.Target_Swarm):
    #     pickle_file = open('result.pkl', 'rb')
    #     data = pickle.load(pickle_file)
    #     p = data['p']  # 环境参数
    #     uav_swarm = data['uav_swarm']  # 初始化无人机群
    #     target_swarm = data['target_swarm']  # 初始化目标
    #     pickle_file.close()

    fig.clear()
    ax1 = fig.add_subplot(111, aspect='equal')
    for i in range(p.nx):
        for j in range(p.ny):
            value = p.g_map[i, j]
            ax1.add_patch(
                plt.Rectangle(
                    (i * p.pixel_length_x, j * p.pixel_length_y),  # (x,y)矩形左下角
                    p.pixel_length_x,  # width长
                    p.pixel_length_y,  # height宽
                    color=set_color(value),
                    zorder=1
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
    plt.title('time_counter = ' + str(time_counter) + ' find_count = ' + str(p.found_counter))
    for i in range(p.nu):
        x_axis_data = uav_swarm[i].path[:, 0] * p.pixel_length_x + 1
        y_axis_data = uav_swarm[i].path[:, 1] * p.pixel_length_y + 1
        x_axis_data = [j for j in x_axis_data if j > -1]
        y_axis_data = [j for j in y_axis_data if j > -1]
        plt.plot(x_axis_data, y_axis_data, zorder=2)
    targets_xlist = [i.path[time_counter, 0] * p.pixel_length_x + 1 for i in target_swarm if
                     i.path[time_counter, 0] > -1]
    targets_ylist = [i.path[time_counter, 1] * p.pixel_length_y + 1 for i in target_swarm if
                     i.path[time_counter, 1] > -1]
    plt.scatter(targets_xlist, targets_ylist, s=20, c='red', marker='o', zorder=3)
    plt.pause(0.1)
    plt.draw()
