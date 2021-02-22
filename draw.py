import matplotlib.pyplot as plt
import numpy as np
import math
import parameter
import uav
import target
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D


def set_color(value):
    if value:
        color = 'yellow'
    else:
        color = 'blue'
    return color


def hot_map(time_counter: int, fig: plt.Figure, p: parameter.Parameter, uav_swarm: uav.Uav_Swarm,
            target_swarm: target.Target_Swarm):
    fig.clear()
    ax1 = fig.add_subplot(1, 2, 1)
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
    # 目标散点
    targets_xlist = [i.path[time_counter, 0] * p.pixel_length_x + 1 for i in target_swarm if
                     i.path[time_counter, 0] > -1]
    targets_ylist = [i.path[time_counter, 1] * p.pixel_length_y + 1 for i in target_swarm if
                     i.path[time_counter, 1] > -1]
    plt.scatter(targets_xlist, targets_ylist, s=20, c='blue', marker='s', zorder=3, label="targets")
    # uav散点
    uavs_xlist = [i.path[time_counter, 0] * p.pixel_length_x + 1 for i in uav_swarm if
                  i.path[time_counter, 0] > -1]
    uavs_ylist = [i.path[time_counter, 1] * p.pixel_length_y + 1 for i in uav_swarm if
                  i.path[time_counter, 1] > -1]

    plt.scatter(uavs_xlist, uavs_ylist, s=20, c='red', marker='s', zorder=3, label="uavs")
    # grid_map散点
    grid_xlist = [i * p.pixel_length_x + 1 for i in np.arange(0, p.nx, dtype=int) if i % (p.ox + 1) != 0]
    grid_xlist = [i for i in grid_xlist for _ in range(p.ny)]
    grid_ylist = [i * p.pixel_length_y + 1 for i in np.arange(0, p.ny, dtype=int) if i % (p.oy + 1) != 0] * p.nx
    plt.scatter(grid_xlist, grid_ylist, s=20, c='yellow', marker='s', zorder=2, label="grid")
    # 标签位置
    plt.legend(bbox_to_anchor=(1.05, 0), loc=3, borderaxespad=0)

    ax3 = fig.add_subplot(1, 2, 2)
    plt.xlim(-1, p.nx + 1)
    plt.ylim(-1, p.ny + 1)
    sns.heatmap(p.p_map * (p.nu ** 2), mask=p.p_map == 0, cmap="RdBu_r")
    plt.pause(0.1)
    plt.draw()
