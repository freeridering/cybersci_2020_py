import pickle
import matplotlib.pyplot as plt
import numpy as np
import math
import parameter
import uav
import target
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    try:
        pickle_file = open('result.pkl', 'rb')
        data = pickle.load(pickle_file)
        p = data['p']  # 环境参数
        uav_swarm = data['uav_swarm']  # 初始化无人机群
        target_swarm = data['target_swarm']  # 初始化目标
    except Exception as e:
        print(str(e))
        exit()
    fig = plt.figure(figsize=(15, 4))
    # 已探索的范围
    ax2 = fig.add_subplot(1, 2, 1)
    plt.plot(range(p.time_limit), p.detect_list)
    plt.ylim(0, 1)
    plt.xlim(0, p.time_limit)
    plt.title(' detect_list')
    # 找到的目标个数
    ax3 = fig.add_subplot(1, 2, 2)
    plt.plot(range(p.time_limit), p.fd_ct_list)
    plt.ylim(0, p.nt)
    plt.xlim(0, p.time_limit)
    plt.title(' fd_ct_list')
    plt.show()