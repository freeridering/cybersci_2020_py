import uav
import target
import parameter
from draw import *
import numpy as np
from scheduling import  *



if __name__ == '__main__':
    p = parameter.Parameter()  # 环境参数
    uavs = uav.UavSwarm(p)  # 初始化无人机群
    targets = target.TargetSwarm(p)  # 初始化目标
    time_counter = 0  # 步长计数器
    while time_counter < p.time_limit:
        p.fd_ct_list[time_counter] = p.found_counter
        detect_sum = np.sum(p.detect_map) / np.sum(p.g_map)
        p.detect_list[time_counter] = detect_sum
        uav_step(p, uavs, targets)
        target_step(p, uavs, targets)
        if p.found_counter == p.nt:
            print('在第%i步全都找到了' % time_counter)
            break
        if time_counter == 100:
            draw_map(1, p, uavs, targets)
        if time_counter == 200:
            draw_map(2, p, uavs, targets)
        if time_counter == 300:
            draw_map(3, p, uavs, targets)
        time_counter += 1
    print('main finished')
