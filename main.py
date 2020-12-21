import uav
import target
import parameter
from draw import *
import numpy as np
from scheduling import *
import pickle

if __name__ == '__main__':
    try:
        pickle_file = open('data.pkl', 'rb')
        data = pickle.load(pickle_file)
        p = data['p'] # 环境参数
        uavs = data['uavs']  # 初始化无人机群
        targets = data['targets'] # 初始化目标
        pickle_file.close()
    except:
        p = parameter.Parameter()  # 环境参数
        uavs = uav.UavSwarm(p)  # 初始化无人机群
        targets = target.TargetSwarm(p)  # 初始化目标
        data = {'uavs': uavs, 'targets': targets, 'p': p}
        with open('data.pkl', 'wb') as f:
            pickle.dump(data, f, 0)
            f.close()
    time_counter = 0  # 步长计数器
    while time_counter < p.time_limit:
        p.fd_ct_list[time_counter] = p.found_counter
        p.detect_list[time_counter] = np.sum(p.detect_map) / np.sum(p.g_map)
        uav_swarm_step(time_counter, p, uavs, targets)
        target_swarm_step(time_counter, p, uavs, targets)
        if p.found_counter == p.nt:
            print('在第%i步全都找到了' % time_counter)
            break
        if time_counter == 100:
            draw_map(1, p, uavs, targets)
        elif time_counter == 200:
            draw_map(2, p, uavs, targets)
        elif time_counter == 300:
            draw_map(3, p, uavs, targets)
        time_counter += 1
    print('main finished')
