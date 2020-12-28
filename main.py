import uav
import target
import parameter
from draw import *
import numpy as np
from scheduling import *
import pickle
import typing

if __name__ == '__main__':
    try:
        pickle_file = open('data.pkl', 'rb')
        data = pickle.load(pickle_file)
        p = data['p']  # 环境参数
        uav_swarm = data['uav_swarm']  # 初始化无人机群
        target_swarm = data['target_swarm']  # 初始化目标
        pickle_file.close()
    except:
        p = parameter.Parameter()  # 环境参数
        uav_swarm = [uav.UavSingle(i, p) for i in range(p.nu)]
        target_swarm = [target.TargetSingle(i, p) for i in range(p.nt)]
        data = {'uav_swarm': uav_swarm, 'target_swarm': target_swarm, 'p': p}
        with open('data.pkl', 'wb') as f:
            pickle.dump(data, f, 0)
            f.close()
    time_counter = 0  # 步长计数器
    fig = plt.figure()
    while time_counter < p.time_limit:
        p.fd_ct_list[time_counter] = p.found_counter
        p.detect_list[time_counter] = np.sum(p.detect_map) / np.sum(p.g_map)
        uav_swarm_step(time_counter, p, uav_swarm, target_swarm)
        target_swarm_step(time_counter, p, uav_swarm, target_swarm)
        if p.found_counter == p.nt:
            print('在第%i步全都找到了' % time_counter)
            break
        draw(time_counter,fig, p, uav_swarm, target_swarm)

        time_counter += 1
    print('main finished')
    data = {'uav_swarm': uav_swarm, 'target_swarm': target_swarm, 'p': p}
    with open('result.pkl', 'wb') as f:
        pickle.dump(data, f, 0)
        f.close()
