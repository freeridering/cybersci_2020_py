import parameter
import numpy as np
import uav
import target


# 暂时没什么用的类
class EventScheduling:
    def __init__(self):
        self.event_list = []  # 事件列表
        self.timing = 0  # 计时器

    def add_evevt(self):
        """
        添加事件
        :return:
        """
        pass

    def fetch_event(self):
        """
        取符合要求的事件
        :return:
        """
        pass

    def scheduling(self, p: parameter.Parameter):
        """
        调度过程主体
        :return:
        """
        self.uav_step(p)
        self.target_step(p)

    def uav_step(self, p: parameter.Parameter):
        p.S_a_p = p.S_a
        p.S_r_p = p.S_r
        p.V = np.zeros([p.nx, p.ny], dtype=float)

    def target_step(self, p: parameter.Parameter):
        pass


def uav_step(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    p.S_a_p = p.S_a
    p.S_r_p = p.S_r
    p.V = np.zeros([p.nx, p.ny], dtype=float)
    for i in range(p.nu):
        p.detect_map[uavs.uavs[i].pnow[0], uavs.uavs[i].pnow[1]] = 1
        p.V[uavs.uavs[i].pnow[0], uavs.uavs[i].pnow[1]] = 1
        uavs.uavs[i].path.append(uavs.uavs[i].pos_now)  # 记录路径
        uavs.uavs[i].pos_past = uavs.uavs[i].pos_now  # 赋值上一时刻点
        uavs.uavs[i].pos_now = uavs.uavs[i].way_global[0:2]
        # 更新概率图
        detection(p, uavs, targets)
    p.S_a_p = p.S_a
    p.S_a = cal_S_a()
    p.S_r_p = p.S_r
    p.S_r = cal_S_r()
    search_way_local(p, uavs, targets)
    cal_S_d_i(p, uavs, targets)
    search_way_global(p, uavs, targets)



def search_way_global(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    pass


def cal_S_d_i(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    return []


def search_way_local(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):



def cal_S_r():
    return []


def cal_S_a():
    return  []


def detection(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    pass


def target_step(p: parameter.Parameter, uavs: uav.UavSwarm, targets: target.TargetSwarm):
    pass
