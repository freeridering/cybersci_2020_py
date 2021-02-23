import uav
import target
import parameter
import scheduling
import numpy as np


# 算法包
# 需求：多智能体搜索/优化算法，解决路径寻优问题

def pso():
    """
    粒子群
    :return:
    """
    pass


class GA:
    """
    遗传算法
    :return:
    """

    def __init__(self, p: parameter.Parameter):
        pass

    def coding(self):
        pass

    def init_swarm(self):
        pass

    def selection(self):
        pass

    def crossover(self):
        pass

    def mutation(self):
        pass

    def cal_fitness(self):
        pass

    def decoding(self):
        pass


def sa():
    """
    模拟退火算法

    :return:
    """


def ann():
    """
    人工神经网络
    :return:
    """
    pass


def de():
    """
    差分进化算法

    :return:
    """


# 暂定蚁群算法
def aco():
    """
    蚁群算法
    :return:
    """


# 简单迭代
# 若持续迭代n_i代后仍然没有优化则判定迭代终止
def interation(time_counter: int, uav_swarm: uav.Uav_Swarm, target_swarm: target.Target_Swarm, p: parameter.Parameter):
    """
    :param uav_swarm:
    :param target_swarm:
    :param p:
    :return:
    """
    p.inter_counter = 0
    p.last_J_group = -np.Inf
    while inter_restric(time_counter, uav_swarm, p):
        for i in range(p.nu):
            uav_swarm[i].all_way_local = scheduling.get_all_way_local(p, uav_swarm[i])
            scheduling.interation_global(time_counter, p, uav_swarm[i], uav_swarm, target_swarm)
        p.inter_counter += 1
    print("time_counter", time_counter, "interation end")


def inter_restric(time_counter: int, uav_swarm: uav.Uav_Swarm, p: parameter.Parameter):
    J_group = 0
    for i in range(p.nu):
        J_group += uav_swarm[i].Jmax[time_counter]
    print("inter_counter: ", p.inter_counter, "J_group: ", J_group)
    if J_group > p.last_J_group:
        # 找到优化组
        print("inter_counter: ", p.inter_counter, "update last_J_group", J_group)
        p.inter_counter = 0  # 重置计数器
        p.last_J_group = J_group
        for i in range(p.nu):
            if uav_swarm[i].way_global_inter[0, 0] == uav_swarm[i].way_global_inter[1, 0]:
                pass
            else:
                uav_swarm[i].way_global = np.copy(uav_swarm[i].way_global_inter)
        return True
    elif p.inter_counter > p.inter_limit:
        # 超过计数器但没有更优优化组，迭代终止
        return False
    else:
        return True
