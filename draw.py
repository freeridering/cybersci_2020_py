import parameter
import uav
import target


def draw_map(fig_id, p: parameter.Parameter, uavs:uav.UavSwarm, targets:target.TargetSwarm):
    uavs_list = list(uavs.uavs)  # 从set转化到list
    for i in range(len(uavs_list)):
        draw_uav_way(p, uavs_list[i])
    targets_list = list(targets.targets)
    for i in range(len(targets_list)):
        draw_tar_way(p, targets_list[i])


def draw_uav_way(p: parameter.Parameter, uav: uav.UavSingle):
    pass


def draw_tar_way(p: parameter.Parameter, target: target.TargetSingle):
    pass
