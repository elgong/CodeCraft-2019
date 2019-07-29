#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @Time  : 19-3-14 下午8:42
# @Author: elgong_hdu
# @File  : MapInterface.py


# 给定两个道路，一个路口，确定转向
def estimate_by_car_id(cross_from, cross_to, cross_id, cross_map={}):
    """

    :param cross_from:  来的道路ID
    :param cross_to:   去的道路ID
    :param cross_id:  路口ID
    :param cross_map:
    :param road_map:
    :return:
    """
    from_index = cross_map[cross_id].index(cross_from)
    #print(cross_to,cross_id)
    #print(cross_id, cross_to)
    to_index = cross_map[cross_id].index(cross_to)

    sub = to_index - from_index
    if sub > 0:
        if sub == 1:
            return "left"
        elif sub == 2:
            return "front"
        elif sub == 3:
            return "right"
    elif sub < 0:
        if sub == -1:
            return "right"
        elif sub == -2:
            return "front"
        elif sub == -3:
            return "left"

# 给定一个朝向路口的道路ID，一个路口ID，  求某个方向的道路ID
def estimate_road_id(road_from, cross_id, fangxiang = "left", cross_map={}):
    from_index = cross_map[cross_id].index(road_from)

    if fangxiang == "left":
        if from_index == 3:
            return cross_map[cross_id][0]
        return cross_map[cross_id][from_index+1]
    if fangxiang == "right":
        if from_index == 0:
            return cross_map[cross_id][3]
        return cross_map[cross_id][from_index-1]
    if fangxiang == "front":
        if from_index < 2:
            return cross_map[cross_id][from_index+2]
        else:
            return cross_map[cross_id][from_index - 2]
        raise Exception(" 方向错了")

#
# 已知车ID, 调度的路口， 以及车在的道路，找到车所在的道路的具体线路和位置
def find_which_line_is_car(car_id, cross_id, road_id, road_map = {}):

    this_line = 1  # 默认当1吧
    this_weizhi = -1  # 位置
    road_to_this_cross_dic = road_map[road_id][cross_id]  # [cross_1] 对应的字典
    for line in road_to_this_cross_dic:
        for car in road_to_this_cross_dic[line]:
            if car[0] == car_id:
                this_line = line
                this_weizhi = car[1]
                break
        else:
            continue
        break
    if this_weizhi == -1:
        raise Exception("没找到车")
    return this_line, this_weizhi



# 查看某线的某个车前方是否有车
# return：
#       有车时： 车ID, 车的位置
#        无车： -1
def is_hava_a_car_at_front(car_id, cross_id, road_id, line, road_map = {}):
    front_car_id = -1  #
    front_car_weizhi = -1  # 位置
    num = 0  # 统计前车个数
    road_to_this_cross_dic = road_map[road_id][cross_id]  # [cross_1] 对应的字典
    for car in road_to_this_cross_dic[line]: # 遍历  [[id, weizhi], [id, weizhi]]
        if car[0] != car_id:
            front_car_id = car[0]
            front_car_weizhi = car[1]
            num += 1
        else:
            break

    if front_car_id == -1:
        return -1, -1, -1
    return front_car_id, front_car_weizhi, num

# 已知正在调度的路口，以及来车， 通过来车转向查找新道路 ID
def find_road_id(this_road_id, which, cross_id, cross_map={}):

    """
    :param this_road_id: 来车
    :param which: 转的方向
    :param cross_id:  当前调度路口
    :param cross_map:
    :return:  新道路ID
             -1   查不到返回-1
    """
    this_road_index = cross_map[cross_id].index(this_road_id)
    if which == "left":
        return cross_map[cross_id][(this_road_index + 1) % 4]
    elif which == "right":
        index = this_road_index - 1
        if index == -1:
            index = 3
        return cross_map[cross_id][index]
    elif which == "front":
        if this_road_index < 2:
            return cross_map[cross_id][this_road_index+2]
        else:
            return cross_map[cross_id][this_road_index-2]
    raise Exception(" 寻找道路ID出现未知意外")


# 查找线路
def find_road_id_2(cross_id, which, cross_map={}, road_map={}):
    """
    :param cross_id:
    :param which:
    :param cross_map:
    :return:
    """
    #print(cross_id)

    if which == "up" and cross_map[cross_id][0] != -1:
        if road_map[cross_map[cross_id][0]]["isDuplex"] == 0:
            #print(cross_map)
            if cross_id == road_map[cross_map[cross_id][0]]["cross_1"]:
                return cross_map[cross_id][0]
            else:
                return -1
        else:
            return cross_map[cross_id][0]

    elif which == "right" and cross_map[cross_id][1] != -1:
        #print(cross_map[cross_id][1])
        if road_map[cross_map[cross_id][1]]["isDuplex"] == 0:
            if cross_id == road_map[cross_map[cross_id][1]]["cross_1"]:
                return cross_map[cross_id][1]
            else:
                return -1
        else:
            return cross_map[cross_id][1]

    elif which == "down" and cross_map[cross_id][2] != -1:

        if road_map[cross_map[cross_id][2]]["isDuplex"] == 0:
            if cross_id == road_map[cross_map[cross_id][2]]["cross_1"]:
                return cross_map[cross_id][2]
            else:
                return -1
        else:
            return cross_map[cross_id][2]

    elif which == "left" and cross_map[cross_id][3] != -1:
        if road_map[cross_map[cross_id][3]]["isDuplex"] == 0:
            if cross_id == road_map[cross_map[cross_id][3]]["cross_1"]:
                return cross_map[cross_id][3]
            else:
                return -1
        else:
            return cross_map[cross_id][3]
    else:
        return -1


def find_road_id_by_2_Cross(cross_1, cross_2, road_map={}, cross_map={}):

    find_road_id = -1
    for road_id in cross_map[cross_1]:
        if road_id == -1:
            continue
        if road_map[road_id]["cross_sum"] - cross_1 == cross_2:
            find_road_id = road_id
    if find_road_id == -1:
        raise Exception("路线没找到！！！！")
    return find_road_id


def find_first_car(cross_id, road_id, road_map={}, cross_map={}, car_map={}):
    """
    :param cross_id: 调度路口
    :param road_id:
    :param road_map:
    :return:  -1 不影响
    """
    # 先找到排第一的车
    # print(cross_id, road_id,)
    road_length = road_map[road_id]["road_length"]
    car_id = -1
    if cross_id not in road_map[road_id].keys():
        return -1
    for leng in range(road_length):
        # print(road_map[road_id])
        for k in road_map[road_id][cross_id].keys():
            # 某线有车
            if len(road_map[road_id][cross_id][k]) != 0:
                for l in range(len(road_map[road_id][cross_id][k])):
                    if (road_length - leng) == road_map[road_id][cross_id][k][l][1]:
                        car_id = road_map[road_id][cross_id][k][l][0]
                        #print(car_id)
                        if car_map[car_id]["state"] == "stop":
                            return -1
                        else:

                            this_step = car_map[car_id]["step_count"]

                            ##############################
                            if this_step == len(car_map[car_id]["best_plan_route"])-1:
                                return "-1"
                            #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                            return estimate_by_car_id(car_map[car_id]["best_plan_route"][this_step], car_map[car_id]["best_plan_route"][this_step+1], cross_id, cross_map)
    return -1

# 开车，通过两个道路，找到中间的cross
def find_cross_by_2_road(road_id1, road_id2, road_map={}):
    road_id1_cross = road_map[road_id1]["cross"]
    road_id2_cross = road_map[road_id2]["cross"]
    for cross1 in road_id1_cross:
        for cross2 in road_id2_cross:
            if cross1 == cross2:
                return cross1


def is_finish_all_car(car_map={}):
    """
    :param car_map:
    :return:  True  完了
    """
    for car_id in car_map.keys():
        if car_map[car_id]["car_now_where"] != 0:
            return False
    return True



# 根据当前路口，找到当前道路方向

def find_fangxiang_by_road(road_id, cross_id, cross_map={}):
    """

    :param road_id:
    :param cross_map:
    :return:
    """
    index = cross_map[cross_id].index(road_id)
    if index == 0:
        return "up"
    elif index == 1:
        return "right"
    elif index == 2:
        return "down"
    else:
        return "left"




