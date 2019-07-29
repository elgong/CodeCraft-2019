#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @Time  : 19-3-10 上午1:05
# @Author: elgong_hdu
# @File  : Dataloader.py
"""
# 最开始直接使用python3.6构建的地图, 该版本字典默认key升序排序。
# 网上说python3.5 版本没有排序。。。。。
# 自测的python3.5 是排序了的。。。。。。
# 把排序方法丢这里，方便未来修改吧
from collections import OrderedDict

old = {'''''}
new = OrderedDict()
sorted_key = sorted(old)

for key in sorted_key:
    if old.get(key) is not None:
        new[key] = old.get(key)
old = new  # 覆盖原来的字典

####### 注意情况 ###############
"""
from CoordinateSystem import *

def data_load(road_map={}, cross_map={}, car_map={},
                road_txt="../config_8/road.txt",
                cross_txt="../config_1/cross.txt",
                car_txt="../config_8/car.txt"):

    """ 加载 道路数据
         road_map = {路ID：{属性1：value1, 属性2：value2}}
         src = "../config/road.txt"
         road_map[id]:{
                 "road_length",
                 "road_max_speed",
                 "road_channel",
                 "isDuplex",
                 "cross":[cross1,cross2]
                 # 朝向路口1的线
                 cross_1 = {line1: []
                              line2：[]
                             }
                 # 朝向路口2的线
                 cross_2 = { line1: []
                             line2：[]
                             "left":0, "right":0, "front":0 # 记录转向数量
                             }


                // 以下参数被移除了
                // "cross_1",
                // "cross_2",
                 "line":{
                          zheng1: {
                             车辆ID1: 当前位置
                             车辆ID2: 当前位置
                          }
                          zheng2: {
                             车辆ID:                          }
                          ...
                 }
         }
     """
    with open(road_txt, "r") as f:
        # 遍历文件所有行
        for line in f:
            #  print(line)
            if "#" in line:  # 跳过第一行
                pass
            else:
                [road_id, road_length, road_max_speed, road_channel, cross_1, cross_2, isDuplex] = line.strip()[1:-1].split(",")

                # 首先是基本信息
                road_map[int(road_id)] = {
                    "road_length": int(road_length),  # 道路长度
                    "road_max_speed": int(road_max_speed),  # 限速
                    "road_channel": int(road_channel),  # 行车道
                    "cross": [int(cross_1), int(cross_2)],
                    "cross_1": int(cross_1),
                    "cross_2": int(cross_2),  # 连接的路口
                    "cross_sum": int(cross_1)+int(cross_2),  # 已知一个球另一个用到

                    "isDuplex": int(isDuplex),  # 是否双向
                    "first_car_id": 0,
                    "first_car_id_count": 0

                }

                ## 双向
                if road_map[int(road_id)]["isDuplex"] == 1:
                    for i in range(int(road_channel)):
                        if i == 0:
                            road_map[int(road_id)][int(cross_1)] = {i + 1: []}  # 朝向cross_1 的线：[car, 位置]
                            road_map[int(road_id)][int(cross_2)] = {i + 1: []}  # 朝向cross_2 的线：[car, 位置]
                            road_map[int(road_id)][str(int(cross_1))] = 0  # 路组
                            road_map[int(road_id)][str(int(cross_2))] = 0  # 露珠

                        else:
                            road_map[int(road_id)][int(cross_1)][i + 1] = []  # 朝向cross_1 的线：[car, 位置]
                            road_map[int(road_id)][int(cross_2)][i + 1] = []  # 朝向cross_2 的线：[car, 位置]
                ## 单向
                else:
                    for i in range(int(road_channel)):
                        if i == 0:
                            road_map[int(road_id)][int(cross_2)] = {i + 1: []}

                            road_map[int(road_id)][str(int(cross_2))] = 0  # 露珠
                        else:
                            road_map[int(road_id)][int(cross_2)][i + 1] = []
        #road_map["n"] = 0

    """ 
        加载 路口数据
        cross_map = {
                      路口ID1: [上，右，下，左]
                      路口ID2: [上，右，下，左]
                    }
    """
    with open(cross_txt, "r") as f:
        # 遍历文件所有行
        for line in f:
         #   print(line)
            if "#" in line:  # 跳过第一行
                pass
            else:
                [cross_id, road_up, road_right, road_down, road_left] = line.strip()[1:-1].split(",")

                # print("cross_id={}, road_up={}, road_right={}, road_down={}, road_left={}".format(cross_id, road_up, road_right, road_down, road_left))

                cross_map[int(cross_id)] = [int(road_up), int(road_right), int(road_down), int(road_left)]
        # 路口ID需要排序放入
       # sorted(cross_map.keys())

    """ 
        加载 车辆信息
        car_map = {}
        src = "../config/road.txt"
    """
    with open(car_txt, "r") as f:
        # 遍历文件所有行
        for line in f:
          #print(line)
            if "#" in line:  # 跳过第一行
                pass
            else:
                [car_id, car_from, car_to, car_speed, planTime] = line.strip()[1:-1].split(",")

                # if int(planTime) not in car_time.keys():
                #     car_time[int(planTime)] = [int(car_id)]
                # else:
                #     car_time[int(planTime)].append(int(car_id))
                # 首先是基本信息
                car_map[int(car_id)] = {
                   # "car_id": int(car_id), 多余
                    "car_from": int(car_from), # 起点
                    "car_to": int(car_to),   # 目的地
                    "car_speed": int(car_speed),  # 正常速度
                    "planTime": int(planTime),  # 计划出发时间
                    # "which_way": {
                    #     "road_Id": -1,   # 当前所处道路ID
                    #     "road_line": "0",   # 所处line
                    # },

                    "car_now_where": 0,

                    "best_plan_route": [],  # 当前最优规划路线  道路
                    "step_count": 0,  # 当前走过的道路个数， 让car_map[car_id]["best_plan_route"][step] 始终指向当前道路
                    "has_passed_route": [],  # 已经走过的路线
                    "real_start_time": int(planTime), # 实际出行时间
                    "state": "unlabel"
                }


if __name__ == "__main__":
#     """
#     调试使用
#     """
    #import sys

    #print(sys.version)
    road_map = {}
    cross_map = {}
    car_map = {}

    data_load(road_map, cross_map, car_map,
              road_txt="../config/road.txt",
              cross_txt="../config/cross.txt",
              car_txt="../config/car.txt")
    print(road_map[5000])
#
#
#     # a= []
#     # a.append(["down"])
#     # a.append(["up"])
#     # for i in a:
#     #     print(find_road_id_2(15, str(i[0]), cross_map, road_map))
# #
# #     print(len(cross_map))
# #     print(road_map[5010])
# #     print(cross_map)
#
#
#     array = make_coordinate_system(road_map, cross_map)
#     #print(np.shape(array))
#     map2d = Array2D(array)
#
#     #print(np.shape(map2d.data))
#     # 显示地图当前样子
#     map2d.showArray2D()
#     #print(map2d)
#     aStar = AStar(map2d, 38, 30)
#     pathList = aStar.start(cross_map, road_map)
#     has_passed_road = []
#     print(pathList)
#     cross_1 = 38
#     for point in pathList:
#         print(point)
#         cross_2 = array[point.x[0]][point.y[0]]
#         has_passed_road.append(find_road_id_by_2_Cross(cross_1, cross_2, road_map, cross_map))
#         cross_1 = cross_2
#     print(has_passed_road)
#     print("----------------------")





    #car_map[10002]["best_plan_route"]
    # print(cross_map[6][0])
    # print(road_map[5059]["road_max_speed"])
    # print(car_map[1000]["car_from"])
    #
    # print(estimate_road_id(5000, 1, "right", cross_map))
    # this_road_id = 5000
    # which = "front"
    # cross_id = 2
    # print(find_road_id(this_road_id, which, cross_id, cross_map))


