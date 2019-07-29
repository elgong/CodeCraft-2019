#!/usr/bin/env python
# -*- coding:utf-8 -*-
#@Time  : 19-3-19 下午8:35
#@Author: elgong_hdu
#@File  : diaodu.py.py

from Dataloader import *
import copy
from MapInterface import *
from MyQueue import my_queue
from Astar import *
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################


# 一个时间片的调度函数
def one_time_slice(array, map2d, cross_map={}, road_map={}, car_map={}):
    last_time_wait_car = []
    flag_raise = True
    flag_lock = False
    wait_car_list_pass_road = []
    wait_car_list = []
    # ########################step1   标记所有的车的状态
    # 遍历路口

    for cross_id in cross_map.keys():
        for road_id in cross_map[cross_id]:
            # 无路口
            if road_id == -1:
                continue

            # 解析道路上的信息
            road_length = road_map[road_id]["road_length"]
            road_max_speed = road_map[road_id]["road_max_speed"]

            # 单向 并且没有朝向这个路口
            if road_map[road_id]["isDuplex"] == 0 and road_map[road_id]["cross_2"] != cross_id:
                continue
            road_map[road_id]["n"] = 0
            # 开始处理
            for line in road_map[road_id][cross_id].keys():
                this_line_car_list = road_map[road_id][cross_id][line]
                # print(this_line_car_list)
                this_line_car_num = len(this_line_car_list)
                for i in range(this_line_car_num):
                    car_id = this_line_car_list[i][0]
                    car_where = this_line_car_list[i][1]
                    car_speed = car_map[car_id]["car_speed"]


                    if car_map[car_id]["car_now_where"] != car_where:
                        # print("car_id" + str(car_id))
                        flag = " 车位置信息错" + str(car_id) + "car_map:" + str(car_map[car_id]["car_now_where"]) + "但是" + str(car_where)
                        raise Exception(flag)
                     # 如果车的下个位置还在路上，并且前方无车，或者车辆不足当
                    if (min(car_speed, road_max_speed) + car_where) <= road_length:
                        # 本线路第一che
                        if i == 0:
                            car_map[car_id]["car_now_where"] = road_map[road_id][cross_id][line][0][1] = (
                                        min(car_speed, road_max_speed) + car_where)
                            #print("车到了 road:" +str(road_id) + "   where" + str(car_map[car_id]["car_now_where"]))
                            car_map[car_id]["state"] = "stop"
                            #print("----------------" + str(car_map[car_id]["car_now_where"]))

                        # 前方车不当路
                        elif (min(car_speed, road_max_speed) + car_where) < this_line_car_list[i-1][1]:
                            car_map[car_id]["car_now_where"] = road_map[road_id][cross_id][line][i][1] = (
                                        min(car_speed, road_max_speed) + car_where)
                            car_map[car_id]["state"] = "stop"
                            #print("车到了 road:" + str(road_id) + "   where" + str(car_map[car_id]["car_now_where"]))

                        # 前方车当路，且stop
                        elif car_map[this_line_car_list[i-1][0]]["state"] == "stop":
                            car_map[car_id]["car_now_where"] = road_map[road_id][cross_id][line][i][1] = \
                            road_map[road_id][cross_id][line][i - 1][1] - 1  # 跟到屁股后
                            car_map[car_id]["state"] = "stop"
                            #print("车到了 road:" + str(road_id) + "   where" + str(car_map[car_id]["car_now_where"]))

                        else:
                            car_map[car_id]["state"] = "wait"
                            wait_car_list.append(car_id)
                    # 超过马路了
                    else:
                        # 本线路第一che
                        if i == 0:
                            car_map[car_id]["state"] = "wait"
                            wait_car_list_pass_road.append(car_id)

                            # # 记录下优先级最高的车
                            # if road_map[road_id]["first_car_id"] == 0:
                            #     road_map[road_id]["first_car_id"] = car_id

                        # 前车不过马路，stop
                        elif car_map[this_line_car_list[i-1][0]]["state"] == "stop":
                            car_map[car_id]["car_now_where"] = road_map[road_id][cross_id][line][i][1] = \
                            road_map[road_id][cross_id][line][i - 1][1] - 1  # 跟到屁股后
                            car_map[car_id]["state"] = "stop"
                           # print("车到了 road:" + str(road_id) + "   where" + str(car_map[car_id]["car_now_where"]))
                            #print("!!!!!!!!!!!!!!!!!!")
                        elif car_map[this_line_car_list[i-1][0]]["state"] == "wait":
                            car_map[car_id]["state"] = "wait"
                            wait_car_list_pass_road.append(car_id)
                        else:
                            raise Exception("!!!!!!")


    #print(wait_car_list_pass_road)
    # ##################开始调度所有汽车啦
    wait_car_list_pass_road_num = -1
    wait_car_list_num = -1

    need_to_change_road = 0
    num = 0
    # road_map["n"] = 0
    cross_dic ={}
    # print("111111111111111111111111111111111111111111111111111111111")
    while len(wait_car_list_pass_road) != 0 or len(wait_car_list) != 0:

###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
###################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
        num += 1
        # print(num)
        # if num == 5:
        #     raise Exception("111111111111111")



        # 所有的路口遍历开始啦
        for cross_id in cross_map.keys():
            if cross_id not in cross_dic.keys():
                cross_dic[cross_id] = 0
            unsorted_road_id = cross_map[cross_id]  # 道路ID[上，右，下，左]
            sorted_road_id = sorted(unsorted_road_id)  # 排序后的道路ID
            flag_change_road = False
            flag_change_cross = False

            #
            fflag = 0

            # 按道路ID升序 处理
            for road_id in sorted_road_id:

                road_que = my_queue()

                road_que_unPass_road = []
                road_que_unPass_road.clear()
                if road_id == -1:  # 有些路口只接两个道路，空白为-1
                    continue
                if road_map[road_id]["isDuplex"] == 0 and road_map[road_id]["cross_2"] != cross_id:
                    continue

                # road_map[road_id]["first_car_id"] = 0
                # 解析道路上的信息
                road_length = road_map[road_id]["road_length"]
                #print("road_id    " + str(road_id))
                #print("road_length   " + str(road_length))
                road_max_speed = road_map[road_id]["road_max_speed"]

                # wait 状态的车，组成队列
                # print(road_id, cross_id)
                for leng in range(road_length):
                    # print(road_map[road_id])
                    for k in road_map[road_id][cross_id].keys():
                        # 某线有车
                        if len(road_map[road_id][cross_id][k]) != 0:
                            for l in range(len(road_map[road_id][cross_id][k])):
                                if (road_length - leng) == road_map[road_id][cross_id][k][l][1] and road_map[road_id][cross_id][k][l][0] in wait_car_list_pass_road:
                                    road_que.add(road_map[road_id][cross_id][k][l][0])  # 存住车的ID

                for leng in range(road_length):
                    # print(road_map[road_id])
                    for k in road_map[road_id][cross_id].keys():
                        # 某线有车
                        if len(road_map[road_id][cross_id][k]) != 0:
                            for l in range(len(road_map[road_id][cross_id][k])):
                                if (road_length - leng) == road_map[road_id][cross_id][k][l][1] and \
                                        road_map[road_id][cross_id][k][l][0] in wait_car_list:
                                    road_que_unPass_road.append(road_map[road_id][cross_id][k][l][0])  # 存住车的ID

                # if fflag == 0:
                #
                #     if road_que.length() > 0:
                #         cross_dic[cross_id] += 1
                #         fflag += 1
                #     if cross_dic[cross_id] == 5:
                #         raise Exception(cross_id)
                #         pass


                # 过路口的车
                # print("----------------------")
                while road_que.length() != 0:
                    # print(road_map[road_id]["first_car_id"])


                    flag_change_road = False
                    this_time_car_id = road_que.peek()  # 得到优先级最高的车
                    this_time_car_step = car_map[this_time_car_id]["step_count"]
                    this_time_car_speed = car_map[this_time_car_id]["car_speed"]
                    this_time_car_where = car_map[this_time_car_id]["car_now_where"]
                    if min(this_time_car_speed, road_max_speed) + this_time_car_where <= road_length:
                        raise Exception("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

                    # print(this_time_car_id)
                    if this_time_car_id == road_map[road_id]["first_car_id"]:
                        road_map[road_id]["first_car_id_count"] += 1
########################################################################################################################
######################################  动态规划开始   ###################################################################
########################################################################################################################
                    #if flag_raise == True and flag_lock == True and need_to_change_road < 3:
                        if road_map[road_id]["first_car_id_count"] >= 2:
                            #print("need_to_plan------------------------------------------------------------------------------")
                            # #print(car_map[16384])
                            #print(car_map[this_time_car_id]["best_plan_route"])


                            next_road_id = car_map[this_time_car_id]["best_plan_route"][this_time_car_step + 1]
                            # next_direction = estimate_by_car_id(road_id, next_road_id, cross_id, cross_map)

                            end_cross_id = car_map[this_time_car_id]["car_to"]
                            start_cross_id = cross_id
                            #print(road_id, cross_id)
                            # 当前道路想对于路口的方向
                            now_fangxiang = find_fangxiang_by_road(road_id, cross_id, cross_map)

                            # 下一路的方向
                            next_fangxiang = find_fangxiang_by_road(next_road_id, cross_id, cross_map)

                            aStar = AStar(map2d)
                            pathList = aStar.start(car_id, start_cross_id, end_cross_id, now_fangxiang, next_fangxiang, car_map, cross_map, road_map)
                            cross_1 = start_cross_id
                            this_car_new_best_road = []
                            # if pathList == None:
                            #     break
                            for point in pathList:
                                # print(point)
                                cross_2 = array[point.x[0]][point.y[0]]
                                this_car_new_best_road.append(
                                    find_road_id_by_2_Cross(cross_1, cross_2, road_map, cross_map))
                                cross_1 = cross_2

                            need_to_change_road += 1
                            car_map[this_time_car_id]["best_plan_route"] = car_map[this_time_car_id]["best_plan_route"][:car_map[this_time_car_id]["step_count"]+1] + this_car_new_best_road
                            # print(car_map[this_time_car_id]["best_plan_route"])
                            # if need_to_change_road == 3:
                            #     flag_lock == False
                            #     need_to_change_road = 0
                            del aStar
                            # raise Exception("111111s")
                            # print(" 动态规划玩陈个")

                        # 动态规划
                    else:
                        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        road_map[road_id]["first_car_id"] = this_time_car_id
                        road_map[road_id]["first_car_id_count"] = 1


                    #print("处理这个   " + str(this_time_car_id))
                    # this_time_car_step = car_map[this_time_car_id]["step_count"]  # 这个车走过的街区个数，过去的才算
                    #  this_time_car_from_road = car_map[this_time_car_id]["best_plan_route"][this_time_car_step]






                    # 如果 没有超过路口
                    if min(this_time_car_speed, road_max_speed) + this_time_car_where <= road_length:
                        this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id, road_id, road_map)
                        front_car_id, front_car_weizhi, front_car_num = is_hava_a_car_at_front(this_time_car_id,
                                                                                               cross_id, road_id,
                                                                                               this_line, road_map)
                        #print("没超过路口")
                        #print("前车位置car    " + str(front_car_weizhi))
                        # 如果前方无车
                        if front_car_id == -1:
                            car_map[this_time_car_id]["car_now_where"] = road_map[road_id][cross_id][this_line][0][
                                1] = (
                                    min(this_time_car_speed, road_max_speed) + this_time_car_where)

                            road_que.pop()
                            wait_car_list_pass_road.remove(this_time_car_id)
                            car_map[this_time_car_id]["state"] = "stop"

                        # 如果有车，肯定是钟太了
                        elif car_map[front_car_id]["state"] == "stop":
                            #print(front_car_id, front_car_weizhi, front_car_num)
                            #print(car_map[road_map[road_id][cross_id][this_line][front_car_num-1][0]]["state"])
                            if car_map[road_map[road_id][cross_id][this_line][front_car_num-1][0]]["state"] != "stop":
                                raise Exception("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                            # 前车没影响
                            if (min(this_time_car_speed, road_max_speed) + this_time_car_where) < road_map[road_id][cross_id][this_line][front_car_num - 1][1]:
                                car_map[this_time_car_id]["car_now_where"] = \
                                road_map[road_id][cross_id][this_line][front_car_num][1] = (
                                    min(this_time_car_speed, road_max_speed) + this_time_car_where)
                                #print(" 移除队列 ：" + str(road_que.pop()))
                                road_que.pop()
                                wait_car_list_pass_road.remove(this_time_car_id)
                                car_map[this_time_car_id]["state"] = "stop"
                                #print("车到了 road:" + str(road_id) + "   where" + str(car_map[this_time_car_id]["car_now_where"]))

                            # 前车当路
                            else:
                                car_map[this_time_car_id]["car_now_where"] = road_map[road_id][cross_id][this_line][front_car_num][1] = road_map[road_id][cross_id][this_line][front_car_num - 1][1] - 1 #front_car_weizhi - 1
                                #print(" 移除队列 ：" + str(road_que.pop()))
                                road_que.pop()
                                wait_car_list_pass_road.remove(this_time_car_id)
                                car_map[this_time_car_id]["state"] = "stop"
                                #print("车到了 road:" + str(road_id) + "   where" + str(car_map[this_time_car_id]["car_now_where"]))
                                # print("!!!!!!前车当了，只能到" + str(front_car_weizhi-1))
                        else:
                            raise Exception("error")
                            continue
                    # 超过路口了
                    elif min(this_time_car_speed, road_max_speed) + this_time_car_where > road_length:
                        this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id, road_id, road_map)
                        front_car_id, front_car_weizhi, front_car_num = is_hava_a_car_at_front(this_time_car_id,
                                                                                               cross_id, road_id,
                                                                                            this_line, road_map)

                        #print("前方的车" + str(front_car_id))
                        # 到家了, 前方没车阻挡
                        if cross_id == car_map[this_time_car_id]["car_to"] and front_car_id == -1:
                            #print(" 移除队列 ：" + str(road_que.pop()))
                            # print(str(this_time_car_id) + "  到家")
                            road_que.pop()
                            wait_car_list_pass_road.remove(this_time_car_id)
                            car_map[this_time_car_id]["state"] = "stop"
                            this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id, road_id,
                                                                            road_map)
                            # 记录走过的路
                            car_map[this_time_car_id]["has_passed_route"].append(road_id)
                            car_map[this_time_car_id]["step_count"] += 1
                           # print("到了家啦" + str(this_time_car_id))

                            # 到家的车都是0
                            car_map[this_time_car_id]["car_now_where"] = 0
                            # 从图上删除
                            del road_map[road_id][cross_id][this_line][0]
                            continue
                            #print("车到了 road:" + str(road_id) + "   where" + str(car_map[this_time_car_id]["car_now_where"]))
                            # print(road_map[road_id][cross_id])
                        # 前方有车，stop
                        elif front_car_id != -1: #                  and car_map[front_car_id]["state"] == "stop":
                            car_map[this_time_car_id]["car_now_where"] = \
                            road_map[road_id][cross_id][this_line][front_car_num][1] = front_car_weizhi - 1
                           # print(" 移除队列 ：" + str(road_que.pop()))
                            road_que.pop()
                            wait_car_list_pass_road.remove(this_time_car_id)
                            car_map[this_time_car_id]["state"] = "stop"
                            continue
                           # print("!!!!!!前车当了，只能到" + str(front_car_weizhi - 1))
                        # elif front_car_id != -1 and car_map[front_car_id]["state"] == "wait" and cross_id == car_map[this_time_car_id]["car_to"]:
                        #     print("~!~!~!!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~!~~!~!")
                        #     break
                        # 前方无车
                        else:
                            # 车预计要进入的下一路ID
                            next_road_id = car_map[this_time_car_id]["best_plan_route"][this_time_car_step+1]
                           # print("this_time_car_id" + str(this_time_car_id))
                         #   print("---next road---------------" + str(next_road_id))
                            up = min(road_map[next_road_id]["road_max_speed"], this_time_car_speed) - (
                                        road_length - this_time_car_where)

                            # 保持在当前路口
                            if up <= 0:
                                this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id, road_id,
                                                                                road_map)
                                ############################# 下边不确定
                                car_map[this_time_car_id]["car_now_where"] = road_map[road_id][cross_id][this_line][0][1] = road_length
                              #  print(" 移除队列 ：" + str(road_que.pop()))
                                road_que.pop()
                                wait_car_list_pass_road.remove(this_time_car_id)
                                car_map[this_time_car_id]["state"] = "stop"
                                continue


                            # print(road_id, next_road_id)
                            next_road_id = car_map[this_time_car_id]["best_plan_route"][this_time_car_step + 1]

                            next_road_id = car_map[this_time_car_id]["best_plan_route"][this_time_car_step + 1]
                            next_direction = estimate_by_car_id(road_id, next_road_id, cross_id, cross_map)
        
                ###############  这里需要考虑下个道路的啥傻傻了
                            if next_direction == "front":
                               # print("front")
                                cross1_id = road_map[next_road_id]["cross_sum"] - cross_id  # 新道路通向的路口

                                # 毒死了标志, 下面需要重新规划
                                flag_all_road_can_not_set = len(road_map[next_road_id][cross1_id].keys())

                                # 遍历新道路的每一条线，找到车进入的位置
                                for cross1_id_key in road_map[next_road_id][cross1_id].keys():

                                    flag_all_road_can_not_set -= 1
                                    # 前方没车，或者车很远
                                    if len(road_map[next_road_id][cross1_id][cross1_id_key]) == 0 or road_map[next_road_id][cross1_id][cross1_id_key][-1][1] > up:  # 新道路没车，直接放在地一个
                                        road_map[next_road_id][cross1_id][cross1_id_key].append([this_time_car_id, up])
                                        car_map[this_time_car_id]["car_now_where"] = up
                                        car_map[this_time_car_id]["step_count"] += 1
                                        road_que.pop()
                                     #   print(" 移除队列11 ：" + str(road_que.pop()))
                                      #  print(this_time_car_id)
                                        wait_car_list_pass_road.remove(this_time_car_id)
                                     #   print(wait_car_list)
                                      #  print("---------------------------")
                                        car_map[this_time_car_id]["state"] = "stop"
                                        flag_all_road_can_not_set = 100  # 100没有实际意义，只是让循环完不等于0

                                        this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id,
                                                                                        road_id,
                                                                                        road_map)
                                        # 从图上删除
                                        del road_map[road_id][cross_id][this_line][0]
                                        # 记录走过的路
                                        car_map[this_time_car_id]["has_passed_route"].append(road_id)
                                      #  print("车到了 road:" + str(road_id) + "   where" + str(
                                      #      car_map[car_id]["car_now_where"]))
                                        break
                                    # 前方有车，当道了, 还是wait
                                    elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]]["state"] == "wait":
                                       # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                                        flag_change_road = True
                                        break
                                    elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]]["state"] == "stop" and road_map[next_road_id][cross1_id][cross1_id_key][-1][1] > 1:
                                        new_ = road_map[next_road_id][cross1_id][cross1_id_key][-1][1] - 1
                                        road_map[next_road_id][cross1_id][cross1_id_key].append([this_time_car_id, new_])
                                        car_map[this_time_car_id]["car_now_where"] = new_
                                        car_map[this_time_car_id]["step_count"] += 1
                                        road_que.pop()
                                     #   print(" 移除队列 ：" + str(road_que.pop()))
                                        wait_car_list_pass_road.remove(this_time_car_id)
                                        car_map[this_time_car_id]["state"] = "stop"
                                        this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id,
                                                                                        road_id,
                                                                                        road_map)
                                        flag_all_road_can_not_set = 100  # 100没有实际意义，只是让循环完不等于0
                                        # 从图上删除
                                        del road_map[road_id][cross_id][this_line][0]
                                        # 记录走过的路
                                        car_map[this_time_car_id]["has_passed_route"].append(road_id)

                                        break
                                    # elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]]["state"] == "stop" and road_map[next_road_id][cross1_id][cross1_id_key][-1][1] == 1:
                                # 有等待车，那我也等
                                if flag_change_road == True:
                              #      print("--------------------------------------------------------------------------------------------------------------------")
                                    # flag_change_cross = True
                                    break

                                if flag_all_road_can_not_set == 0:
                                    car_map[this_time_car_id]["car_now_where"] = \
                                    road_map[road_id][cross_id][this_line][0][1] = road_length
                                    road_que.pop()
                                    wait_car_list_pass_road.remove(this_time_car_id)
                                    car_map[this_time_car_id]["state"] = "stop"

                                    # if flag_raise == True:
                                    #     print(cross_id)
                                    #     raise Exception(" 需要调度了")
                                continue

###############################  该拐弯了
                            # 如果左转
                            if next_direction == "left":

                                #print("left")
                                #  查看右侧道路的ID
                                right_road_first_car_fangxiang = -1
                                right_road_id = find_road_id(road_id, "right", cross_id, cross_map)

                                if right_road_id != -1:
                                    # print(cross_id, right_road_id)
                                    right_road_first_car_fangxiang = find_first_car(cross_id, right_road_id, road_map, cross_map, car_map)
                                    # print(right_road_first_car_fangxiang)
                                # 因为调度时 右侧道路不一定是下一个要调度的id, 所以需要查找右侧道路的第一个车状态
                                # right_road_first_car_fangxiang = 0
                                # if right_road_first_car_id != 0:
                                #     print("right_road_first_car_id" + str(right_road_first_car_id))
                                #     right_car_step_count = car_map[right_road_first_car_id]["step_count"]
                                #     right_road_first_car_Next_road = \
                                #     car_map[right_road_first_car_id]["best_plan_route"][right_car_step_count + 1]
                                #     right_road_first_car_fangxiang = estimate_by_car_id(right_road_id,
                                #                                                         right_road_first_car_Next_road,
                                #                                                         cross_id, cross_map)
                                #     print("right_road_first_car_Next_road" + str(right_road_first_car_Next_road))
                                #     print("right_road_first_car_fangxiang" + str(right_road_first_car_fangxiang))

                                # 右侧没有道路 || 右侧道路没车 || 有车但不是直行
                                # 可以直接左行
                                if right_road_id == -1 or right_road_first_car_fangxiang != "front":
                                    #print(right_road_first_car_fangxiang=="front")
                                    #print("11111111111111111111111111")

                                    cross1_id = road_map[next_road_id]["cross_sum"] - cross_id  # 新道路通向的路口

                                    # 毒死了标志, 下面需要重新规划
                                    flag_all_road_can_not_set = len(road_map[next_road_id][cross1_id].keys())

                                    # 遍历新道路的每一条线，找到车进入的位置
                                    for cross1_id_key in road_map[next_road_id][cross1_id].keys():

                                        flag_all_road_can_not_set -= 1
                                        # 前方没车，或者车很远

                                        if len(road_map[next_road_id][cross1_id][cross1_id_key]) == 0 or \
                                                road_map[next_road_id][cross1_id][cross1_id_key][-1][1] > up:  # 新道路没车，直接放在地一个
                                            road_map[next_road_id][cross1_id][cross1_id_key].append([this_time_car_id, up])
                                            car_map[this_time_car_id]["car_now_where"] = up
                                            car_map[this_time_car_id]["step_count"] += 1
                                            road_que.pop()
                                       #     print(" 移除队列 ：" + str(road_que.pop()))
                                            wait_car_list_pass_road.remove(this_time_car_id)
                                            car_map[this_time_car_id]["state"] = "stop"
                                            flag_all_road_can_not_set = 100  # 100没有实际意义，只是让循环完不等于0

                                            this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id,
                                                                                            road_id,
                                                                                            road_map)
                                            # 从图上删除
                                            del road_map[road_id][cross_id][this_line][0]
                                            # 记录走过的路
                                            car_map[this_time_car_id]["has_passed_route"].append(road_id)
                                         #   print("可以左拐")

                                            break
                                        # 前方有车，当道了, 还是wait
                                        elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]]["state"] == "wait":
                                         #   print("左拐遇到前方车是等待")
                                            flag_change_road = True
                                            break
                                        elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]][
                                            "state"] == "stop" and road_map[next_road_id][cross1_id][cross1_id_key][-1][
                                            1] > 1:
                                            new_ = road_map[next_road_id][cross1_id][cross1_id_key][-1][1] - 1
                                            road_map[next_road_id][cross1_id][cross1_id_key].append(
                                                [this_time_car_id, new_])
                                            car_map[this_time_car_id]["car_now_where"] = new_
                                            car_map[this_time_car_id]["step_count"] += 1
                                            road_que.pop()
                                        #    print(" 移除队列 ：" + str(road_que.pop()))
                                            wait_car_list_pass_road.remove(this_time_car_id)
                                            car_map[this_time_car_id]["state"] = "stop"
                                            this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id,
                                                                                            road_id,
                                                                                            road_map)
                                         #   print("可以左拐")

                                            flag_all_road_can_not_set = 100  # 100没有实际意义，只是让循环完不等于0
                                            # 从图上删除
                                            del road_map[road_id][cross_id][this_line][0]
                                            # 记录走过的路
                                            car_map[this_time_car_id]["has_passed_route"].append(road_id)

                                            break
                                        elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]][
                                            "state"] == "stop" and road_map[next_road_id][cross1_id][cross1_id_key][-1][
                                            1] == 1:
                                            continue
                                        elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]][
                                            "state"] == "wait" and road_map[next_road_id][cross1_id][cross1_id_key][-1][
                                            1] == 1:
                                            raise Exception("为预料特别情况")
                                        else:
                                            raise Exception("为预料的特别情况")

                                        # elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]]["state"] == "stop" and road_map[next_road_id][cross1_id][cross1_id_key][-1][1] == 1:
                                    # 有等待车，那我也等
                                    if flag_change_road == True:
                                        # flag_change_cross = True
                                        break
###############################################################  调度
                                    if flag_all_road_can_not_set == 0:
                                        # if flag_raise == True:
                                        #     print(cross_id)
                                        #     raise Exception(" 需要调度了")
                                        car_map[this_time_car_id]["car_now_where"] = \
                                            road_map[road_id][cross_id][this_line][0][1] = road_length
                                        road_que.pop()
                                        wait_car_list_pass_road.remove(this_time_car_id)
                                        car_map[this_time_car_id]["state"] = "stop"
                                # 右侧直行，那就要结束本道路的调度
                                else:
                                    #
                                 #   print("右侧有车直行， 左转等待中。。。。。")
                                    break

                            # 如果右转
                            if next_direction == "right":
                             #   print("right")
                                # 查看左侧道路的ID
                                left_road_first_car_fangxiang = -1
                                front_road_first_car_fangxiang = -1
                                left_road_id = find_road_id(road_id, "left", cross_id, cross_map)
                                front_road_id = find_road_id(road_id, "front", cross_id, cross_map)

                                if left_road_id != -1:
                                    # print(cross_id, left_road_id)
                                    left_road_first_car_fangxiang = find_first_car(cross_id, left_road_id,
                                                                                    road_map, cross_map,
                                                                                    car_map)
                                if front_road_id != -1:
                             #       print(" 前方")
                                    # print(cross_id, front_road_id)
                                    front_road_first_car_fangxiang = find_first_car(cross_id, front_road_id,
                                                                                   road_map, cross_map,
                                                                                   car_map)
                                    # print(left_road_first_car_fangxiang)

                                # 先考虑等待的情况
                                # 左侧 直行
                                if left_road_id != -1 and left_road_first_car_fangxiang == "front":
                                   # print(" 右转时， 左侧有直行车")
                                    break

                                # 前方 左转
                                if front_road_id != -1 and front_road_first_car_fangxiang == "left":
                                 #   print(" 右转时， 前方有左转车")
                                    break

                              #  print("没有左拐，直行车")
                                # print(this_time_car_id)

                                # 可以直接右转啦
                                cross1_id = road_map[next_road_id]["cross_sum"] - cross_id  # 新道路通向的路口

                                # 毒死了标志, 下面需要重新规划
                                flag_all_road_can_not_set = len(road_map[next_road_id][cross1_id].keys())
                                #print(this_time_car_where)
                                #print(this_time_car_speed)
                                #print(road_map[next_road_id][cross1_id])
                                #print(car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]][
                                 #       "state"])

                                # 遍历新道路的每一条线，找到车进入的位置
                                for cross1_id_key in road_map[next_road_id][cross1_id].keys():

                                    flag_all_road_can_not_set -= 1
                                    # 前方没车，或者车很远
                                    if len(road_map[next_road_id][cross1_id][cross1_id_key]) == 0 or \
                                            road_map[next_road_id][cross1_id][cross1_id_key][-1][
                                                1] > up:  # 新道路没车，直接放在地一个
                                        road_map[next_road_id][cross1_id][cross1_id_key].append(
                                            [this_time_car_id, up])
                                        car_map[this_time_car_id]["car_now_where"] = up
                                        car_map[this_time_car_id]["step_count"] += 1
                                        road_que.pop()
                                        wait_car_list_pass_road.remove(this_time_car_id)
                                   #     print(" 移除队列22 ：" + str(road_que.pop()))

                                   #     print(wait_car_list)
                                        car_map[this_time_car_id]["state"] = "stop"
                                        flag_all_road_can_not_set = 100  # 100没有实际意义，只是让循环完不等于0

                                        this_line, this_weizhi = find_which_line_is_car(this_time_car_id,
                                                                                        cross_id,
                                                                                        road_id,
                                                                                        road_map)
                                        # 从图上删除
                                        del road_map[road_id][cross_id][this_line][0]
                                        # 记录走过的路
                                        car_map[this_time_car_id]["has_passed_route"].append(road_id)
                                        break
                                    # 前方有车，当道了, 还是wait
                                    elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]][
                                        "state"] == "wait":
                                        flag_change_road = True
                                        break
                                    elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]][
                                        "state"] == "stop" and \
                                            road_map[next_road_id][cross1_id][cross1_id_key][-1][
                                                1] > 1:
                                        new_ = road_map[next_road_id][cross1_id][cross1_id_key][-1][1] - 1
                                        road_map[next_road_id][cross1_id][cross1_id_key].append(
                                            [this_time_car_id, new_])
                                        car_map[this_time_car_id]["car_now_where"] = new_
                                        car_map[this_time_car_id]["step_count"] += 1
                                        road_que.pop()
                                     #   print(" 移除队列 ：" + str(road_que.pop()))
                                        wait_car_list_pass_road.remove(this_time_car_id)
                                        car_map[this_time_car_id]["state"] = "stop"
                                        this_line, this_weizhi = find_which_line_is_car(this_time_car_id,
                                                                                        cross_id,
                                                                                        road_id,
                                                                                        road_map)
                                        flag_all_road_can_not_set = 100  # 100没有实际意义，只是让循环完不等于0
                                        # 从图上删除
                                        del road_map[road_id][cross_id][this_line][0]
                                        # 记录走过的路
                                        car_map[this_time_car_id]["has_passed_route"].append(road_id)

                                        break
                                    elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]][
                                        "state"] == "stop" and \
                                            road_map[next_road_id][cross1_id][cross1_id_key][-1][
                                                1] == 1:
                                        continue
                                    else:
                                        raise Exception("为预料的特别情况")

                                    # elif car_map[road_map[next_road_id][cross1_id][cross1_id_key][-1][0]]["state"] == "stop" and road_map[next_road_id][cross1_id][cross1_id_key][-1][1] == 1:
                                # 有等待车，那我也等
                                if flag_change_road == True:
                                    # flag_change_cross = True
                                    break

                                if flag_all_road_can_not_set == 0:
                                    # if flag_raise == True:
                                    #     print(cross_id)
                                    #     raise Exception(" 需要调度了")
                                    car_map[this_time_car_id]["car_now_where"] = \
                                        road_map[road_id][cross_id][this_line][0][1] = road_length
                                    road_que.pop()
                                    wait_car_list_pass_road.remove(this_time_car_id)
                                    car_map[this_time_car_id]["state"] = "stop"
                    else:
                        raise Exception("11111")
                    #continue
                # 单独处理  不过路口的车
                this_index = 0
                # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                for index in range(len(road_que_unPass_road)):

                    this_time_car_id = road_que_unPass_road[this_index]  # 得到优先级最高的车

                    # print("处理这个   " + str(this_time_car_id))
                    this_time_car_step = car_map[this_time_car_id]["step_count"]  # 这个车走过的街区个数，过去的才算
                    #  this_time_car_from_road = car_map[this_time_car_id]["best_plan_route"][this_time_car_step]
                    this_time_car_speed = car_map[this_time_car_id]["car_speed"]
                    this_time_car_where = car_map[this_time_car_id]["car_now_where"]
                    # print(car_map[this_time_car_id])
                    #print(cross_id, road_id)
                    #print(this_time_car_id, this_time_car_where)
                    # print(road_map[5000])

                    # 如果 没有超过路口
                    if min(this_time_car_speed, road_max_speed) + this_time_car_where <= road_length:
                        this_line, this_weizhi = find_which_line_is_car(this_time_car_id, cross_id, road_id, road_map)
                        front_car_id, front_car_weizhi, front_car_num = is_hava_a_car_at_front(this_time_car_id,
                                                                                               cross_id, road_id,
                                                                                               this_line, road_map)
                        # print("没超过路口")
                        # print("前车位置car    " + str(front_car_weizhi))
                        # 如果前方无车
                        if front_car_id == -1:
                            car_map[this_time_car_id]["car_now_where"] = road_map[road_id][cross_id][this_line][0][
                                1] = (
                                    min(this_time_car_speed, road_max_speed) + this_time_car_where)

                            road_que_unPass_road.remove(this_time_car_id)
                            wait_car_list.remove(this_time_car_id)
                            # wait_car_list.remove(this_time_car_id)
                            car_map[this_time_car_id]["state"] = "stop"

                        # 如果有车，等待状态
                        elif car_map[road_map[road_id][cross_id][this_line][front_car_num - 1][0]]["state"] == "wait":
                            this_index += 1
                            continue
                        elif car_map[road_map[road_id][cross_id][this_line][front_car_num - 1][0]]["state"] == "stop":
                            # 前车没影响
                            if (min(this_time_car_speed, road_max_speed) + this_time_car_where) < \
                                    road_map[road_id][cross_id][this_line][front_car_num - 1][1]:
                                car_map[this_time_car_id]["car_now_where"] = \
                                    road_map[road_id][cross_id][this_line][front_car_num][1] = (
                                        min(this_time_car_speed, road_max_speed) + this_time_car_where)
                                # print(" 移除队列 ：" + str(road_que.pop()))
                                road_que_unPass_road.remove(this_time_car_id)
                                wait_car_list.remove(this_time_car_id)
                                # wait_car_list.remove(this_time_car_id)
                                car_map[this_time_car_id]["state"] = "stop"
                                # print("车到了 road:" + str(road_id) + "   where" + str(car_map[this_time_car_id]["car_now_where"]))

                            # 前车当路
                            else:
                                car_map[this_time_car_id]["car_now_where"] = \
                                road_map[road_id][cross_id][this_line][front_car_num][1] = \
                                road_map[road_id][cross_id][this_line][front_car_num - 1][1] - 1  # front_car_weizhi - 1
                                # print(" 移除队列 ：" + str(road_que.pop()))
                                road_que_unPass_road.remove(this_time_car_id)
                                wait_car_list.remove(this_time_car_id)
                                car_map[this_time_car_id]["state"] = "stop"
                                # print("车到了 road:" + str(road_id) + "   where" + str(car_map[this_time_car_id]["car_now_where"]))
                                # print("!!!!!!前车当了，只能到" + str(front_car_weizhi-1))
                        else:
                            raise Exception("error")
                        continue
                del road_que_unPass_road
                    # 超过路口了
        wait_car_list_pass_road_num = len(wait_car_list_pass_road)
        wait_car_list_num = len(wait_car_list)
    return

def set_all_car_state_to_unlabel(car_map={}):
    car_on_road_num = 0
    for car_id in car_map:
        car_map[car_id]["state"] = "unlabel"
        if car_map[car_id]["car_now_where"] != 0:
            car_on_road_num += 1
    return car_on_road_num


def set_all_road_clock_to_zero(road_map={}):
    for road_id in road_map.keys():
       # print(road_map[road_id])
        cross = road_map[road_id]["cross"]
        # print(cross)
        if str(cross[0]) in road_map[road_id].keys():
            road_map[road_id][str(cross[0])] = 0
           #  print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        if str(cross[1]) in road_map[road_id].keys():
            road_map[road_id][str(cross[1])] = 0



def  set_cat_to_road(car_id_list, time_slice, car_map={}, road_map={},):
    car_id_list = list(sorted(car_id_list))
    iindex = 0
    for l in range(len(car_id_list)):

        car_id = car_id_list[iindex]
        road_id1 = car_map[car_id]["best_plan_route"][0]
        road_id2 = car_map[car_id]["best_plan_route"][1]
        car_speed = car_map[car_id]["car_speed"]
        this_cross = find_cross_by_2_road(road_id1, road_id2, road_map)
        # print("find"+str(this_cross))

        flag_is_success_to_insert = False
        # 放车，直观最后一辆

        for line in road_map[road_id1][this_cross].keys():
            if len(road_map[road_id1][this_cross][line]) == 0:
                road_map[road_id1][this_cross][line].append([car_id, car_speed])
                car_map[car_id]["car_now_where"] = car_speed
                car_map[car_id]["planTime"] = time_slice + 1
                # print("----------")
                # if flag_test == True:
                #     print("添加的车de 位置road" + str(road_id1))
                #     print("添加的车de 位置line" + str(line))

                if car_map[car_id]["car_now_where"] != car_speed:
                    raise Exception("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
                flag_is_success_to_insert = True

                car_id_list.remove(car_id)
                break
            elif road_map[road_id1][this_cross][line][-1][1] == 1:
                continue
            elif road_map[road_id1][this_cross][line][-1][1] <= car_speed:
                # if len(road_map[road_id1][this_cross][line]) > 4:
                #     continue

                road_map[road_id1][this_cross][line].append(
                    [car_id, (road_map[road_id1][this_cross][line][-1][1] - 1)])
                car_map[car_id]["car_now_where"] = (road_map[road_id1][this_cross][line][-1][1])
                car_map[car_id]["planTime"] = time_slice + 1
                # if flag_test == True:
                #     print("添加的车de 位置road" + str(road_id1))
                #     print("添加的车de 位置line" + str(line))
                if car_map[car_id]["car_now_where"] != (road_map[road_id1][this_cross][line][-1][1]):
                    raise Exception("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
                flag_is_success_to_insert = True

                car_id_list.remove(car_id)
                break
            else:
                road_map[road_id1][this_cross][line].append([car_id, car_speed])
                car_map[car_id]["car_now_where"] = car_speed
                car_map[car_id]["planTime"] = time_slice + 1
                # if flag_test == True:
                #     print("添加的车de 位置road" + str(road_id1))
                #     print("添加的车de 位置line" + str(line))
                if car_map[car_id]["car_now_where"] != car_speed:
                    raise Exception("11111")
                flag_is_success_to_insert = True
                car_id_list.remove(car_id)
                break

    return car_id_list