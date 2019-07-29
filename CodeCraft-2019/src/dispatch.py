#!/usr/bin/env python
# -*- coding:utf-8 -*-
import os
import numpy as np
# 产生数据
def get_dispatch_list_by_speed(dispatch_num, time_slice = {}, car_map={}):
    """
    @author: ZXW
    :param dispatch_num:
    :param time_slice:
    :param car_map:
    :return:
    """

    car_id = []
    dic = {}

    # 先得到一个包含所有车的大列表, 按照车速降序
    for car_name in car_map.keys():  
        if car_map[car_name]["car_speed"] not in dic:
            dic[car_map[car_name]["car_speed"]]=[car_name]
        else:
            dic[car_map[car_name]["car_speed"]] += [car_name]

#### 需要额外排序

    car_key_list = []
    for car_key in dic.keys():
        car_key_list.append(car_key)
    for car_key in sorted(car_key_list, reverse=True):
        car_id += dic[car_key]
        # print(car_key)


    while True:
        car_num = 0
        num = 0
        dispatch_list = []
        now_time_slice = time_slice["time"]

        for i in range(len(car_id)):

            if car_num < min(len(car_id)+1, dispatch_num):
                if(car_map[car_id[num]]["planTime"] <= now_time_slice):
                    tmp = car_id[num]
                    dispatch_list.append(car_id[num])
                    car_id.remove(tmp)
                    car_num += 1
                else:
                    num += 1
            else:
                break
        dispatch_list.sort()
        yield dispatch_list, len(car_id)
        del dispatch_list


# 产生数据
def get_dispatch_list_by_weizhi(dispatch_num, array, car_id_dic ={}, time_slice={}, car_map={}):



    car_id = []
    car_id_list_speed=[]

    shuzhi_loss_min = []
    shuiping_loss_min = []
    rest = []
    dic = {}

    # 先得到一个包含所有车的大列表, 按照车速降序
    for car_name in sorted(list(car_map.keys())):
        if car_map[car_name]["car_speed"] not in dic:
            dic[car_map[car_name]["car_speed"]] = [car_name]
        else:
            dic[car_map[car_name]["car_speed"]] += [car_name]

    #### 需要额外排序

    car_key_list = []
    for car_key in dic.keys():
        car_key_list.append(car_key)
    for car_key in sorted(car_key_list, reverse=True):
        car_id_list_speed += dic[car_key]

    for car_id in sorted(list(car_map.keys())):
        start_x = np.where(array == car_map[car_id]["car_from"])[0][0]
        start_y = np.where(array == car_map[car_id]["car_from"])[1][0]

        end_x = np.where(array == car_map[car_id]["car_to"])[0][0]
        end_y = np.where(array == car_map[car_id]["car_to"])[1][0]

        delte_x = abs(end_x - start_x)
        delte_y = abs(end_y - start_y)

        if delte_x > 4 and delte_y <= 3:
            shuzhi_loss_min.append(car_id)
        elif delte_x <= 3 and delte_y > 4:
            shuiping_loss_min.append(car_id)
        else:
            rest.append(car_id)

    shuzhi_loss_min = [x for x in car_id_list_speed if x in shuzhi_loss_min]
    shuiping_loss_min = [x for x in car_id_list_speed if x in shuiping_loss_min]
    rest = [x for x in car_id_list_speed if x in rest]
    if len(shuzhi_loss_min) > len(shuiping_loss_min):
        car_id = shuzhi_loss_min + shuiping_loss_min + rest
    else:
        car_id = shuiping_loss_min + shuzhi_loss_min + rest
    car_id_dic["car"] = car_id

    while True:
        car_num = 0
        num = 0
        dispatch_list = []
        now_time_slice = time_slice["time"]
        #print(len(car_id_dic["car"]))

        for i in range(len(car_id_dic["car"])):

            if car_num < min(len(car_id_dic["car"]) + 1, dispatch_num):
                if (car_map[car_id_dic["car"][num]]["planTime"] <= now_time_slice):
                    tmp = car_id_dic["car"][num]
                    dispatch_list.append(car_id_dic["car"][num])
                    car_id_dic["car"].remove(tmp)
                    car_num += 1
                else:
                    num += 1
            else:
                break
        dispatch_list.sort()
        yield dispatch_list, len(car_id_dic["car"])
        del dispatch_list




