import logging
import sys

from CoordinateSystem import *
from MapInterface import *
from MyQueue import *
from Astar import *
import time
from tqdm import tqdm

# import threading  #######################3python 多线程只能单核？？？？ 解决不了
from multiprocessing import Pool
import multiprocessing

# ##################################  完全模拟出来了官方结果， 但是没有用啊 ###################################################################################
# ##################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
# ##################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
# ##################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
# ##################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################
# ##################################  调度核心啊，任务书中的 描述的是错的， 不是直到所有wait车都解决才结束， 而是路口遍历超过5次，就锁死啦###############################

"""
    step1: 静态规划 单车最优路径
    step2：动态调整 等待车辆
    step3：生成结果
"""


from diaodu import *
from dispatch import *

def main():

    if len(sys.argv) != 5:
        exit(1)

    # 本地调试打印
    flag_test = False

    # 路径接口
    car_path = sys.argv[1]
    road_path = sys.argv[2]
    cross_path = sys.argv[3]
    answer_path = sys.argv[4]

    # 地图的数据结构
    road_map = {}
    cross_map = {}
    car_map = {}

    # 给get_dispatch_list函数 传入当前时间用的字典
    time_slice = {}
    time_slice["time"] = 0

    # 从文件中加载 数据
    data_load(road_map, cross_map, car_map,
                  road_txt=road_path,
                  cross_txt=cross_path,
                  car_txt=car_path)

    # 建立地图的路口坐标系
    array = make_coordinate_system(road_map, cross_map)
    map2d = Array2D(array)

    # 存放车辆
    car_id_dic = {}
    car_id_dic["car"] = []
    # # 车自动生成, 速度降序, 车ID 升序
    # data_generate = get_dispatch_list_by_speed(30, time_slice, car_map)

    # # 车自动生成, 速度降序, 车ID 升序
    data_generate = get_dispatch_list_by_weizhi(70, array, car_id_dic, time_slice, car_map)

    # print(next(data_generate))

    # 寻路方法
    aStar = AStar(map2d)


    # 添加失败的车
    failed_insert_car_list = []

    # 路上在跑的车数量
    car_on_road_num = 0

    # 数据生成器空了的标志
    flag_remaining = 0  # 数据生成器没有数据，会提前返回0


    # 成功塞车标志
    flag_is_success_to_insert = False

    """
        时间片开始啦 时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦
        时间片开始啦 时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦  时间片开始啦

    """
    tmp_car_list = []
    # 时间片
    #for i in range(4000):
    for i in [j*3 for j in range(2000)]:
        # 跑完会自动结束, 4000 保底

        if flag_test== True:
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" + str(i+1))
        time_slice["time"] = i + 1
        # 一次时间片的调度
     #   start = time.time()



       # end = time.time()

       # print(end - start)
        # print(car_on_road_num)
        if car_on_road_num == 0 and i+1 > 5:
            break


        car_id_list, remaining = next(data_generate)
        # print(car_id_list)

        if flag_remaining != 1:
            # car_id_list = list(sorted(car_id_list))
            # 分批次寻路
            for car_id in car_id_list:
                end_cross_id = car_map[car_id]["car_to"]
                start_cross_id = car_map[car_id]["car_from"]

                pathList = aStar.start(car_id, start_cross_id, end_cross_id, 0, 0, car_map, cross_map, road_map, )
                cross_1 = start_cross_id
                car_map[car_id]["best_plan_route"].clear()
                for point in pathList:
                    cross_2 = array[point.x[0]][point.y[0]]
                    next_road = find_road_id_by_2_Cross(cross_1, cross_2, road_map, cross_map)

                    car_map[car_id]["best_plan_route"].append(next_road)
                    # 设置路组############################################### 条参数
                    road_map[next_road][str(cross_2)] = 1
                    cross_1 = cross_2
                if car_id == 14676:
                    print(car_map[car_id]["best_plan_route"])


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
                # print(road_map[road_id1])
                for line in road_map[road_id1][this_cross].keys():
                    if len(road_map[road_id1][this_cross][line]) == 0:
                        road_map[road_id1][this_cross][line].append([car_id, car_speed])
                        car_map[car_id]["car_now_where"] = car_speed
                        car_map[car_id]["planTime"] = i + 1
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
                        car_map[car_id]["planTime"] = i + 1
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
                        car_map[car_id]["planTime"] = i + 1
                        # if flag_test == True:
                        #     print("添加的车de 位置road" + str(road_id1))
                        #     print("添加的车de 位置line" + str(line))
                        if car_map[car_id]["car_now_where"] != car_speed:
                            raise Exception("11111")
                        flag_is_success_to_insert = True
                        car_id_list.remove(car_id)

                        break
                if flag_is_success_to_insert == False:
                    car_id_list.remove(car_id)
                    tmp_car_list.append(car_id)

        # 生成器空了的标志
        if remaining == 0:
            if len(tmp_car_list) !=0:
                car_id_dic["car"].extend(tmp_car_list)
                tmp_car_list.clear()
            else:
                flag_remaining = 1


        one_time_slice(array, map2d, cross_map, road_map, car_map)

        # 清空车的状态
        car_on_road_num = set_all_car_state_to_unlabel(car_map)

        one_time_slice(array, map2d, cross_map, road_map, car_map)

        # 清空车的状态
        car_on_road_num = set_all_car_state_to_unlabel(car_map)
        one_time_slice(array, map2d, cross_map, road_map, car_map)

        # 清空车的状态
        car_on_road_num = set_all_car_state_to_unlabel(car_map)

        one_time_slice(array, map2d, cross_map, road_map, car_map)

        # 清空车的状态
        car_on_road_num = set_all_car_state_to_unlabel(car_map)
        set_all_road_clock_to_zero(road_map)

    with open(answer_path, "a") as f:
        #        global car_map
        # print(car_map)
        f.write("#(carId,StartTime,RoadId)")
        f.write("\r\n")

        for car_id in car_map.keys():

            str_car = "(" + (str(car_id)) + (",") + (str(car_map[car_id]["planTime"])) + (",")

            for l in car_map[car_id]["has_passed_route"]:
                str_car = str_car + (str(l)) + ","
            str_car = str_car[:-1] + (")")
            # print(str_car)
            f.write(str_car)
            f.write("\r\n")
            del str_car


if __name__ == "__main__":
    start = time.time()
    main()
    end = time.time()

    print(end - start)
