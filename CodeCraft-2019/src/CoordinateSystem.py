# !/usr/bin/env python
# -*- coding:utf-8 -*-
# @Time  : 19-3-15 上午9:44
# @Author: elgong_hdu
# @File  : CoordinateSystem.py

from MapInterface import *
import numpy as np
np.set_printoptions(threshold=np.inf)

# 递归函数，处理每个点
def do_this_cross_id(this_cross_col, this_cross_row, cross_id, coor_list=[], road_map={}, cross_map={}):
    """
    :param this_cross_col:
    :param this_cross_row:
    :param coor_list:
    :param road_map:
    :param cross_map:
    :return:
    """
    # 递归结束条件
    if coor_list[this_cross_col][this_cross_row] is not 0:
        return

    coor_list[this_cross_col][this_cross_row] = cross_id
    for i in range(4):
        if cross_map[cross_id][i] == -1:  # 空的
            continue

        # 上方
        if i == 0:
          #  print("-----" + str(cross_id))
            front_road_id = cross_map[cross_id][0]
            front_cross_id = road_map[front_road_id]["cross_sum"] - cross_id
            col = this_cross_col - 1
            do_this_cross_id(col, this_cross_row, front_cross_id, coor_list, road_map, cross_map)

        # 右方
        elif i == 1:
            right_road_id = cross_map[cross_id][1]
            right_cross_id = road_map[right_road_id]["cross_sum"] - cross_id
            row = this_cross_row + 1
            do_this_cross_id(this_cross_col, row, right_cross_id, coor_list, road_map, cross_map)

        # 下方
        elif i == 2:
            down_road_id = cross_map[cross_id][2]
            down_cross_id = road_map[down_road_id]["cross_sum"] - cross_id
            col = this_cross_col + 1
            do_this_cross_id(col, this_cross_row, down_cross_id, coor_list, road_map, cross_map)
        # 左方
        elif i == 3:
            left_road_id = cross_map[cross_id][3]
            left_cross_id = road_map[left_road_id]["cross_sum"] - cross_id
            row = this_cross_row - 1
            do_this_cross_id(this_cross_col, row, left_cross_id, coor_list, road_map, cross_map)

def make_coordinate_system(road_map={}, cross_map={}):
    """
    :param road_map:
    :param cross_map:
    :return:

    # 思路：
        1. 生成一个 n×n 的二维列表，  n 为cross个数
        2. 根据每个点的相对位置， 从中心点开始赛入
        3. 删除空行，空列
    """

    cross_sum = len(cross_map)
    mid = int(cross_sum)   # 坐标中间值

    original_coor = [[0 for col in range(cross_sum*2)] for row in range(cross_sum*2)]
    # print(np.shape(original_coor))

    # 插入的位置  col=行   row=列
    row_to_insert = col_to_insert = mid

    first_cross_id = list(cross_map.keys())[0]
    # 递归、
    do_this_cross_id(mid, mid, first_cross_id, original_coor, road_map, cross_map)
    array = np.array(original_coor, dtype=int)

    #print(array)

    col_num = 0
    for col in array:
        if max(col) == 0:
            array = np.delete(array, col_num, axis=0)
        else:
            col_num += 1
    col_num = 0
    for row in range(cross_sum*2):
        if max(array[:, cross_sum*2 - row - 1]) == 0:
            array = np.delete(array, cross_sum*2 - row - 1, axis=1)
        else:
            col_num += 1

    # for i in array:
    #     print(i)
    # print(np.shape(original_coor))
    return array

