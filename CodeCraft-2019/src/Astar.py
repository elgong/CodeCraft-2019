#!/usr/bin/env python
# -*- coding:utf-8 -*-
#@Time  : 19-3-16 下午9:50
#@Author: elgong_hdu
#@File  : Astar.py

import numpy as np
class Array2D:
    """
        说明：
            1.构造方法需要两个参数，即二维数组的宽和高
            2.成员变量w和h是二维数组的宽和高
            3.使用：‘对象[x][y]’可以直接取到相应的值
            4.数组的默认值都是0
    """
    def __init__(self, array):
        self.w = np.shape(array)[1]  # 二维数组宽
        self.h = np.shape(array)[0]  # 二维数组高
        self.data = array
        #print(np.shape(array))
        #print(np.shape(self.data))
        #self.data = [[0 for y in range(h)] for x in range(w)]  # 数组默认值0

    def showArray2D(self):
        for y in range(self.h):
            for x in range(self.w):
                print(self.data[y][x], end=' ')
            print("")

    def __getitem__(self, item):
        return self.data[item]


class Point:
    """
    表示一个点
    """
    import numpy as np
    def __init__(self, x, y):
        self.x = x  # np.shape(xy)[0]
        self.y = y  # np.shape(xy)[1]

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

    def __str__(self):
        return "x:" + str(self.x) + ",y:" + str(self.y)  # str() 函数将对象转化为适于人阅读的形式


# class AStar:
#     import numpy as np
#     """
#     AStar算法的Python3.x实现
#     """
#
#     class Node:  # 描述AStar算法中的节点数据
#         def __init__(self, this_point, end_point, g=0):  # point：自己的坐标。endpoint:终点坐标。g:从起始点移动到指定方格的移动耗费
#             self.point = this_point # 自己的坐标
#             self.father = None  # 父节点
#             self.g = g  # g值
#             self.h = (abs(end_point.x - self.point.x) + abs(end_point.y - self.point.y)) * 10  # 计算h值，h指从指定方格移动到终点方格的预计耗费
#
#     def __init__(self, map2d, start, end):
#         """
#         :param map2d:  地图
#         :param start:  开始路口
#         :param end:    结束路口
#         :param cross_map:
#         :param road_map:
#         """
#         # print(cross_map)
#         # 开启表
#         self.openList = []
#         # 关闭表
#         self.closeList = []
#         # 寻路地图
#         self.map2d = map2d
#         # print(np.shape(map2d))
#         x_start_point = np.where(map2d.data == start)[0]
#         y_start_point = np.where(map2d.data == start)[1]
#
#         x_end_point = np.where(map2d.data == end)[0]
#         y_end_point = np.where(map2d.data == end)[1]
#
#         # 起点终点
#         self.startPoint = Point(x_start_point, y_start_point)
#         self.endPoint = Point(x_end_point, y_end_point)
#         # 可行走标记
#         #self.passTag = passTag
#
#     def getMinNode(self):
#         """
#         获得openlist中F值最小的节点
#         return: Node
#         """
#         currentNode = self.openList[0]
#         for node in self.openList:
#             if node.g + node.h < currentNode.g + currentNode.h:
#                 currentNode = node
#         return currentNode
#
#     def pointInCloseList(self, point):
#         """
#         判断point是否在关闭表中
#
#         """
#         for node in self.closeList:
#             if node.point == point:
#                 return True
#         return False
#
#     def pointInOpenList(self, point):
#         """
#         判断point是否在开启表中
#         """
#         for node in self.openList:
#             if node.point == point:
#                 return node
#         return None  # 代表没有返回值
#
#     def endPointInCloseList(self):
#         """
#         判断终点是否在开启表中
#         """
#         for node in self.openList:
#             if node.point == self.endPoint:
#                 return node
#         return None
#
#     def searchNear(self, minF, offsetX, offsetY, car_id, car_map = {}, cross_map={}, road_map={}):
#         """
#         搜索节点周围的点
#         :param minF:F值最小的节点
#         :param offsetX:坐标偏移量
#         :param offsetY:
#         :return:
#         """
#
#         # 越界检测
#         if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.map2d.h - 1 or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.map2d.w - 1:
#             return
#
#         # 如果不可以通过，就借宿
#
#         now_cross_id = self.map2d.data[[minF.point.x], [minF.point.y]]
#
#         road_list = cross_map[now_cross_id[0][0]]
#
#         car_speed = car_map[car_id]["car_speed"]
#         step = 0
#         # 参数
#         a = 1
#         b = 1
#
#         # 向上
#         if offsetX == -1:
#             if road_list[0] == -1 or (road_map[road_list[0]]["isDuplex"] == 0 and road_map[road_list[0]]["cross_1"] != now_cross_id):
#                 return
#             road_max_speed = road_map[road_list[0]]["road_max_speed"]
#             road_length = road_map[road_list[0]]["road_length"]
#             this_road_cost_time = road_length / min(road_max_speed, car_speed)
#             loss_car_and_road_speed = abs(road_max_speed - car_speed)
#             step = this_road_cost_time * a + loss_car_and_road_speed * b
#
#         # 向下
#         if offsetX == 1:
#             if road_list[2] == -1 or (road_map[road_list[2]]["isDuplex"] == 0 and road_map[road_list[2]]["cross_1"] != now_cross_id):
#                 return
#             road_max_speed = road_map[road_list[2]]["road_max_speed"]
#             road_length = road_map[road_list[2]]["road_length"]
#             this_road_cost_time = road_length / min(road_max_speed, car_speed)
#             loss_car_and_road_speed = abs(road_max_speed - car_speed)
#             step = this_road_cost_time * a + loss_car_and_road_speed * b
#
#         # 向左
#         if offsetY == -1:
#             if road_list[3] == -1 or (road_map[road_list[3]]["isDuplex"] == 0 and road_map[road_list[3]]["cross_1"] != now_cross_id):
#                 return
#             road_max_speed = road_map[road_list[3]]["road_max_speed"]
#             road_length = road_map[road_list[3]]["road_length"]
#             this_road_cost_time = road_length / min(road_max_speed, car_speed)
#             loss_car_and_road_speed = abs(road_max_speed - car_speed)
#             step = this_road_cost_time * a + loss_car_and_road_speed * b
#
#         # 向you
#         if offsetY == 1:
#             if road_list[1] == -1 or (road_map[road_list[1]]["isDuplex"] == 0 and road_map[road_list[1]]["cross_1"] != now_cross_id):
#                 return
#             road_max_speed = road_map[road_list[1]]["road_max_speed"]
#             road_length = road_map[road_list[1]]["road_length"]
#             this_road_cost_time = road_length / min(road_max_speed, car_speed)
#             loss_car_and_road_speed = abs(road_max_speed - car_speed)
#             step = this_road_cost_time * a + loss_car_and_road_speed * b
#
#         # 如果在关闭表中，就忽略
#         currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)
#
#         if self.pointInCloseList(currentPoint):
#             return
#         # # 设置单位花费
#         # if offsetX == 0 or offsetY == 0:
#         #     step = 10
#         # else:
#         #     step = 14
#         # 如果不在openList中，就把它加入openlist
#         currentNode = self.pointInOpenList(currentPoint)
#         if not currentNode:  # 如果不为空
#             currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step)
#             currentNode.father = minF
#             self.openList.append(currentNode)
#             return
#         # 如果在openList中，判断minF到当前点的G是否更小
#         if minF.g + step < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
#             currentNode.g = minF.g + step
#             currentNode.father = minF
#
#     def start(self, car_id, car_map = {}, cross_map={}, road_map={}):
#         """
#         开始寻路
#         :return: None或Point列表（路径）
#         """
#         # 1.将起点放入开启列表
#         startNode = AStar.Node(self.startPoint, self.endPoint)
#         self.openList.append(startNode)
#         # 2.主循环逻辑
#         while True:
#             # 找到F值最小的点
#             minF = self.getMinNode()
#             # 把这个点加入closeList中，并且在openList中删除它
#             self.closeList.append(minF)
#             self.openList.remove(minF)
#             # 判断这个节点的上下左右节点
#             self.searchNear(minF, -1, 0, car_id, car_map, cross_map, road_map)
#             self.searchNear(minF, 1, 0,  car_id, car_map, cross_map, road_map)
#             self.searchNear(minF, 0, -1,  car_id, car_map, cross_map, road_map)
#             self.searchNear(minF, 0, 1,  car_id, car_map, cross_map, road_map)
#             # 判断是否终止
#             point = self.endPointInCloseList()
#             if point:  # 如果终点在关闭表中，就返回结果
#                 # print("关闭表中")
#                 cPoint = point
#                 pathList = []
#                 while True:
#                     if cPoint.father:
#                         pathList.append(cPoint.point)
#                         cPoint = cPoint.father
#                     else:
#                         return list(reversed(pathList))
#
#             if len(self.openList) == 0:
#                 return None


class AStar:
    import numpy as np
    """
    AStar算法的Python3.x实现
    """

    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, this_point, end_point, g=0):  # point：自己的坐标。endpoint:终点坐标。g:从起始点移动到指定方格的移动耗费
            self.point = this_point # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值
            self.h = (abs(end_point.x - self.point.x) + abs(end_point.y - self.point.y)) * 10  # 计算h值，h指从指定方格移动到终点方格的预计耗费

    def __init__(self, map2d):
        """
        :param map2d:  地图
        :param start:  开始路口
        :param end:    结束路口
        :param cross_map:
        :param road_map:
        """
        # print(cross_map)
        # 开启表
        self.openList = []
        # 关闭表
        self.closeList = []
        # 寻路地图
        self.map2d = map2d
        # print(np.shape(map2d))
        # x_start_point = np.where(map2d.data == start)[0]
        # y_start_point = np.where(map2d.data == start)[1]
        #
        # x_end_point = np.where(map2d.data == end)[0]
        # y_end_point = np.where(map2d.data == end)[1]
        #
        # # 起点终点
        # self.startPoint = Point(x_start_point, y_start_point)
        # self.endPoint = Point(x_end_point, y_end_point)
        self.step_num = 0
        # 可行走标记
        #self.passTag = passTag

    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        return: Node
        """
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):
        """
        判断point是否在关闭表中

        """
        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):
        """
        判断point是否在开启表中
        """
        for node in self.openList:
            if node.point == point:
                return node
        return None  # 代表没有返回值

    def endPointInCloseList(self):
        """
        判断终点是否在开启表中
        """
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None

    def searchNear(self, minF, offsetX, offsetY, car_id, car_map = {}, cross_map={}, road_map={}):
        """
        搜索节点周围的点
        :param minF:F值最小的节点
        :param offsetX:坐标偏移量
        :param offsetY:
        :return:
        """

        # 越界检测
        if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.map2d.h - 1 or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.map2d.w - 1:
            return

        # 如果不可以通过，就借宿
        #print([minF.point.x][0], [minF.point.y])

        now_cross_id = self.map2d.data[[minF.point.x], [minF.point.y]]

        now_cross_id_x = [minF.point.x][0][0]
        now_cross_id_y = [minF.point.y][0][0]
        # print(now_cross_id_x, now_cross_id_y)

        road_list = cross_map[now_cross_id[0][0]]

        car_speed = car_map[car_id]["car_speed"]
        step = 0
        # 参数 图2 653
        a = 5
        b = 0.4
        c = 4
        d = 10
        # 向上
        if offsetX == -1:
            if road_list[0] == -1 or (road_map[road_list[0]]["isDuplex"] == 0 and road_map[road_list[0]]["cross_1"] != now_cross_id):
                return
            road_max_speed = road_map[road_list[0]]["road_max_speed"]
            road_length = road_map[road_list[0]]["road_length"]
            road_channel = road_map[road_list[0]]["road_channel"]

            next_cross = road_map[road_list[0]]["cross_sum"] - now_cross_id
            #print(next_cross[0])

            # 路阻上来了
            #road_clock = road_map[road_list[0]][str(self.map2d.data[[np.array([now_cross_id_x-1])], np.array([now_cross_id_y])]]
            road_clock = road_map[road_list[0]][str(next_cross[0][0])]
            this_road_cost_time = road_length / min(road_max_speed, car_speed)
            loss_car_and_road_speed = abs(road_max_speed - car_speed)
            step = this_road_cost_time * a + loss_car_and_road_speed * b  + (1 - road_channel/4)*c + road_clock * d

        # 向下
        if offsetX == 1:
            if road_list[2] == -1 or (road_map[road_list[2]]["isDuplex"] == 0 and road_map[road_list[2]]["cross_1"] != now_cross_id ):
                return
            road_max_speed = road_map[road_list[2]]["road_max_speed"]
            road_length = road_map[road_list[2]]["road_length"]
            road_channel = road_map[road_list[2]]["road_channel"]
            next_cross = road_map[road_list[2]]["cross_sum"] - now_cross_id

            # 路阻上来了
            #road_clock = road_map[road_list[0]][str(self.map2d.data[[np.array([now_cross_id_x-1])], np.array([now_cross_id_y])]]
            road_clock = road_map[road_list[2]][str(next_cross[0][0])]

            this_road_cost_time = road_length / min(road_max_speed, car_speed)
            loss_car_and_road_speed = abs(road_max_speed - car_speed)
            step = this_road_cost_time * a + loss_car_and_road_speed * b  + (1- road_channel/4)*c + road_clock * d

        # 向左
        if offsetY == -1:
            if road_list[3] == -1 or (road_map[road_list[3]]["isDuplex"] == 0 and road_map[road_list[3]]["cross_1"] != now_cross_id):
                return
            road_max_speed = road_map[road_list[3]]["road_max_speed"]
            road_length = road_map[road_list[3]]["road_length"]
            road_channel = road_map[road_list[3]]["road_channel"]

            next_cross = road_map[road_list[3]]["cross_sum"] - now_cross_id

            # 路阻上来了
            #road_clock = road_map[road_list[0]][str(self.map2d.data[[np.array([now_cross_id_x-1])], np.array([now_cross_id_y])]]
            road_clock = road_map[road_list[3]][str(next_cross[0][0])]

            this_road_cost_time = road_length / min(road_max_speed, car_speed)
            loss_car_and_road_speed = abs(road_max_speed - car_speed)
            step = this_road_cost_time * a + loss_car_and_road_speed * b  + (1- road_channel/4)*c + road_clock * d

        # 向you
        if offsetY == 1:
            if road_list[1] == -1 or (road_map[road_list[1]]["isDuplex"] == 0 and road_map[road_list[1]]["cross_1"] != now_cross_id):
                return
            road_max_speed = road_map[road_list[1]]["road_max_speed"]
            road_length = road_map[road_list[1]]["road_length"]
            road_channel = road_map[road_list[1]]["road_channel"]
            next_cross = road_map[road_list[1]]["cross_sum"] - now_cross_id

            # 路阻上来了
            #road_clock = road_map[road_list[0]][str(self.map2d.data[[np.array([now_cross_id_x-1])], np.array([now_cross_id_y])]]
            road_clock = road_map[road_list[1]][str(next_cross[0][0])]

            this_road_cost_time = road_length / min(road_max_speed, car_speed)
            loss_car_and_road_speed = abs(road_max_speed - car_speed)
            step = this_road_cost_time * a + loss_car_and_road_speed * b  + (1- road_channel/4)*c + road_clock * d

        # 如果在关闭表中，就忽略
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)

        if self.pointInCloseList(currentPoint):
            return
        currentNode = self.pointInOpenList(currentPoint)
        if not currentNode:  # 如果不为空
            currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step)
            currentNode.father = minF
            self.openList.append(currentNode)
            return
        # 如果在openList中，判断minF到当前点的G是否更小
        if minF.g + step < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
            currentNode.g = minF.g + step
            currentNode.father = minF

    def start(self, car_id, start_cross_id, end_cross_id, now_fangxiang = 0, next_fangxiang = 0, car_map = {}, cross_map={}, road_map={}):
        """
        开始寻路
        :return: None或Point列表（路径）
        """


        # print(cross_map)
        # 开启表
        self.openList.clear()
        # 关闭表
        self.closeList.clear()
        # 寻路地图
        self.map2d
        # print(np.shape(map2d))
        x_start_point = np.where(self.map2d.data == start_cross_id)[0]
        y_start_point = np.where(self.map2d.data == start_cross_id)[1]

        x_end_point = np.where(self.map2d.data == end_cross_id)[0]
        y_end_point = np.where(self.map2d.data == end_cross_id)[1]

        # 起点终点
        self.startPoint = Point(x_start_point, y_start_point)
        self.endPoint = Point(x_end_point, y_end_point)
        # 1.将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint)
        self.openList.append(startNode)
        self.step_num = 0


        # 2.主循环逻辑
        flag = 0  # 方向限制只有一次
        while True:

            # 找到F值最小的点
            minF = self.getMinNode()
            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF)
            self.openList.remove(minF)

            # now_cross_id = self.map2d.data[[self.startPoint.x], [self.startPoint.x]]
            # full_road_num = cross_map[now_cross_id[0][0]].count(-1)


            # 判断这个节点的上下左右节点
            # 这里是动态规划用到的
            if flag == 0 and next_fangxiang != 0:
                flag = 1
                if next_fangxiang != "up" and now_fangxiang != "up":
                    self.searchNear(minF, -1, 0, car_id, car_map, cross_map, road_map)
                if next_fangxiang != "down" and now_fangxiang != "down":
                    self.searchNear(minF, 1, 0,  car_id, car_map, cross_map, road_map)
                if next_fangxiang != "left" and now_fangxiang != "left":
                    self.searchNear(minF, 0, -1, car_id, car_map, cross_map, road_map)
                if next_fangxiang != "right" and now_fangxiang != "right":
                    #print("!!!!")
                    self.searchNear(minF, 0, 1, car_id, car_map, cross_map, road_map)

                #print(minF)
                # 处理没路可以找的特别情况
                if len(self.openList) == 0:
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!")
                    if next_fangxiang == "up":
                        self.searchNear(minF, -1, 0, car_id, car_map, cross_map, road_map)
                    if next_fangxiang == "down":
                        self.searchNear(minF, 1, 0, car_id, car_map, cross_map, road_map)
                    if next_fangxiang == "left":
                        self.searchNear(minF, 0, -1, car_id, car_map, cross_map, road_map)
                    if next_fangxiang == "right":
                        self.searchNear(minF, 0, 1, car_id, car_map, cross_map, road_map)
            else:

                self.searchNear(minF, -1, 0, car_id, car_map, cross_map, road_map)
                self.searchNear(minF, 1, 0,  car_id, car_map, cross_map, road_map)
                self.searchNear(minF, 0, -1, car_id, car_map, cross_map, road_map)
                self.searchNear(minF, 0, 1,  car_id, car_map, cross_map, road_map)
            self.step_num += 1
            # 判断是否终止
            point = self.endPointInCloseList()
            if point:  # 如果终点在关闭表中，就返回结果
                # print("关闭表中")
                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        return list(reversed(pathList))

            if len(self.openList) == 0:
                return None

