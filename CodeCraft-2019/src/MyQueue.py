#!/usr/bin/env python
# -*- coding:utf-8 -*-
#@Time  : 19-3-14 下午8:45
#@Author: elgong_hdu
#@File  : MyQueue.py

# python 自带的队列，无法查看队首元素而不弹出
# 这里简单实现了类似堆栈的 peek() 方法
class my_queue:
    """
        简单类实现
    """
    def __init__(self):
        self.items = []

    def is_empty(self):
        return self.items == []

    def add(self, item):
        self.items.insert(0, item)

    def pop(self):
        if len(self.items) == 0:
            return 0
        return self.items.pop()

    def length(self):
        return len(self.items)

    def peek(self):
        if len(self.items) == 0:
            return 0
        return self.items[-1]