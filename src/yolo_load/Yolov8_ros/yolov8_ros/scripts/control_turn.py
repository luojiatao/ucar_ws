#!/usr/bin/env python
# -*- coding: utf-8 -*-

#直线自动导航处理与输出控制
#by SR 黄朝炜 2024-1-15

import cv2
import rospy
import numpy as np
import time
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBoxes
import math

from geometry_msgs.msg import Twist
import sys, select, termios, tty
from std_msgs.msg import String

#初始等待3s启动程序
time.sleep(3)

# 初始化节点
rospy.init_node('yolo_to_turn', anonymous=True)

# 定义变量来保存订阅的主题和处理后的数据
turn = 0
go=1

# 定义回调函数来处理订阅的主题数据
def callback(data):
    if data.bounding_boxes != []:
        global turn
        global go
        n=int(len(data.bounding_boxes))  #记录总共有多少个检测结果
        cone_point_red=[]
        cone_point_blue=[]
        cone_point_yellow=[]
        for i in range(n):
            #提取并放置桶
            if data.bounding_boxes[i].Class == "red":
                new_point=[ (data.bounding_boxes[i].xmin+data.bounding_boxes[i].xmax)/2, 
                            (data.bounding_boxes[i].ymin+data.bounding_boxes[i].ymax)/2]
                cone_point_red.append(new_point)
            if data.bounding_boxes[i].Class == "blue":
                new_point=[ (data.bounding_boxes[i].xmin+data.bounding_boxes[i].xmax)/2, 
                            (data.bounding_boxes[i].ymin+data.bounding_boxes[i].ymax)/2]
                cone_point_blue.append(new_point)
            if data.bounding_boxes[i].Class == "yellow":
                new_point=[ (data.bounding_boxes[i].xmin+data.bounding_boxes[i].xmax)/2, 
                            (data.bounding_boxes[i].ymin+data.bounding_boxes[i].ymax)/2]
                cone_point_yellow.append(new_point)

        #默认画面width-1280，height-720
        w_point=1280/2
        h_point=720 

        #中心预瞄点提取
        # 计算红色和蓝色点的平均x坐标和y坐标
        if cone_point_red != [] and cone_point_blue != []:    
            average_x = sum([point[0] for point in cone_point_red + cone_point_blue]) / len(cone_point_red + cone_point_blue)
            average_y = sum([point[1] for point in cone_point_red + cone_point_blue]) / len(cone_point_red + cone_point_blue)
        # 得到平均点,并添加画面底部
            mid_points= [[w_point,h_point],[average_x, average_y]]
        else:
            mid_points=[]

        #排序黄色
        if cone_point_yellow != [] :
            cone_point_yellow = sorted(cone_point_yellow, key=lambda x: x[1])   #自动排序，从小到大
        if cone_point_red == [] and cone_point_blue == [] and cone_point_yellow !=[] :
            average_x = sum([point[0] for point in cone_point_yellow]) / len(cone_point_yellow)
            average_y = sum([point[1] for point in cone_point_yellow]) / len(cone_point_yellow)
            mid_points= [[w_point,h_point],[average_x, average_y]]


        if cone_point_yellow[-1][1] > (h_point*0.9):
            time.sleep(25)
            go=0    #转向就不管了
                   # time.sleep(2) #滞后停止
                    #print(cone_point_yellow[-1][1])

        #计算转向斜率,左正右负，中间0         
        if mid_points != [] and  mid_points[1][0] != mid_points[0][0]:
            theta = math.atan2 ( (mid_points[1][1] - mid_points[0][1]) , ( mid_points[1][0] - mid_points[0][0]) )
            angle_degrees= -(math.degrees(theta)+90)
        else:
            angle_degrees = 0
            #print(angle_degrees)
            #rospy.loginfo("Steering angle:%.2f degress",angle_degrees)

        #异常停止
        if angle_degrees<90:
            turn=angle_degrees/90
        else:
            turn=0
            go=0 

            #红蓝色桶已经没了，停止
            #if cone_point_red == [] and cone_point_blue == []:
              #  turn=0
             #   go=0

subscriber = rospy.Subscriber('/yolov8/BoundingBoxes', BoundingBoxes, callback)
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)


def publish_cmd_vel():
    global turn  # 引用外部变量
    if turn != 0 :  # 检查是否已接收到数据
        # 提取xmin值并转换为Twist消息的angular.z部分
        
        twist = Twist()  # 
        if go != 0 and abs(turn) < 0.9:
            twist.linear.x=0.5
            twist.linear.z=0    #亮闪黄，自动驾驶中
        else:
            twist.linear.x=-0.9
            twist.linear.z=1    #完成任务，亮蓝色

        if abs(turn) < 0.2 :
            twist.angular.z = 0
        else:
            twist.angular.z = turn

        print('linear:'+str(twist.linear.x)+'  angular:'+str(twist.angular.z))
        
        publisher.publish(twist)  # 发布Twist消息到cmd_vel话题
    #else:
        #rospy.loginfo("No data received yet.")  # 如果未收到数据，记录一条信息日志

# 循环调用publish_cmd_vel函数来发布Twist消息到cmd_vel话题
while not rospy.is_shutdown():
    publish_cmd_vel()
