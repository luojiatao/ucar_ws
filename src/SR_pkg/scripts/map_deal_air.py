#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# from matplotlib.pyplot import axis, plot
#from scipy import optimize
from numpy.core.defchararray import index
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from map_msgs.msg import OccupancyGridUpdate

from sr_pkg.srv import *


#global_location = [0,0,0] #x,y,wz
car_stop_point=[0,0] #x,y
car_local_pointx=True   #小车的局部相对x坐标
car_local_pointy=True   #小车的局部相对y坐标

map_data = OccupancyGridUpdate()
map_get = 0

def line_fun(x, A, B):
    return A * x + B

def Least_squares(x,y):
    x_ = x.mean()
    y_ = y.mean()
    m = np.zeros(1)
    n = np.zeros(1)
    k = np.zeros(1)
    p = np.zeros(1)
    for i in np.arange(0,len(x)):
        k = (x[i]-x_)* (y[i]-y_)
        m += k
        p = np.square( x[i]-x_ )
        n = n + p
    a = m/n
    b = y_ - a* x_
    return a,b

def show_map_img(msg):
    map_datalist=msg.data
    width=msg.width
    global car_local_pointx
    global car_local_pointy
    car_toflat_dis=0.4 #小车到平板的真实距离
    car_stop_point=[0,0] #小车开始节点时停车地方的xy坐标
    resolution=1/40 #分辨率
    point_list=[]   #障碍物激光点相对车辆的坐标
    result_po_list=[]   #最后筛选出来认为是障碍的点
    car_pose=np.array([]) #小车到每个板前面的姿态点
    po_class=2 #聚类算法的目标聚类数
    if car_local_pointx==True and car_local_pointy==True:
        index=len(map_datalist)-1
        car_local_pointx=((index%width)*resolution)/2
        car_local_pointy=((index/width)*resolution)/2
        #print("calculation now!")
    #print("car_x:",car_local_pointx,"car_y:",car_local_pointy)
    for index in range(0,len(map_datalist)):
        if map_datalist[index]>=90:
            x=(index%width)*resolution
            y=(index/width)*resolution
            point_tocarx_w=x-car_local_pointx+car_stop_point[0] #障碍的世界x坐标
            point_tocary_w=y-car_local_pointy+car_stop_point[1] #障碍的世界y坐标
            point_list.append([point_tocarx_w,point_tocary_w])
    range_x=np.array([-0.3,0.8])*0.8  #C区世界坐标x范围
    range_y=np.array([-0.5,0.5])*0.8 #C区世界坐标y范围
    #print("range_x:",range_x)
    #print('range_y:',range_y)
    for i in range(0,len(point_list)):
        if point_list[i][0]<range_x[0] or point_list[i][0]>range_x[1] or point_list[i][1]<range_y[0] or point_list[i][1]>range_y[1]:
            continue
        result_po_list.append(point_list[i])
    result_po_list=np.array(result_po_list)
    print(result_po_list)
    if len(result_po_list)<=3:
        print("no point!!!!!!")
        return car_pose #没有障碍返回空列表

    #聚类
    centre_init_index=np.random.choice(a=len(result_po_list),size=po_class,replace=False)    #中心点的坐标，随即初始化
    centre_point=[] #用以存放中心点
    for i in range(0,po_class):
        centre_point.append(result_po_list[centre_init_index[i]])
    del centre_init_index
    centre_point=np.array(centre_point)
    next_centre_point=centre_point+0.01
    class_list=[]
    while np.any(np.abs(next_centre_point-centre_point)>=0.0001):
        #print("in while")
        distance=[]
        class_list=[]   #分类的点
        for cl in range(0,po_class):
            dis=np.sum((result_po_list-centre_point[cl])**2,axis=1)
            distance.append(dis.tolist())
            class_list.append([])
        centre_point=next_centre_point.copy()
        #print(distance)
        distance=np.array(distance)
        
        min_list=np.argmin(distance,axis=0)  #求出每列的最小值
        for poindex,cl_num in enumerate(min_list,0):
            class_list[cl_num].append(result_po_list[poindex].tolist())
        class_list=np.array(class_list)
        #print(class_list)
        for cl_num in range(0,len(class_list)):
            next_centre_point[cl_num,:]=np.mean(class_list[cl_num],axis=0)
    #print(centre_point)

    #对聚类点做
    #print(type(class_list[0]))
    #y=Ax+B
    A=[]
    B=[]

    for cl in range(0,po_class):
        class_list_arr=np.array(class_list[cl])
        a, b = Least_squares(class_list_arr[:,0], class_list_arr[:,1])
        A.append(a)
        B.append(b)

    # for cl in range(0,po_class):
    #     x1 = np.arange(-0.5, 0.1, 0.001)#30和75要对应x0的两个端点，0.01为步长
    #     y1 = A[cl] * x1 + B[cl]
    #     plt.plot(x1, y1)
    
    # print('A:',A)
    # print('B:',B)

    car_pose=np.zeros((po_class,3))
    for i in range(0,po_class):
        vct_angle=math.atan2(centre_point[i][1]-car_stop_point[1],centre_point[i][0]-car_stop_point[0])
        c=0
        if vct_angle>0:
            c=B[i]-car_toflat_dis*math.sqrt(A[i]**2+1)
        else:
            c=B[i]+car_toflat_dis*math.sqrt(A[i]**2+1)  #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！需要添加判断是+还是-
        car_pose[i,0]=(centre_point[i][0]+A[i]*centre_point[i][1]-A[i]*c)/(1+A[i]**2)
        car_pose[i,1]=A[i]*car_pose[i,0]+c
        car_pose[i,2]=math.atan2(centre_point[i][1]-car_pose[i,1],centre_point[i][0]-car_pose[i,0])*57.3
        # x1=np.arange(-0.5,0.1,0.001)
        # y1=((-1/A[i])*(x1-centre_point[i][0])+centre_point[i][1]).copy()
        # plt.plot(x1,y1)
        # del x1,y1

    car_pose_dis=math.sqrt(((car_pose[0,0:2]-car_pose[1,0:2])**2).sum())
    print(car_pose)
    angle_diff=math.fabs(car_pose[0,2] - car_pose[1,2])
    if angle_diff>=360:
        angle_diff-=360
    if car_pose_dis >=0.3 or angle_diff>=60:
        return car_pose
    

    centre_mean=centre_point.mean(axis=0)
    
    car_pose=np.zeros(3)
    a,b=Least_squares(result_po_list[:,0],result_po_list[:,1])
    vct_angle=math.atan2(centre_mean[1]-car_stop_point[1],centre_mean[0]-car_stop_point[0])
    c=0
    if vct_angle>0:
        c=b-car_toflat_dis*math.sqrt(a**2+1)
    else:
        c=b+car_toflat_dis*math.sqrt(a**2+1)  #！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！需要添加判断是+还是-
    car_pose[0]=(centre_mean[0]+a*centre_mean[1]-a*c)/(1+a**2)
    car_pose[1]=a*car_pose[0]+c
    car_pose[2]=math.atan2(centre_mean[1]-car_pose[1],centre_mean[0]-car_pose[0])*57.3
    car_pose_local_x=car_pose[0]-car_stop_point[0]
    car_pose_local_y=car_pose[1]-car_stop_point[1]
    if car_pose_local_x<=-1 or car_pose_local_x>=1 or car_pose_local_y<=-1 or car_pose_local_y>=1:
        return []
    print(car_pose)


    # x1 = np.arange(-0.5, 0.1, 0.001)#30和75要对应x0的两个端点，0.01为步长
    # y1 = a * x1 + b
    # plt.plot(x1, y1)
    # x1 = np.arange(-0.5, 0.1, 0.001)#30和75要对应x0的两个端点，0.01为步长
    # y1 = -1/a * (x1-centre_mean[0])+centre_mean[1]
    # plt.plot(x1, y1)
    # x1 = np.arange(-0.5, 0.1, 0.001)#30和75要对应x0的两个端点，0.01为步长
    # y1 = a * x1+c
    # plt.plot(x1, y1)
    # plt.scatter(centre_mean[0],centre_mean[1])
    # plt.scatter(car_pose[0],car_pose[1])
    # plt.text(car_pose[0],car_pose[1],"car_pose")
    plt.scatter(result_po_list[:,0],result_po_list[:,1])
    # plt.scatter(centre_point[:,0],centre_point[:,1])
    # plt.xlim(-1,1)
    # plt.ylim(-1.1)
    plt.pause(0.4)     #设置暂停时间，太快图表无法正常显示
    plt.ioff()       # 关闭画图的窗口，即关闭交互模式
    plt.show()       # 显示图片，防止闪退
    # print(car_pose)
    return [car_pose]
    
def show_map_img_callback(msg):
    global map_get
    global map_data
    map_get = 1
    map_data = msg

def pose_callback(req):
    #global global_location
    global car_stop_point
    global map_get
    global map_data
    response = get_poseResponse()
    car_stop_point[0] = req.gobal_pose[0]
    car_stop_point[1] = req.gobal_pose[1]
    print(car_stop_point)
    #global_location[2] = req.gobal_pose[2]
    #data = rospy.wait_for_message("/move_base/local_costmap/costmap_updates",OccupancyGridUpdate,20)
    while not rospy.is_shutdown():
        if map_get ==1:
            break
    print(map_data)
    result = show_map_img(map_data)
    if len(result)!=0:
       response.count = len(result)
       response.pose1[0] =  result[0][0]
       response.pose1[1] =  result[0][1]
       response.pose1[2] =  result[0][2]
       if len(result)==2:  
          response.pose2[0] =  result[1][0]
          response.pose2[1] =  result[1][1]
          response.pose2[2] =  result[1][2]
    else:
       response.count = 0
    return response


if __name__ == "__main__":
    plt.ion()
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("listener_map")
    print("map_deal start!")
    #3.实例化 订阅者 对象
    sub1 = rospy.Subscriber("/move_base/local_costmap/costmap_updates",OccupancyGridUpdate,show_map_img_callback,queue_size=10)
    pose_srv = rospy.Service('pose_evaluate',get_pose,pose_callback)
    #sub2 = rospy.Subscriber()
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    rospy.spin()
