#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy #导入rospy库
import actionlib #导入actionlib 库
import os,inspect #导入os库
from actionlib_msgs.msg import * #导入actionlib的所有模块
from geometry_msgs.msg import Pose, Point, Quaternion, Twist #导入四个消息数据类型，姿态，目标点，四元数，运动消息Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal #导入movebase的两个消息数据类型
from tf.transformations import quaternion_from_euler #导入tf变换库的欧拉角转四元数库
from math import pi #导入圆周率pi
from std_msgs.msg import String #导入标准消息的字符串消息数据格式
from std_msgs.msg import Int8
from darknet_msgs.msg import darknet
import yaml
import time
import threading
import shutil
from sr_pkg.srv import *


arcuo_id_list=["0","1","2"]
nav_goals=[]
goal_dict=dict()
place_name=""
aruco_id=""
final_result=""


#初始化节点
rospy.init_node('smartcar_test',anonymous=False)
square_size = 1.0
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5) #实例化一个消息发布函数
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction) #action服务器连接

goal_arrive_pub = rospy.Publisher('/goal_arrive',Int8,queue_size=1)

end_pub = rospy.Publisher('/move_base_simple/goal_arrive',Int8,queue_size=1)


#设置参数 
rospy.loginfo('等待move_base action服务器连接...')
move_base.wait_for_server(rospy.Duration(30))
rospy.loginfo('已连接导航服务')

yaml_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/goal.yaml"
#添加导航坐标点,输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
def nav_to(x,y,th):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id='map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose_e(x,y,th)
    move(goal)

def get_state(timeout):
    start_time = rospy.Time.now().to_sec()
    while  not rospy.is_shutdown():
        state = move_base.get_state()
        #print(str(state))
        now_time = rospy.Time.now().to_sec()
        if state == 3:
           return True
        if (now_time-start_time)>timeout:
           return False

#写一个函数 用于任务完成提示。
def move(goal):
    move_base.send_goal(goal)
    if get_state(30):
        rospy.loginfo(place_name+'导航成功！')
    else:
        while not rospy.is_shutdown():
            rospy.loginfo('时间超时，进入恢复状态，重新导航。')
            move_for(0,-1.2,5)
            move_base.send_goal(goal)
            if get_state(10):
                break
    
def shutdown():
    rospy.loginfo('机器人任务停止')
    move_base.cancel_goal()
    rospy.sleep(2)
    cmd_vel_pub.publish(Twist)
    rospy.sleep(1)
    
def pose_e(x,y,th):#输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
    new_pose=Pose()
    new_pose.position.x=float(x)
    new_pose.position.y=float(y)
    #机器朝向，平面朝向弧度转化成四元数空间位姿
    q=quaternion_from_euler(0.0,0.0,float(th)/180.0*pi)
    new_pose.orientation.x=q[0]
    new_pose.orientation.y=q[1]
    new_pose.orientation.z=q[2]
    new_pose.orientation.w=q[3]
    return  new_pose

def yaml_read():
    global goal_dict
    global nav_goals
    f = open(yaml_path) 
    cfg =  f.read()
    goal_dict = yaml.load(cfg)
    nav_goals=goal_dict.keys()


def nav_callback(data):
    g=data.data
    goal=g.decode("utf-8")
    if goal in nav_goals:
        goal_data=goal_dict[goal]
        nav_to(goal_data[0],goal_data[1],goal_data[2])

def aruco_callback(data):
    global aruco_id
    aruco_id = data.data

def start_callback(data):
    global start_game
    if 'g' in data.data:
        start_game = 1
    elif 'p' in data.data:
        start_game = 2

def nav_place(place):
    global place_name
    place_name=place
    print("尝试导航去:"+place_name)
    goal=place
    if goal in nav_goals:
        goal_data=goal_dict[goal]
        nav_to(goal_data[0],goal_data[1],goal_data[2])
        
        

def move_for(xspeed,tspeed,time_second):
    twist_data=Twist()
    twist_data.linear.x=xspeed
    twist_data.angular.z=tspeed
    time_start=time.time()
    while time.time()-time_start<time_second:
        cmd_vel_pub.publish(twist_data)
    cmd_vel_pub.publish(Twist())

def msg_sub():
    rospy.Subscriber("/easy_aruco_node/aruco_id",String,aruco_callback,queue_size=1)
    rospy.Subscriber("/start_move",String,start_callback,queue_size=1)

def thread_job():
    rospy.spin() 

def move_forward_x(xspeed,time_second):
    twist=Twist()
    twist.linear.x=xspeed
    time_start=time.time()
    while time.time()-time_start<time_second:
        cmd_vel_pub.publish(twist)
    cmd_vel_pub.publish(Twist())


if __name__ == "__main__":
    global person_list
    global get_result
    global start_game
   
    get_result = 0
    start_game = 0
    person_list = [0,0,0,0]
    yaml_read()
    msg_sub()
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    print("Wait for start!!!")
    move_forward_x(1.0,1.0)
    nav_place("E1")
    arrvie_id = Int8()
    arrvie_id.data=1
    goal_arrive_pub.publish(arrvie_id)
    nav_place("E2")
    rospy.sleep(1)
    rospy.wait_for_service("pose_evaluate",10)
    client = rospy.ServiceProxy("pose_evaluate", get_pose)
    request = get_poseRequest()

    # move_for(0,-1.0,1)

    pos_data=goal_dict["E2"]
    request.gobal_pose[0] = pos_data[0]
    request.gobal_pose[1] = pos_data[1]
    request.gobal_pose[2] = pos_data[2]
    
    move_for(0,-1.0,1)

    response = client.call(request)

    move_for(0,-1.0,1)

    print(response.pose1)
    print(response.pose2)
    if response.count!=0:
        print("goto pose1")
        nav_to(response.pose1[0],response.pose1[1],response.pose1[2])
        rospy.sleep(0.5)
        arrvie_id = Int8()
        arrvie_id.data=21
        goal_arrive_pub.publish(arrvie_id)
        if response.count ==2:    
            print("goto pose2")
            nav_to(response.pose2[0],response.pose2[1],response.pose2[2])
            rospy.sleep(0.5)
            arrvie_id = Int8()
            arrvie_id.data=22
            goal_arrive_pub.publish(arrvie_id)
    arrvie_id = Int8()
    arrvie_id.data=4
    goal_arrive_pub.publish(arrvie_id)
    # nav_place("D")
  
    
    
    #cmd="play ~/ucar_ws/src/sr_pkg/mp3/final.mp3"
    #os.system(cmd)
