#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import os
import cv2
import numpy as np
import time
import threading


class UcarCamera:
    def __init__(self):
        rospy.init_node("ucar_camera", anonymous=True)
        
        self.img_width=int(rospy.get_param('~image_width',default=1280))
        self.img_height=int(rospy.get_param('~image_height',default=720))

        self.camera_topic_name=rospy.get_param('~cam_topic_name',default="/ucar_camera/image_raw")
        self.cam_pub=rospy.Publisher(self.camera_topic_name,Image,queue_size=1)          #定义发布器
        
        self.image_temp=Image()                    #创建一个ROS的用于发布图片的Image()消息
        self.image_temp.header.frame_id = 'opencv' #定义图片header里的id号
        self.image_temp.height=self.img_height        #定义图片高度
        self.image_temp.width=self.img_width          #定义图片宽度
        self.image_temp.encoding='rgb8'            #图片格式    
        self.image_temp.is_bigendian=True
        self.image_temp.step=self.img_width*3         #告诉ROS图片每行的大小 28是宽度3是3byte像素（rgb）
        ## 设置摄像头相关信息
        device_path=rospy.get_param('device_path',default="/dev/ucar_video")
        self.cap = cv2.VideoCapture(device_path)
        self.cap.set(3, self.img_width) 
        self.cap.set(4, self.img_height)
        codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
        self.cap.set(cv2.CAP_PROP_FOURCC, codec)
        self.cam_pub_rate=int(rospy.get_param('~rate',default=15))
        ros_rate = rospy.Rate(self.cam_pub_rate)  #定义发布频率
        while not rospy.is_shutdown():
            ros_rate = rospy.Rate(self.cam_pub_rate)  #初始化发布频率
            ret,frame_1 = self.cap.read()   ##ret 为布尔值表示是否可以获得图像    frame为获取的帧
            frame_1 = cv2.flip(frame_1,1) 
            self.frame = cv2.cvtColor(frame_1, cv2.COLOR_BGR2RGB) # 由OPENCV默认的BGR转为通用的RGB
            self.image_temp.header = Header(stamp=rospy.Time.now())   #定义图片header
            self.image_temp.data=np.array(self.frame).tostring()   #图片内容，这里要转换成字符串
            self.cam_pub.publish(self.image_temp)
            ros_rate.sleep()
       
if __name__ == '__main__':
    ucar_camera =  UcarCamera()


