#!/usr/bin/env python3
# coding:utf-8 
import numpy as np
import cv2
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import os
import threading
import sys


def capture(id):
    print ("已到达区域:{} ".format(id))
    cv2.imwrite("/home/luo/ucar_ws/src/sr_pkg/darknet/{}.jpg".format(id),frame2)
    print("截图成功!:Capture{}".format(id))


def callback(data):
    global frame2
    if data.data=="E1":
        capture("E1")

    if data.data=="E2":
        capture("E2")

    if data.data=="E3":
        capture("E3")

    if data.data=="E4":
        capture("E4")

    if data.data=="D1":
        capture("D1")

    if data.data=="D2":
        capture("D2")

    if data.data=="D3":
        capture("D3")

    if data.data=="D4":
        capture("D4")

    if data.data=="C1":
        capture("C1")

    if data.data=="C2":
        capture("C2")

    if data.data=="C3":
        capture("C3")

    if data.data=="C4":
        capture("C4")

    if data.data=="B1":
        capture("B1")

    if data.data=="B2":
        capture("B2")

    if data.data=="B3":
        capture("B3")

    if data.data=="B4":
        capture("B4")

    if data.data=="F1":
        capture("F1")

    if data.data=="F2":
        capture("F2")

    if data.data=="F3":
        capture("F3")

    if data.data=="F4":
        capture("F4")
        rospy.signal_shutdown("finish")

    # if data.data==3:
    #     sys.exit()

def main():
    global frame2
    print("启动: ucar_cam-ROS-node!")
    # cap = cv2.VideoCapture("/dev/ucar_video")
    cap = cv2.VideoCapture(0) 
    rospy.init_node('ucar_cam', anonymous=True)
    # bridge = CvBridge()
    # image_pub = rospy.Publisher("/usb_cam/image_raw",Image, queue_size=1)
    weight=640
    height=480
    cap.set(3, weight)  # 设置分辨率 3和4 分别代表摄像头的属性值。你可以使用函数 cap.get(propId) 来获得视频的一些参数信息。这里propId 可以是 0 到 18 之间的任何整数。每一个数代表视频的一个属性,见表其中的一些值可以使用cap.set(propId,value) 来修改,value 就是你想要设置成的新值。例如,我可以使用 cap.get(3) 和 cap.get(4) 来查看每一帧的宽和高。默认情况下得到的值是 640X480。但是我可以使用 ret=cap.set(3,320)和 ret=cap.set(4,240) 来把宽和高改成 320X240。
    cap.set(4, height)
    codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
    # print(codec)
    cap.set(cv2.CAP_PROP_FOURCC, codec)
    #fps =cap.get(cv2.CAP_PROP_FPS) #获取视频帧数
    # cap.set(cv2.CAP_PROP_AUTOFOCUS, False)  # 禁止自动对焦
    print("等待指示!")
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame2 = cv2.flip(frame,1)   ##图像左右颠倒
        # image_pub.publish(bridge.cv2_to_imgmsg(frame2, "bgr8"))
        # cv2.imshow('Camera_USB', frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

def thread_job():
    rospy.Subscriber("/goal_arrive", String, callback)
    rospy.spin()

  
if __name__ == '__main__':
    main()
