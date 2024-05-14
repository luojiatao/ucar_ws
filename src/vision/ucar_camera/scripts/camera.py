#!/usr/bin/env python3
# coding:utf-8
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread
import sys

def main():
    print("启动 ucar_cam ROS 节点！")
    cap = cv2.VideoCapture("/dev/ucar_video")
    if not cap.isOpened():
        rospy.logerr("无法打开 ucar_video 设备！")
        return
    width, height = 640, 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    rospy.init_node('ucar_cam', anonymous=True)
    bridge = CvBridge()
    image_pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
    # 启动图像发布线程
    thread = Thread(target=publish_images, args=(cap, bridge, image_pub))
    thread.start()
    rospy.spin()  # 保持节点运行
    cap.release()
    cv2.destroyAllWindows()

def publish_images(cap, bridge, image_pub):
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            frame = cv2.flip(frame, 1)  # 水平翻转
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(image_msg)
        else:
            rospy.logwarn("摄像头读取帧失败")
    # 释放资源应当在此处进行，确保线程安全
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
