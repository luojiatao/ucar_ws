#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# 初始化CV Bridge
bridge = CvBridge()

def process_image(img):
    # 图像预处理
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # 边缘检测
    edges = cv2.Canny(blurred, 50, 150)
    # 轮廓寻找
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 遍历轮廓并进行曲线拟合
    for contour in contours:
        # 确保轮廓长度足够
        if len(contour) > 5:
            # 使用椭圆拟合轮廓
            ellipse = cv2.fitEllipse(contour)
            # 在原图上绘制椭圆
            cv2.ellipse(img, ellipse, (0,255,0), 2)
    return img

def image_callback(msg):
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    processed_image = process_image(cv_image)
    # 显示处理后的图像
    processed_image = cv2.resize(processed_image, (640, 480))  # 可以调整这里的尺寸以适应显示需求
    cv2.imshow("Image window", processed_image)
    cv2.waitKey(3)

def main():
    rospy.init_node('curve_detector', anonymous=True)
    rospy.Subscriber('/cam', Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
