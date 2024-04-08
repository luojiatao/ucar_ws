#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# 初始化CvBridge
bridge = CvBridge()

def imgmsg_to_cv2(data):
    """
    将ROS图像消息转换为OpenCV图像。
    """
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        return cv_image
    except CvBridgeError as e:
        print(e)
        return None

def process_binary_image(img):
    """
    对收到的图像进行二值化处理。
    """
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
    return binary_image

def detect_lines(binary_image, img):
    """
    在二值化图像上进行边缘检测和直线检测。
    """
    edges = cv2.Canny(binary_image, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return img

def process_and_show_image(data):
    """
    处理图像消息的主要函数，进行二值化处理和直线检测。
    """
    cv_image = imgmsg_to_cv2(data)
    if cv_image is not None:
        binary_image = process_binary_image(cv_image)
        result_image = detect_lines(binary_image, cv_image)
        # 调整图像大小
        resized_image = cv2.resize(result_image, (640, 480))  # 可以调整这里的尺寸以适应显示需求
        cv2.imshow("Detected Lines", resized_image)
        cv2.waitKey(3)
    print("Lines detected successfully")

def setup_subscriber():
    """
    设置ROS节点和图像订阅者。
    """
    rospy.init_node('line_detector', anonymous=True)
    rospy.Subscriber("/cam", Image, process_and_show_image)

def main():
    """
    主函数,启动ROS节点和图像订阅。
    """
    setup_subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
