import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()
path_publisher = None

def detect_curves_and_midline(image):
    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 使用阈值进行二值化处理
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    # 找到轮廓
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # 分析每个轮廓，寻找和拟合曲线
    fitted_curves = []
    for contour in contours:
        # 使用多项式拟合轮廓点
        if len(contour) > 5:  # 至少需要5个点来拟合一个圆
            ellipse = cv2.fitEllipse(contour)
            fitted_curves.append(ellipse)
    
    if len(fitted_curves) >= 2:
        # 假设拟合的前两个椭圆代表两条主要曲线
        ellipse1, ellipse2 = sorted(fitted_curves, key=lambda e: e[0][0])[:2]
        midline_points = []
        for y in range(image.shape[0]):
            # 计算两个椭圆中心的中点
            mid_x = int((ellipse1[0][0] + ellipse2[0][0]) / 2)
            midline_points.append((mid_x, y))
        return midline_points
    return []

def image_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    midline_points = detect_curves_and_midline(cv_image)
    # 创建和发布路径消息
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "camera_frame"
    for x, y in midline_points:
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = y
        pose.pose.position.y = x
        pose.pose.position.z = 0
        path_msg.poses.append(pose)
    path_publisher.publish(path_msg)

def setup_ros_nodes():
    global path_publisher
    rospy.init_node('curve_midline_to_path', anonymous=True)
    rospy.Subscriber("/cam", Image, image_callback)
    path_publisher = rospy.Publisher("/path", Path, queue_size=10)

def main():
    setup_ros_nodes()
    rospy.spin()

if __name__ == '__main__':
    main()
