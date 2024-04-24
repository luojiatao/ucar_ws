import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# 创建CvBridge对象，用于ROS和OpenCV图像格式之间的转换
bridge = CvBridge()

# 路径发布者
path_publisher = None

# 定义检测图像中的边缘和中线的函数
def detect_edges_and_midline(binary_image):
    height, width = binary_image.shape
    midline_points = []
    # 遍历图像的每一行
    for i in range(height):
        row = binary_image[i, :]
        # 寻找左边缘
        left_edge = np.argmax(row[:width // 2])
        # 寻找右边缘
        right_edge = np.argmax(row[width // 2:]) + width // 2
        # 如果找到有效的边缘，则计算中线点
        if left_edge > 0 and right_edge > width // 2:
            midline_point = (left_edge + right_edge) // 2
            midline_points.append((midline_point, i))
    return midline_points

# 定义图像接收回调函数
def image_callback(data):
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    # 将图像转换为灰度图
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # 图像二值化
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
    # 检测中线点
    midline_points = detect_edges_and_midline(binary_image)
    # 创建路径消息
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "camera_frame"
    height, width = binary_image.shape
    for x, y in midline_points:
        pose = PoseStamped()
        pose.header = path_msg.header
        # 坐标转换
        pose.pose.position.x = -(y - height / 2)
        pose.pose.position.y = -(x - width / 2)
        pose.pose.position.z = 0
        path_msg.poses.append(pose)
    # 发布路径消息
    path_publisher.publish(path_msg)

# 定义ROS节点初始化函数
def setup_ros_nodes():
    global path_publisher
    rospy.init_node('midline_to_path', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    path_publisher = rospy.Publisher("/path", Path, queue_size=10)

# 主函数
def main():
    setup_ros_nodes()
    rospy.spin()

if __name__ == '__main__':
    main()
