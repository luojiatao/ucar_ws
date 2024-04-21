import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()
path_publisher = None

def detect_edges_and_midline(binary_image):
    height, width = binary_image.shape
    midline_points = []

    for i in range(height):
        row = binary_image[i, :]
        left_edge = np.argmax(row[:width // 2])
        right_edge = np.argmax(row[width // 2:]) + width // 2
        if left_edge > 0 and right_edge > width // 2:
            midline_point = (left_edge + right_edge) // 2
            midline_points.append((midline_point, i))

    return midline_points

def image_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
    midline_points = detect_edges_and_midline(binary_image)

    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "camera_frame"

    height, width = binary_image.shape

    for x, y in midline_points:
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = -(y - height / 2)  # x坐标转换，考虑图像中线为x轴
        pose.pose.position.y = -(x - width / 2)   # y坐标转换，考虑图像水平方向为y轴
        pose.pose.position.z = 0
        path_msg.poses.append(pose)

    path_publisher.publish(path_msg)

def setup_ros_nodes():
    global path_publisher
    rospy.init_node('midline_to_path', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, image_callback)
    path_publisher = rospy.Publisher("/path", Path, queue_size=10)

def main():
    setup_ros_nodes()
    rospy.spin()

if __name__ == '__main__':
    main()
