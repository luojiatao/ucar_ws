import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# 初始化CvBridge
bridge = CvBridge()

def detect_edges_and_midline(binary_image):
    height, width = binary_image.shape
    # 修改为三通道图像以支持彩色绘制
    result_image = np.zeros((height, width, 3), dtype=np.uint8)  

    for i in range(height):
        left_edge = np.argmax(binary_image[i, :width // 2])
        right_edge = np.argmax(binary_image[i, width // 2:]) + width // 2
        if left_edge > 0:
            # 在结果图像上绘制左边缘，保持为白色，增加线条宽度
            cv2.line(result_image, (left_edge, i), (left_edge, i), (255, 255, 255), thickness=5)
        if right_edge > width // 2:
            # 在结果图像上绘制右边缘，保持为白色，增加线条宽度
            cv2.line(result_image, (right_edge, i), (right_edge, i), (255, 255, 255), thickness=5)
        if left_edge and right_edge:
            midline_point = (left_edge + right_edge) // 2
            # 在结果图像上绘制中线，用红色，增加线条宽度
            cv2.line(result_image, (midline_point, i), (midline_point, i), (0, 0, 255), thickness=5)

    return result_image  # 返回包含边缘和中线的结果图像

def image_callback(data):
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    # 图像二值化
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

    # 检测边缘和中线
    result_image = detect_edges_and_midline(binary_image)

    # 显示结果图像
    result_image = cv2.resize(result_image, (640, 480))  # 可以调整这里的尺寸以适应显示需求
    cv2.imshow("Result", result_image)
    cv2.waitKey(3)

def setup_subscriber():
    """
    设置ROS节点和图像订阅者。
    """
    rospy.init_node('curve_detector', anonymous=True)
    rospy.Subscriber("/cam", Image, image_callback)

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
